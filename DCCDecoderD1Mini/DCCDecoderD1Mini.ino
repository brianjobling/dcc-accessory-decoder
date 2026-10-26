/* DCCDecoderD1Mini
 *
 * DCC decoder for a Wemos D1 mini (ESP8266), ported from the ESP32-C3 version
 * (https://github.com/brianjobling/dcc-decoder). Listens to the DCC track
 * signal through a 6N137 optocoupler circuit and drives a red/white bi-colour
 * LED through a DRV8833 H-bridge as directional head/tail lights:
 *
 *   F0 on,  forward -> white     F0 on, reverse -> red
 *   F0+F1 on        -> same colour, dimmed
 *   F0 off          -> lights off
 *
 * The watched loco address is programmable through the rails and saved to
 * EEPROM: write the address on the programming track (<W n> in DCC-EX), or
 * throw accessory 999 then toggle F0 on the wanted loco, or type the number
 * into the serial monitor.
 *
 * DO NOT CONNECT DIRECT TO TRACK - use the isolation circuit from
 * https://www.digitaltown.co.uk/79DCCDecoderCircuit.php (3.3V supply).
 *
 * Pins (D1 mini):
 *   D5 (GPIO14)  DCC signal in from the optocoupler
 *   D1 (GPIO5)   DRV8833 IN1
 *   D2 (GPIO4)   DRV8833 IN2
 *   D4 (GPIO2)   onboard status LED (active low)
 *   D3 (GPIO0)   optional learn button to GND
 */

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <EEPROM.h>

//---- pins ----
const int dccpin = 14;         //D5: DCC signal in
const int drvIn1 = 5;          //D1: DRV8833 IN1
const int drvIn2 = 4;          //D2: DRV8833 IN2
const int ledpin = 2;          //D4: onboard status LED, active low
const int learnButtonPin = 0;  //D3: optional pushbutton to GND

//---- headlight tuning ----
/*duty 0-255 per colour so red/white brightness can be balanced in software.
If white/red come out swapped for your DRV8833/LED wiring, flip swapDir.*/
const byte whiteDuty = 255;
const byte redDuty = 255;
const byte dimDivisor = 4;     //F0+F1 dims to duty/dimDivisor
const byte swapDir = 1;        //1 = swap which output polarity is "forward" (this build: LED wired mirrored)

/*On this hardware the interrupt fires on every transition of the DCC signal
(half-bits): a "1" half is ~58us, a "0" half is ~100us+ (DCC-EX). Anything
under halfbittime is a 1-half.*/
int halfbittime = 88;

static byte  Preamble = 0;
static byte  PreambleCounter = 0;
static byte  PacketStart = 0;

volatile unsigned long ISREdgeCount = 0; //total edges seen, for diagnostics

/*Edge-to-edge times are captured in the ISR into a ring buffer so that no edge
is ever lost while loop() is busy. The head/tail are single bytes so the
256-entry buffer wraps automatically (~15ms of cushion at DCC edge rates).*/
volatile uint16_t ISRIntervals[256];
volatile byte ISRHead = 0;
volatile unsigned long ISRLevelReject = 0; //interrupts rejected as same-level echoes
static int  ISRLastLevel = -1;
static unsigned long ISRLastEdgeTime = 0;

//---- debugging ----
#define DEBUG_DCC 1 //set to 0 to silence debug output when you put the script live
int dbgLastAccAddr = -1;
int dbgLastDir = -1;
unsigned int dbgRepeatCount = 0;
unsigned long dbgLastPrintTime = 0;

//interval histogram + stage counters (updated in processDCC once synced)
unsigned int dbgIvUnder70 = 0;
unsigned int dbgIv70to110 = 0;
unsigned int dbgIv110to130 = 0;
unsigned int dbgIv130to180 = 0;
unsigned int dbgIvOver180 = 0;
unsigned int dbgGoodPackets = 0;
unsigned int dbgBitsOne = 0;
unsigned int dbgBitsZero = 0;
unsigned int dbgHalfMismatch = 0;
unsigned int dbgPreambles = 0;
unsigned int dbgPktAssembled = 0;
unsigned int dbgChecksumFail = 0;

//---- loco function watching ----
int watchLocoAddr = 8; //default; overridden by the EEPROM-saved value

/*Learn mode: throw accessory 999 (or press the button on D3), then toggle F0
on the loco you want. The first loco whose F0-F4 state CHANGES becomes the
watched loco - unambiguous even though DCC-EX rebroadcasts every session loco.
Also: any CV1 write heard on the programming track is adopted directly.*/
const int learnAccAddr = 999;
byte learnMode = 0;
unsigned long learnStartTime = 0;
const unsigned long learnTimeoutMs = 30000;
byte lastFG1ByAddr[128]; //per-address F0-F4 state seen during learn, 255 = none

//---- loco state from the decoder ----
byte curSpeed = 0;   //0-126
byte curDirFwd = 0;  //default reverse (red) until a packet says otherwise; 1 = forward
byte curF0 = 0;
byte curF1 = 0;
byte confirmFlashesLeft = 0; //>0 = acknowledgement blink in progress (2 per flash)

const unsigned int flashMs = 300;

//---- EEPROM persistence (ESP8266 has no Preferences) ----
const byte eepromMagic = 0x42;
void saveLocoAddrEEPROM(int addr){
  EEPROM.write(0, eepromMagic);
  EEPROM.write(1, (byte)addr);
  EEPROM.commit();
}
int loadLocoAddrEEPROM(int fallback){
  if(EEPROM.read(0) == eepromMagic){
    byte a = EEPROM.read(1);
    if(a >= 1 && a <= 127) return a;
  }
  return fallback;
}

/*Light state persists at EEPROM[2] (bit0=F0, bit1=F1, bit2=dirFwd) so a
track-contact dropout that resets the board puts the lights straight back
to how they were, instead of waiting for DCC-EX to resend the state.
Saved only when it changes; overwritten by the first packet heard.*/
byte packLightState(){
  return (curF0 ? 1 : 0) | (curF1 ? 2 : 0) | (curDirFwd ? 4 : 0);
}
void saveLightStateIfChanged(){
  static int lastSaved = -1;
  if(lastSaved < 0) lastSaved = EEPROM.read(2); //don't rewrite what's stored
  byte s = packLightState();
  if(s == lastSaved) return;
  lastSaved = s;
  EEPROM.write(0, eepromMagic);
  EEPROM.write(2, s);
  EEPROM.commit();
}
void loadLightStateEEPROM(){
  if(EEPROM.read(0) != eepromMagic) return;
  byte s = EEPROM.read(2);
  if(s > 7) return; //never written (0xFF from the factory)
  curF0 = s & 1;
  curF1 = (s >> 1) & 1;
  curDirFwd = (s >> 2) & 1;
}

//onboard status LED is active-low: brightness 0 = off, 255 = full
void setLed(byte brightness){
  analogWrite(ledpin, 255 - brightness);
}

/*DRV8833 slow-decay drive: hold one input high, PWM the other inverted, so the
outputs stay actively driven through the whole PWM cycle.*/
void driveBridgeA(byte duty){ //polarity A: IN1 side high
  analogWrite(drvIn1, 255);
  analogWrite(drvIn2, 255 - duty);
}
void driveBridgeB(byte duty){ //polarity B: IN2 side high
  analogWrite(drvIn2, 255);
  analogWrite(drvIn1, 255 - duty);
}
void bridgeOff(){
  analogWrite(drvIn1, 0);
  analogWrite(drvIn2, 0);
}

//This is the interrupt function checking when pin state changes
void IRAM_ATTR pinstate(){
  ISREdgeCount++;
  /*One real edge can arrive as a burst of interrupts. A real DCC edge always
  lands on the OPPOSITE level from the last accepted edge, so anything at the
  same level is an echo. The time check drops opposite-level noise inside a
  transition (real half-bits are never shorter than 58us).*/
  int lvl = digitalRead(dccpin);
  if(lvl == ISRLastLevel){
    ISRLevelReject++;
    return;
  }
  unsigned long now = micros();
  unsigned long delta = now - ISRLastEdgeTime;
  if(delta < 40) return; //too fast to be a real half-bit
  ISRLastLevel = lvl;
  ISRLastEdgeTime = now;
  if(delta > 65535) delta = 65535;
  ISRIntervals[ISRHead++] = delta; //byte index wraps at 256 automatically
}

//NOTE: the same packet is transmitted multiple times by the command station -
//user code here must be idempotent.
void ControlAccDecoder(byte index, byte dir, int AccAddr, int BoardAddr){

#if DEBUG_DCC
  if(AccAddr != dbgLastAccAddr || dir != dbgLastDir || millis() - dbgLastPrintTime > 1000){
    if(dbgRepeatCount > 0){
      Serial.print("          ...last command repeated ");
      Serial.print(dbgRepeatCount);
      Serial.println(" more time(s)");
    }
    Serial.print(millis());
    Serial.print("ms CMD AccAddr=");
    Serial.print(AccAddr);
    Serial.print(" dir=");
    Serial.print(dir);
    Serial.print(" BoardAddr=");
    Serial.print(BoardAddr);
    Serial.print(" index=");
    Serial.println(index);
    dbgLastAccAddr = AccAddr;
    dbgLastDir = dir;
    dbgRepeatCount = 0;
    dbgLastPrintTime = millis();
  }else{
    dbgRepeatCount++;
  }
#endif

  switch(AccAddr){
    case learnAccAddr: //throwing this "turnout" starts loco learn mode
      startLearnMode();
      break;
    default: //add your own accessory addresses here
      break;
  }
}

//Called for every valid function group one packet from the watched loco.
void ControlLocoFunction(byte LocoAddr, byte F0, byte F1to4){
  if(LocoAddr != watchLocoAddr) return;
  curF0 = F0 > 0;
  curF1 = F1to4 & 1;
}

/*headlight state machine, called every loop() pass: white forward / red
reverse when F0 is on, dimmed when F1 is also on. Only touches the bridge
when something changes.*/
void updateHeadlight(){
  static int applied = -1;
  int state; //encode direction+duty into one comparable value
  byte duty = 0;
  if(curF0){
    duty = curDirFwd ? whiteDuty : redDuty;
    if(curF1) duty = duty / dimDivisor;
  }
  state = duty == 0 ? 0 : ((curDirFwd ? 1 : 2) << 8) | duty;
  if(state == applied) return;
  applied = state;
  if(duty == 0){
    bridgeOff();
  }else if(curDirFwd ^ swapDir){
    driveBridgeA(duty); //forward colour (white); flip swapDir if it comes out red
  }else{
    driveBridgeB(duty);
  }
}

//status LED: learn flash / confirm blinks / heartbeat when idle
void updateStatusLed(){
  static unsigned long lastFlip = 0;
  static byte flashPhase = 0;
  static int applied = -1;
  int desired;

  if(confirmFlashesLeft > 0){ //acknowledgement blink: new address accepted
    if(millis() - lastFlip >= 120){
      lastFlip = millis();
      flashPhase = !flashPhase;
      confirmFlashesLeft--;
    }
    desired = flashPhase ? 255 : 0;
  }else if(learnMode){ //rapid flash while waiting for a loco
    if(millis() - lastFlip >= 100){
      lastFlip = millis();
      flashPhase = !flashPhase;
    }
    desired = flashPhase ? 255 : 0;
  }else{ //mirror F0 so there's a visible sign of life
    desired = curF0 ? 60 : 0;
  }

  if(desired != applied){
    applied = desired;
    setLed(desired);
  }
}

void startLearnMode(){
  if(learnMode) return; //already learning (DCC repeats commands many times)
  learnMode = 1;
  learnStartTime = millis();
  memset(lastFG1ByAddr, 255, sizeof(lastFG1ByAddr));
  Serial.println("LEARN MODE: toggle F0 on the loco you want (30s timeout)");
}

void handleLearnButton(){
  static unsigned long lastPress = 0;
  if(learnMode){
    if(millis() - learnStartTime > learnTimeoutMs){
      learnMode = 0;
      Serial.println("Learn mode timed out, no change");
    }
    return;
  }
  if(digitalRead(learnButtonPin) == LOW && millis() - lastPress > 500){
    lastPress = millis();
    startLearnMode();
  }
}

//accept a loco number typed into the serial monitor, without ever blocking
void handleSerialCommand(){
  static char cmdBuf[8];
  static byte cmdLen = 0;
  while(Serial.available()){
    char c = Serial.read();
    if(c == '\n' || c == '\r'){
      if(cmdLen > 0){
        cmdBuf[cmdLen] = 0;
        cmdLen = 0;
        int addr = atoi(cmdBuf);
        if(addr >= 1 && addr <= 127){
          learnMode = 0;
          setWatchLoco(addr);
        }else{
          Serial.println("Loco number must be 1-127");
        }
      }
    }else if(cmdLen < 7){
      cmdBuf[cmdLen++] = c;
    }
  }
}

void setWatchLoco(int addr){
  watchLocoAddr = addr;
  saveLocoAddrEEPROM(addr);
  curSpeed = 0;
  curF0 = 0;
  curF1 = 0;
  confirmFlashesLeft = 12; //6 quick blinks to confirm the new address was accepted
  resetLocoState(); //forget the old loco's last-seen state
  Serial.print("Now watching loco ");
  Serial.println(addr);
}

void setup() {
  /*Lights first, everything else after: restore the saved headlight state
  and drive the bridge before serial, WiFi teardown or boot blinks, so a
  momentary track-contact dropout barely blinks the lights.*/
  pinMode(drvIn1, OUTPUT);
  pinMode(drvIn2, OUTPUT);
  analogWriteRange(255);
  analogWriteFreq(1000);
  bridgeOff();
  EEPROM.begin(8);
  loadLightStateEEPROM();
  /*A powered dummy car should never sit dark: if the restored state has the
  lights off, show red (reverse) until the track says otherwise. The first
  packets heard overwrite this either way.*/
  if(!curF0){
    curF0 = 1;
    curDirFwd = 0;
  }
  updateHeadlight();       //lights back on within ms of power returning

  Serial.begin(115200);
  Serial.println();
  Serial.println("DCCDecoderD1Mini");

  //WiFi would add interrupt jitter and we never need it - hard off
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();

  watchLocoAddr = loadLocoAddrEEPROM(watchLocoAddr);
  Serial.print("Watching loco ");
  Serial.println(watchLocoAddr);
  Serial.println("To change: throw turnout/accessory 999, write the address on the");
  Serial.println("programming track (<W n>), or type a loco number (1-127) here.");

  pinMode(learnButtonPin, INPUT_PULLUP);
  pinMode(dccpin, INPUT_PULLUP);
  pinMode(ledpin, OUTPUT);

  for(int q = 0; q < 3; q++){ //blink 3 times at boot to show we're alive
    setLed(255);
    delay(150);
    setLed(0);
    delay(150);
  }

  attachInterrupt(digitalPinToInterrupt(dccpin), pinstate, CHANGE); //half-bit decode uses every transition
}

void loop() {
  processDCC();
  updateHeadlight();
  saveLightStateIfChanged();
  updateStatusLed();
  handleLearnButton();
  handleSerialCommand();

#if DEBUG_DCC
  //every 2s report whether DCC edges are arriving and how decoding is going.
  //A live DCC-EX feed gives roughly 17000 edges/sec and ~280 pkts per window.
  static unsigned long dbgLastEdgeReport = 0;
  static unsigned long dbgLastEdgeCount = 0;
  if(millis() - dbgLastEdgeReport >= 2000){
    unsigned long edges = ISREdgeCount;
    Serial.print(millis());
    Serial.print("ms DIAG edges/sec=");
    Serial.print((edges - dbgLastEdgeCount) / 2);
    Serial.print(" echo=");
    Serial.print(ISRLevelReject);
    Serial.print(" pkts=");
    Serial.print(dbgGoodPackets);
    Serial.print(" iv[<70,<110,<130,<180,>180]=");
    Serial.print(dbgIvUnder70); Serial.print(",");
    Serial.print(dbgIv70to110); Serial.print(",");
    Serial.print(dbgIv110to130); Serial.print(",");
    Serial.print(dbgIv130to180); Serial.print(",");
    Serial.print(dbgIvOver180);
    Serial.print(" bits1=");
    Serial.print(dbgBitsOne);
    Serial.print(" bits0=");
    Serial.print(dbgBitsZero);
    Serial.print(" mism=");
    Serial.print(dbgHalfMismatch);
    Serial.print(" pre=");
    Serial.print(dbgPreambles);
    Serial.print(" asm=");
    Serial.print(dbgPktAssembled);
    Serial.print(" cksfail=");
    Serial.println(dbgChecksumFail);
    dbgIvUnder70 = dbgIv70to110 = dbgIv110to130 = dbgIv130to180 = dbgIvOver180 = 0;
    dbgGoodPackets = 0;
    dbgBitsOne = dbgBitsZero = dbgHalfMismatch = 0;
    dbgPreambles = dbgPktAssembled = dbgChecksumFail = 0;
    dbgLastEdgeCount = edges;
    dbgLastEdgeReport = millis();
  }
#endif
}
