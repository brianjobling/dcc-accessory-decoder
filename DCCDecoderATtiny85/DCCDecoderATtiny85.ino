/* DCCDecoderATtiny85 - full DCC headlight decoder
 *
 * Port of the D1 mini HST headlight decoder (DCCDecoderD1Mini) to a bare
 * ATtiny85 running at 8 MHz internal. Listens to the DCC track signal through
 * the optocoupler front-end and drives a red/white bi-colour LED DIRECTLY from
 * two pins (no DRV8833) as directional head/tail lights:
 *
 *   F0 on,  forward -> white     F0 on, reverse -> red
 *   F0+F1 on        -> same colour, dimmed (software PWM)
 *   F0 off          -> lights off
 *
 * Watched loco defaults to 10. It can still be changed at runtime by writing
 * CV1 on a programming track (DCC-EX <W n>), which is adopted and saved to
 * EEPROM. Light state (F0/F1/direction) is also saved to EEPROM so a momentary
 * track dropout restores the lights within milliseconds of power returning.
 *
 * ATtiny85 pinout (DIP-8), Arduino pin numbers in brackets:
 *   1  PB5 / RESET        (5)   -> 10k pull-up to VCC
 *   2  PB3               (3)   <- DCC signal in from the optocoupler  (PCINT3)
 *   3  PB4               (4)   -- free
 *   4  GND
 *   5  PB0 / OC0A        (0)   <- LED_A
 *   6  PB1 / OC0B        (1)   <- LED_B
 *   7  PB2               (2)   -- free (SCK during ISP)
 *   8  VCC
 *
 * Program with an ISP (e.g. "Arduino as ISP") at 8 MHz internal clock.
 *
 * DO NOT CONNECT DIRECT TO TRACK - use the optocoupler isolation front-end.
 */

#include <EEPROM.h>
#include <avr/interrupt.h>

//---- pins ----
const uint8_t ledA = 0;     //PB0, physical pin 5
const uint8_t ledB = 1;     //PB1, physical pin 6
#define DCC_PIN_BIT PB3     //PB3, physical pin 2 (read straight from PINB in the ISR)

/*---- debug serial (set DBG 1 to build the diagnostic version) --------------
TX-only bit-banged serial on PB4 (physical pin 3) at 9600 baud. Read it with an
FTDI232: FTDI RX <- PB4, FTDI GND <- decoder GND, 9600 8N1. It is hand-rolled
(not SoftwareSerial) because SoftwareSerial would grab the same PCINT0 vector the
DCC input uses. Printing briefly disables interrupts, so a few DCC edges are lost
during each once-a-second snapshot - harmless, DCC repeats constantly.*/
#define DBG 0
#if DBG
#include <util/delay.h>
#define TX_BIT PB4
#define TX_HIGH() (PORTB |=  (1 << TX_BIT))
#define TX_LOW()  (PORTB &= ~(1 << TX_BIT))
/*9600 baud = 104.2us/bit (valid now the clock is forced to a true 8 MHz).
_delay_us is a compile-time-constant busy loop, far more accurate than
delayMicroseconds, so the UART locks cleanly.*/
#define DBG_BIT_US 103

volatile unsigned long dbgEdges = 0; //raw ISR edges accepted
unsigned int dbgGood = 0;   //packets passing checksum
unsigned int dbgShort = 0;  //short-address (bit7==0) packets
unsigned int dbgLong = 0;   //long-address (0xC0-0xE7) packets
unsigned int dbgL10 = 0;    //packets addressed to the watched loco
int dbgLastInstr = -1;      //last instruction byte seen for the watched loco

void txByte(uint8_t b) {
  uint8_t s = SREG; cli();
  TX_LOW(); _delay_us(DBG_BIT_US);                   //start bit
  for (uint8_t i = 0; i < 8; i++) {
    if (b & 1) TX_HIGH(); else TX_LOW();
    b >>= 1;
    _delay_us(DBG_BIT_US);
  }
  TX_HIGH(); _delay_us(DBG_BIT_US);                  //stop bit
  SREG = s;
}
void txStr(const char *s) { while (*s) txByte((uint8_t)*s++); }
void txNum(long n) { char b[12]; ltoa(n, b, 10); txStr(b); }
#endif

//---- headlight behaviour ----
/*If white/red come out swapped for your LED wiring, flip SWAP_COLOURS to 1.*/
#define SWAP_COLOURS 0
const uint8_t dimDivisor = 4;        //F0+F1 dims to roughly 1/dimDivisor brightness
const unsigned long dimPeriodUs = 2000UL; //software-PWM period for the dim mode (~500 Hz)

/*Every transition of the DCC signal is a half-bit: a "1" half is ~58us, a "0"
half is ~100us+. Anything under halfbittime counts as a 1-half.*/
const unsigned int halfbittime = 88;

//---- DCC decode state ----
static byte Preamble = 0;
static byte PreambleCounter = 0;
static byte PacketStart = 0;

/*Edge-to-edge times captured by the ISR into a ring buffer so no edge is lost
while loop() works. 64 entries at ~58us each is ~3.7ms of cushion - plenty with
no blocking serial in loop(). Size is a power of two so the index masks cheaply.*/
#define RING_SIZE 64
#define RING_MASK (RING_SIZE - 1)
volatile uint16_t ISRIntervals[RING_SIZE];
volatile byte ISRHead = 0;
static byte ISRLastLevel = 2;            //2 = "unknown" so the first edge always takes
static unsigned long ISRLastEdgeTime = 0;

//packet assembly (3 bytes for functions/accessories, 4+ for 128-step speed)
static byte pktBytes[6];
static byte pktByteCount = 0;
static byte curByte = 0;
static byte bitInByte = 0;
static byte pendingHalf = 255; //first half of the bit being assembled, 255 = none

//---- loco state ----
int watchLocoAddr = 10;   //default; overridden by an EEPROM-saved value if present
byte curDirFwd = 0;       //1 = forward (white), 0 = reverse (red)
byte curF0 = 0;
byte curF1 = 0;

//change-detection so we don't re-parse identical repeated packets needlessly
static int lastSpeed128 = -1;
static int last28Speed = -1;
static int lastFG1 = -1;

//---- EEPROM persistence (addr0 magic, addr1 loco, addr2 light state) ----
const byte eepromMagic = 0x42;

void saveLocoAddrEEPROM(int addr) {
  EEPROM.update(0, eepromMagic);
  EEPROM.update(1, (byte)addr);
}
int loadLocoAddrEEPROM(int fallback) {
  if (EEPROM.read(0) == eepromMagic) {
    byte a = EEPROM.read(1);
    if (a >= 1 && a <= 127) return a;
  }
  return fallback;
}

byte packLightState() {
  return (curF0 ? 1 : 0) | (curF1 ? 2 : 0) | (curDirFwd ? 4 : 0);
}
void saveLightStateIfChanged() {
  static int lastSaved = -1;
  if (lastSaved < 0) lastSaved = EEPROM.read(2);
  byte s = packLightState();
  if (s == lastSaved) return;
  lastSaved = s;
  EEPROM.update(0, eepromMagic);
  EEPROM.update(2, s);
}
void loadLightStateEEPROM() {
  if (EEPROM.read(0) != eepromMagic) return;
  byte s = EEPROM.read(2);
  if (s > 7) return; //never written (0xFF from the factory)
  curF0 = s & 1;
  curF1 = (s >> 1) & 1;
  curDirFwd = (s >> 2) & 1;
}

//---- LED drive -------------------------------------------------------------
//ledMode: 0 off, 1 white full, 2 red full, 3 white dim, 4 red dim
volatile byte ledMode = 0;

static inline void setPins(byte aHigh, byte bHigh) {
  digitalWrite(ledA, aHigh ? HIGH : LOW);
  digitalWrite(ledB, bHigh ? HIGH : LOW);
}

//soft-PWM the sourcing pin; the other pin stays low. aSource picks which colour.
static void softDim(byte aSource) {
  unsigned long ph = micros() % dimPeriodUs;
  byte on = ph < (dimPeriodUs / dimDivisor);
  if (aSource) setPins(on, 0); else setPins(0, on);
}

//called every loop pass - applies ledMode, doing the soft-PWM for dim modes
void serviceLed() {
  switch (ledMode) {
    case 1: setPins(1, 0); break; //white full
    case 2: setPins(0, 1); break; //red full
    case 3: softDim(1);    break; //white dim
    case 4: softDim(0);    break; //red dim
    default: setPins(0, 0);       //off
  }
}

//work out the wanted mode from F0/F1/direction; only changes ledMode
void updateHeadlight() {
  byte mode;
  if (!curF0) {
    mode = 0;
  } else {
    byte fwd = curDirFwd ^ SWAP_COLOURS; //forward -> white
    if (curF1) mode = fwd ? 3 : 4;       //dimmed
    else       mode = fwd ? 1 : 2;       //full
  }
  ledMode = mode;
}

//---- DCC input interrupt (pin-change on PB3 / PCINT3) ----------------------
ISR(PCINT0_vect) {
  //one real edge can arrive as a burst; a real DCC edge is always the OPPOSITE
  //level from the last accepted one, so same-level interrupts are echoes.
  byte lvl = (PINB >> DCC_PIN_BIT) & 1;
  if (lvl == ISRLastLevel) return;
  unsigned long now = micros();
  unsigned long delta = now - ISRLastEdgeTime;
  if (delta < 40) return; //too fast to be a real half-bit
  ISRLastLevel = lvl;
  ISRLastEdgeTime = now;
  if (delta > 65535) delta = 65535;
  ISRIntervals[ISRHead++ & RING_MASK] = (uint16_t)delta;
#if DBG
  dbgEdges++;
#endif
}

//---- packet handling -------------------------------------------------------
void processpacket() {
  if (pktByteCount < 3) return; //too short

  byte checksum = 0;
  for (byte q = 0; q < pktByteCount - 1; q++) checksum ^= pktBytes[q];
  if (checksum != pktBytes[pktByteCount - 1]) return; //failed checksum

  byte AddressByte = pktBytes[0];
#if DBG
  dbgGood++;
  if (AddressByte != 0 && AddressByte < 0x80) dbgShort++;
  else if (AddressByte >= 0xC0 && AddressByte <= 0xE7) dbgLong++;
  if ((AddressByte & 0x80) == 0 && AddressByte == watchLocoAddr) {
    dbgL10++;
    dbgLastInstr = pktBytes[1];
  }
#endif
  if (AddressByte == 0 || AddressByte == 0xFF || AddressByte == 0xFE) return; //broadcast/idle/reserved

  //service-mode (programming track): 4 bytes, first byte 0111CCVV. Checked
  //before the loco branch because 0x70-0x7F would look like short addresses.
  if (pktByteCount == 4 && (AddressByte & 0b11110000) == 0b01110000) {
    //CC==11 is a byte write; CV = ((b0&3)<<8 | b1) + 1; data in b2
    byte cc = (pktBytes[0] >> 2) & 0b11;
    int cv = (((pktBytes[0] & 0b11) << 8) | pktBytes[1]) + 1;
    if (cc == 0b11 && cv == 1 && pktBytes[2] >= 1 && pktBytes[2] <= 127) {
      setWatchLoco(pktBytes[2]); //adopt a newly programmed address
    }
    return;
  }

  if (bitRead(AddressByte, 7) == 0) { //short loco address 1-127
    if (AddressByte == watchLocoAddr) LocoPacket();
  }
  //accessory decoders are ignored on this build (no learn mode)
}

//decode the parts of a watched-loco packet that drive the lights
void LocoPacket() {
  byte instr = pktBytes[1];

  if (instr == 0b00111111 && pktByteCount >= 4) {       //128-step speed: D SSSSSSS
    if ((int)pktBytes[2] != lastSpeed128) {
      lastSpeed128 = pktBytes[2];
      curDirFwd = bitRead(pktBytes[2], 7);              //direction bit -> colour
    }
  } else if ((instr >> 6) == 0b01) {                    //28-step speed: 01 D SSSSS
    if ((int)instr != last28Speed) {
      last28Speed = instr;
      curDirFwd = bitRead(instr, 5);
    }
  } else if ((instr >> 5) == 0b100) {                   //function group one: 100 F0 F4..F1
    if ((int)instr != lastFG1) {
      lastFG1 = instr;
      curF0 = bitRead(instr, 4);
      curF1 = bitRead(instr, 0);
    }
  }
  //other instruction groups (F5+ etc) don't affect the head/tail lights
}

/*Drain the ISR buffer, pair half-bits into bits, and run preamble detection
and byte assembly. Working in half-bits keeps the decoder polarity independent.*/
void processDCC() {
  static byte ISRTail = 0;
  byte thisBit;
  unsigned int interval;

  while (ISRTail != ISRHead) {
    interval = ISRIntervals[ISRTail++ & RING_MASK];

    if (interval > 12000) { //signal gap - start fresh
      pendingHalf = 255;
      Preamble = 0;
      PreambleCounter = 0;
      continue;
    }

    //two consecutive halves of the same kind make a bit; a mismatch means we're
    //out of phase, so keep the newest half and try again.
    byte half = (interval < halfbittime) ? 1 : 0;
    if (pendingHalf != half) {
      pendingHalf = half;
      continue;
    }
    pendingHalf = 255;
    thisBit = half;

    if (Preamble < 1) {                 //collecting the preamble
      if (thisBit == 1) PreambleCounter++;
      else PreambleCounter = 0;
      if (PreambleCounter > 9) {        //NMRA: >=10 "1" bits before a packet
        Preamble = 1;
        PacketStart = 0;
        PreambleCounter = 0;
      }
    } else if (PacketStart < 1) {       //wait for the packet-start 0 bit
      if (thisBit == 0) {
        PacketStart = 1;
        pktByteCount = 0;
        curByte = 0;
        bitInByte = 0;
      }
    } else {                            //collect bytes until end-of-packet
      if (bitInByte < 8) {
        curByte = (curByte << 1) | thisBit;
        bitInByte++;
      } else {                          //9th bit: 0 = more bytes, 1 = end
        pktBytes[pktByteCount++] = curByte;
        curByte = 0;
        bitInByte = 0;
        if (thisBit == 1) {             //end of packet
          processpacket();
          Preamble = 0;
          PreambleCounter = 0;
        } else if (pktByteCount >= 6) { //too long - resync
          Preamble = 0;
          PreambleCounter = 0;
        }
      }
    }
  }
}

void setWatchLoco(int addr) {
  watchLocoAddr = addr;
  saveLocoAddrEEPROM(addr);
  curF0 = 0;
  curF1 = 0;
  lastSpeed128 = last28Speed = lastFG1 = -1; //forget the old loco's state
}

void setup() {
  /*Force the system clock prescaler to /1 so we run at the full 8 MHz even if
  the CKDIV8 fuse is still programmed (factory default is 1 MHz). DCC half-bit
  timing and the ISR service rate depend on this - at 1 MHz the ISR is too slow
  to catch 58us edges and nothing decodes. Must set CLKPCE then CLKPR within 4
  cycles, which two back-to-back stores satisfy.*/
  CLKPR = 0x80; //enable prescaler change
  CLKPR = 0x00; //prescaler = 1 -> full clock

  /*Lights first: restore the saved state and drive the LED before setting up
  the DCC interrupt, so a momentary track dropout barely blinks the lights.*/
  pinMode(ledA, OUTPUT);
  pinMode(ledB, OUTPUT);
  setPins(0, 0);

  loadLightStateEEPROM();
  //a powered dummy car should never sit dark: if restored state is lights-off,
  //show red (reverse) until the track says otherwise.
  if (!curF0) {
    curF0 = 1;
    curDirFwd = 0;
  }
  updateHeadlight();
  serviceLed(); //lights on within ms of power returning

  watchLocoAddr = loadLocoAddrEEPROM(watchLocoAddr);

#if DBG
  DDRB |= (1 << TX_BIT); //PB4 (pin 3) as output for debug serial
  TX_HIGH();             //idle high
  delay(50);
  txStr("\r\nATtiny DCC watch="); txNum(watchLocoAddr); txStr("\r\n");
#endif

  //DCC input on PB3 with pull-up, pin-change interrupt PCINT3
  DDRB &= ~(1 << DCC_PIN_BIT); //input
  PORTB |= (1 << DCC_PIN_BIT); //pull-up
  GIMSK |= (1 << PCIE);        //enable pin-change interrupts
  PCMSK |= (1 << PCINT3);      //...on PB3 only
  sei();
}

void loop() {
  processDCC();
  updateHeadlight();
  serviceLed();
  saveLightStateIfChanged();

#if DBG
  //once-a-second snapshot: edges arriving? packets decoding? any for our loco?
  static unsigned long dbgLast = 0;
  if (millis() - dbgLast >= 1000) {
    dbgLast = millis();
    unsigned long e = dbgEdges; dbgEdges = 0;
    txStr("e="); txNum(e);
    txStr(" pk="); txNum(dbgGood);
    txStr(" sa="); txNum(dbgShort);
    txStr(" la="); txNum(dbgLong);
    txStr(" l10="); txNum(dbgL10);
    txStr(" instr="); txNum(dbgLastInstr);
    txStr(" F0="); txNum(curF0);
    txStr(" dir="); txNum(curDirFwd);
    txStr("\r\n");
    dbgGood = dbgShort = dbgLong = dbgL10 = 0;
  }
#endif
}
