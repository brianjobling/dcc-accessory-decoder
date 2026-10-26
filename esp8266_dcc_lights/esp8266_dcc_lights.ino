// DCC head/tail light decoder - Wemos D1 mini (ESP8266) + DRV8833 H-bridge.
// Same behaviour as the ATtiny85 version.
//
// Wiring:
//   D2 (GPIO4)  DCC in from optocoupler output (NEVER connect directly to track)
//   D5 (GPIO14) DRV8833 AIN1 \  bi-colour LED across AOUT1/AOUT2 with a series
//   D6 (GPIO12) DRV8833 AIN2 /  resistor; DRV8833 nSLEEP tied high
// AIN1 high / AIN2 low = white, AIN1 low / AIN2 high = red (swap wires if reversed).

#include <ESP8266WiFi.h>
#include <EEPROM.h>

#define DEBUG_SERIAL 0            // 1 = report edge rate / packets / loco activity at 115200

#define DEFAULT_ADDRESS 10
#define PIN_DCC D2
#define PIN_IN1 D5
#define PIN_IN2 D6

#define EE_ADDR_ADDRESS 0
#define EE_ADDR_STATE 1
#define STATE_MARK 0xA0
#define ST_F0 0x01
#define ST_F1 0x02
#define ST_FWD 0x04

#define DIM_LEVEL 256             // of 1023 (analogWrite range set below)

#define RING_SIZE 128             // power of two
#define RING_MASK (RING_SIZE - 1)

// Half-bit durations stored in 4 us units (clipped to 255).
#define HALF_ONE_MIN 10
#define HALF_ONE_MAX 20
#define HALF_ZERO_MIN 23

volatile uint8_t ring[RING_SIZE];
volatile uint8_t ringHead = 0;
volatile uint8_t ringTail = 0;
volatile uint32_t edgeCount = 0;
volatile bool haveLastEdge = false;
volatile uint32_t lastEdge = 0;

uint8_t myAddress = DEFAULT_ADDRESS;
uint8_t state = ST_F0;            // default: lights on, reverse (red)
uint8_t lastSaved = 0;
uint32_t packetCount = 0;
uint32_t locoCount = 0;

void ICACHE_RAM_ATTR onEdge() {
  uint32_t now = micros();
  edgeCount++;
  if (haveLastEdge) {
    uint32_t d = (now - lastEdge) >> 2;
    uint8_t next = (ringHead + 1) & RING_MASK;
    if (next != ringTail) {
      ring[ringHead] = d > 255 ? 255 : (uint8_t)d;
      ringHead = next;
    }
  }
  haveLastEdge = true;
  lastEdge = now;
}

void updateLed() {
  if (!(state & ST_F0)) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    return;
  }
  uint8_t src = (state & ST_FWD) ? PIN_IN1 : PIN_IN2;
  uint8_t snk = (state & ST_FWD) ? PIN_IN2 : PIN_IN1;
  analogWrite(snk, 0);
  if (state & ST_F1) analogWrite(src, DIM_LEVEL);
  else digitalWrite(src, HIGH);
}

void saveState() {
  uint8_t v = STATE_MARK | (state & 0x07);
  if (v != lastSaved) {
    EEPROM.write(EE_ADDR_STATE, v);
    EEPROM.commit();
    lastSaved = v;
  }
}

void loadState() {
  uint8_t v = EEPROM.read(EE_ADDR_STATE);
  if ((v & 0xF8) == STATE_MARK) { state = v & 0x07; lastSaved = v; }
  uint8_t a = EEPROM.read(EE_ADDR_ADDRESS);
  if (a >= 1 && a <= 127) myAddress = a;
}

uint8_t pkt[6];
uint8_t pktLen;
uint8_t prevPkt[3];
bool havePrev = false;

void handlePacket(const uint8_t *p, uint8_t len) {
  uint8_t x = 0;
  for (uint8_t i = 0; i < len; i++) x ^= p[i];
  if (len < 3 || x != 0) return;
  packetCount++;

  // Service-mode direct-byte write: 0111 11AA AAAAAAAA DDDDDDDD (programming track)
  if (len == 4 && (p[0] & 0xFC) == 0x7C) {
    bool same = havePrev && prevPkt[0] == p[0] && prevPkt[1] == p[1] && prevPkt[2] == p[2];
    prevPkt[0] = p[0]; prevPkt[1] = p[1]; prevPkt[2] = p[2]; havePrev = true;
    if (same && p[0] == 0x7C && p[1] == 0x00 && p[2] >= 1 && p[2] <= 127 && p[2] != myAddress) {
      myAddress = p[2];           // CV1 write
      EEPROM.write(EE_ADDR_ADDRESS, myAddress);
      EEPROM.commit();
    }
    return;
  }

  if (p[0] != myAddress) return;
  locoCount++;
  uint8_t b1 = p[1];
  if (b1 == 0x3F && len >= 4) {                 // 128-step speed
    state = (p[2] & 0x80) ? (state | ST_FWD) : (state & ~ST_FWD);
  } else if ((b1 & 0xC0) == 0x40) {             // 14/28-step speed
    state = (b1 & 0x20) ? (state | ST_FWD) : (state & ~ST_FWD);
  } else if ((b1 & 0xE0) == 0x80) {             // function group 1
    state = (state & ~(ST_F0 | ST_F1)) | ((b1 & 0x10) ? ST_F0 : 0) | ((b1 & 0x01) ? ST_F1 : 0);
  } else {
    return;
  }
  updateLed();
  saveState();
}

enum { S_PREAMBLE, S_DATA };
uint8_t pState = S_PREAMBLE;
uint8_t onesCount = 0;
uint8_t curByte = 0, bitIdx = 0;
bool haveHalf = false;
uint8_t firstHalf = 0;

void resetParser() {
  pState = S_PREAMBLE; onesCount = 0; haveHalf = false; pktLen = 0; bitIdx = 0; curByte = 0;
}

void feedBit(uint8_t bit) {
  if (pState == S_PREAMBLE) {
    if (bit) { if (onesCount < 255) onesCount++; }
    else if (onesCount >= 10) { pState = S_DATA; pktLen = 0; bitIdx = 0; curByte = 0; }
    else onesCount = 0;
    return;
  }
  if (bitIdx < 8) {
    curByte = (curByte << 1) | bit;
    if (++bitIdx == 8) {
      if (pktLen >= sizeof(pkt)) { resetParser(); return; }
      pkt[pktLen++] = curByte;
    }
  } else {
    if (bit) {
      handlePacket(pkt, pktLen);
      pState = S_PREAMBLE; onesCount = 1;
    } else {
      bitIdx = 0; curByte = 0;
    }
  }
}

void feedHalf(uint8_t d) {
  uint8_t c;
  if (d >= HALF_ONE_MIN && d <= HALF_ONE_MAX) c = 1;
  else if (d >= HALF_ZERO_MIN) c = 0;
  else { resetParser(); return; }
  if (!haveHalf) { firstHalf = c; haveHalf = true; }
  else if (c == firstHalf) { haveHalf = false; feedBit(c); }
  else { firstHalf = c; pState = S_PREAMBLE; onesCount = 0; }
}

void setup() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  analogWriteRange(1023);
  EEPROM.begin(16);
  loadState();
  updateLed();                                  // instant-on, before serial/WiFi
  WiFi.mode(WIFI_OFF);
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif
  pinMode(PIN_DCC, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DCC), onEdge, CHANGE);
}

void loop() {
  while (ringTail != ringHead) {
    uint8_t d = ring[ringTail];
    ringTail = (ringTail + 1) & RING_MASK;
    feedHalf(d);
  }
#if DEBUG_SERIAL
  static uint32_t t = 0, lastEdges = 0;
  if (millis() - t >= 1000) {
    t = millis();
    uint32_t e = edgeCount;
    Serial.printf("edges/s=%lu pkts=%lu loco=%lu st=%u addr=%u\n",
                  (unsigned long)(e - lastEdges), (unsigned long)packetCount,
                  (unsigned long)locoCount, state, myAddress);
    lastEdges = e;
  }
#endif
  yield();
}
