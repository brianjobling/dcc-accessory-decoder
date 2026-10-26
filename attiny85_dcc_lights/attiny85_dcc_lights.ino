// DCC head/tail light decoder - bare ATtiny85 (DIP-8, 8 MHz internal).
// Board: ATTinyCore > ATtiny85, 8 MHz (internal). Program with Arduino-as-ISP.
//
// Pins (ATTinyCore numbering = PBx):
//   PB2 (pin 7)  DCC in from optocoupler output (NEVER connect directly to track)
//   PB0 (pin 5)  LED leg A \  bi-colour LED with ONE series resistor (150-330 R)
//   PB1 (pin 6)  LED leg B /  between the pins; one pin sources, the other sinks
//   PB3 (pin 2)  optional TX-only debug serial (9600 8N1), see DEBUG_SERIAL
//
// A high / B low = white, A low / B high = red (swap wires if reversed).

#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

#define DEBUG_SERIAL 0            // 1 = report edge rate / packets / loco activity on PB3

#define DEFAULT_ADDRESS 10
#define PIN_LED_A 0
#define PIN_LED_B 1
#define PIN_DEBUG_TX 3
#define DCC_PIN_BIT PB2

#define EE_ADDR_ADDRESS 0
#define EE_ADDR_STATE 1
#define STATE_MARK 0xA0           // upper bits of state byte mark it as valid
#define ST_F0 0x01
#define ST_F1 0x02
#define ST_FWD 0x04

#define PWM_PERIOD_US 2000UL      // 500 Hz
#define PWM_ON_US 500UL           // F1 dimmed brightness (25 %)

#define RING_SIZE 64              // power of two; entries are 1 byte
#define RING_MASK (RING_SIZE - 1)

// Half-bit durations are stored in 4 us units (clipped to 255).
#define HALF_ONE_MIN 10           // 40 us
#define HALF_ONE_MAX 20           // 80 us
#define HALF_ZERO_MIN 23          // 92 us (anything longer, up to the clip, is a 0 half)

volatile uint8_t ring[RING_SIZE];
volatile uint8_t ringHead = 0;
volatile uint8_t ringTail = 0;
volatile uint16_t edgeCount = 0;
volatile uint8_t haveLastEdge = 0;
volatile uint32_t lastEdge = 0;

uint8_t myAddress = DEFAULT_ADDRESS;
uint8_t state = ST_F0;            // default: lights on, reverse (red)
uint8_t lastSaved = 0;

uint16_t packetCount = 0;
uint16_t locoCount = 0;

ISR(PCINT0_vect) {
  uint32_t now = micros();
  edgeCount++;
  if (haveLastEdge) {
    uint32_t d = (now - lastEdge) >> 2;
    uint8_t next = (ringHead + 1) & RING_MASK;
    if (next != ringTail) {       // drop on overflow
      ring[ringHead] = d > 255 ? 255 : (uint8_t)d;
      ringHead = next;
    }
  }
  haveLastEdge = 1;
  lastEdge = now;
}

// ---- LED output ----------------------------------------------------------
void ledOff() {
  digitalWrite(PIN_LED_A, LOW);
  digitalWrite(PIN_LED_B, LOW);
}

void updateLed() {
  if (!(state & ST_F0)) { ledOff(); return; }
  uint8_t src = (state & ST_FWD) ? PIN_LED_A : PIN_LED_B;
  uint8_t snk = (state & ST_FWD) ? PIN_LED_B : PIN_LED_A;
  digitalWrite(snk, LOW);
  if (state & ST_F1) {            // software PWM, no timers touched
    uint32_t phase = micros() % PWM_PERIOD_US;
    digitalWrite(src, phase < PWM_ON_US ? HIGH : LOW);
  } else {
    digitalWrite(src, HIGH);
  }
}

void saveState() {
  uint8_t v = STATE_MARK | (state & 0x07);
  if (v != lastSaved) { EEPROM.update(EE_ADDR_STATE, v); lastSaved = v; }
}

void loadState() {
  uint8_t v = EEPROM.read(EE_ADDR_STATE);
  if ((v & 0xF8) == STATE_MARK) { state = v & 0x07; lastSaved = v; }
  uint8_t a = EEPROM.read(EE_ADDR_ADDRESS);
  if (a >= 1 && a <= 127) myAddress = a;
}

// ---- Debug serial (TX only, bit-banged 9600 8N1) ---------------------------
#if DEBUG_SERIAL
void dbgByte(uint8_t c) {
  uint8_t s = SREG; cli();
  digitalWrite(PIN_DEBUG_TX, LOW); delayMicroseconds(104);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(PIN_DEBUG_TX, (c >> i) & 1); delayMicroseconds(104);
  }
  digitalWrite(PIN_DEBUG_TX, HIGH); delayMicroseconds(104);
  SREG = s;
}
void dbgStr(const char *s) { while (*s) dbgByte(*s++); }
void dbgNum(uint16_t n) {
  char b[6]; uint8_t i = 6; b[--i] = 0;
  do { b[--i] = '0' + n % 10; n /= 10; } while (n);
  dbgStr(b + i);
}
void debugReport() {
  static uint32_t t = 0;
  static uint16_t lastEdges = 0;
  if (millis() - t < 1000) return;
  t = millis();
  uint8_t s = SREG; cli(); uint16_t e = edgeCount; SREG = s;
  dbgStr("edges/s="); dbgNum(e - lastEdges);
  dbgStr(" pkts="); dbgNum(packetCount);
  dbgStr(" loco="); dbgNum(locoCount);
  dbgStr(" st="); dbgNum(state);
  dbgStr("\r\n");
  lastEdges = e;
}
#else
inline void debugReport() {}
#endif

// ---- DCC packet decoding ---------------------------------------------------
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
      EEPROM.update(EE_ADDR_ADDRESS, myAddress);
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
  saveState();
}

// Half-bit -> bit -> byte state machine
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
  } else {                                      // separator / end bit
    if (bit) {
      handlePacket(pkt, pktLen);
      pState = S_PREAMBLE; onesCount = 1;       // this 1 starts the next preamble
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
  else { firstHalf = c; pState = S_PREAMBLE; onesCount = 0; } // lost sync
}

void setup() {
  CLKPR = 0x80; CLKPR = 0x00;                   // force /1 prescaler: true 8 MHz
  pinMode(PIN_LED_A, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  loadState();
  updateLed();                                  // instant-on, before anything else
#if DEBUG_SERIAL
  pinMode(PIN_DEBUG_TX, OUTPUT);
  digitalWrite(PIN_DEBUG_TX, HIGH);
#endif
  pinMode(DCC_PIN_BIT, INPUT);
  GIMSK |= _BV(PCIE);
  PCMSK |= _BV(PCINT2);
}

void loop() {
  while (ringTail != ringHead) {
    uint8_t d = ring[ringTail];
    ringTail = (ringTail + 1) & RING_MASK;
    feedHalf(d);
  }
  updateLed();
  debugReport();
}
