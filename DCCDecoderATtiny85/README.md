# DCCDecoderATtiny85

A minimal DCC (Digital Command Control) accessory decoder for a bare
**ATtiny85** that drives a red/white bi-colour LED as directional head/tail
lights on a model locomotive (an HST Hornby dummy car).

Ported from the ESP8266 version (`DCCDecoderD1Mini`), stripped to fit the
ATtiny's 8 KB flash / 512 B RAM with no serial, WiFi or H-bridge.

## Behaviour

Watches a single loco address (default **10**) and drives the LED from its
F0/F1/direction state:

| Throttle state      | LED              |
|---------------------|------------------|
| F0 on, forward      | white            |
| F0 on, reverse      | red              |
| F0 (+ F1) on        | same colour, dimmed |
| F0 off              | off              |

- **Instant-on:** lights come up within milliseconds of power, defaulting to
  red/reverse, and the last F0/F1/direction is restored from EEPROM so a
  momentary track dropout barely blinks the lights.
- The watched address can be re-programmed at runtime by writing **CV1** on a
  programming track (adopted and saved to EEPROM).

## Hardware

Bi-colour LED driven **directly from two pins** (no H-bridge); current flows
out of one pin, through the LED + one series resistor (~150–330 Ω), into the
other, so one pin sources and the other sinks.

| ATtiny85 pin | Signal |
|---|---|
| 2 (PB3) | DCC signal in, from the optocoupler output (PCINT3) |
| 3 (PB4) | debug serial TX (only when `DBG 1`) |
| 4 (GND) | ground |
| 5 (PB0) | LED A |
| 6 (PB1) | LED B |
| 8 (VCC) | +5 V |

Feed the DCC signal through an optocoupler isolation front-end — **do not
connect the ATtiny directly to the track.**

## Build & flash

Built with [ATTinyCore](https://github.com/SpenceKonde/ATTinyCore), programmed
with an **Arduino Uno as ISP** (`arduinoasisp`, 8 MHz internal clock).

```sh
# one-time: flash ArduinoISP onto the Uno, then set fuses
arduino-cli burn-bootloader -b ATTinyCore:avr:attinyx5:chip=85,clock=8internal \
  -P arduinoasisp -p /dev/cu.usbmodemXXXX

# compile + upload the decoder
arduino-cli upload -b ATTinyCore:avr:attinyx5:chip=85,clock=8internal \
  -P arduinoasisp -p /dev/cu.usbmodemXXXX .
```

Uno → ATtiny ISP wiring: D13→pin7 (SCK), D12→pin6 (MISO), D11→pin5 (MOSI),
D10→pin1 (RESET), 5 V→pin8, GND→pin4.

### 8 MHz clock note

`setup()` forces the clock prescaler to ÷1 (`CLKPR = 0x80; CLKPR = 0x00;`) so
the chip runs at a true 8 MHz even if the CKDIV8 fuse is still set — the DCC
pin-change ISR must service ~58 µs half-bit edges, which it can't do at 1 MHz.

## Debugging

Set `#define DBG 1` to enable a hand-rolled TX-only serial on PB4 (pin 3) at
9600 8N1 (read with an FTDI: FTDI RX ← pin 3, shared GND). It prints a
once-a-second snapshot of edge count, decoded packets, watched-loco packets and
the current F0/direction. Leave `DBG 0` for production (the serial's blocking
transmit otherwise makes the F1-dimmed mode flicker).
