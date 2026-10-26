# dcc-accessory-decoder

DCC (Digital Command Control) decoders that light a red/white bi-colour LED as
directional head/tail lights for a model locomotive (an HST Hornby dummy car) —
white forward, red reverse, F0/F1 control, with instant-on EEPROM state restore.

Two implementations share the same behaviour:

| Project | Target | LED drive |
|---|---|---|
| [**DCCDecoderATtiny85**](DCCDecoderATtiny85/) | bare ATtiny85 (DIP-8, 8 MHz) | two GPIO pins direct |
| [**DCCDecoderD1Mini**](DCCDecoderD1Mini/) | Wemos D1 mini (ESP8266) | DRV8833 H-bridge |

## Behaviour

Watch a single DCC loco address and drive the LED from its F0/F1/direction:

| Throttle state | LED |
|---|---|
| F0 on, forward | white |
| F0 on, reverse | red |
| F0 (+ F1) on   | same colour, dimmed |
| F0 off         | off |

- **Instant-on:** the LED is driven first thing at power-up, restoring the last
  F0/F1/direction from EEPROM, so a momentary track dropout barely blinks it.
- The watched address is programmable over the rails (CV1 write on a programming
  track; the D1 mini also supports accessory-999 learn mode and serial entry).

Feed the DCC signal through an optocoupler isolation front-end — **never connect
direct to the track.**

See each subproject's README for wiring, build and flashing details.
