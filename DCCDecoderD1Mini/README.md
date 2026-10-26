# DCCDecoderD1Mini

A DCC (Digital Command Control) decoder for a **Wemos D1 mini (ESP8266)** that
drives a red/white bi-colour LED through a DRV8833 H-bridge as directional
head/tail lights on a model locomotive (an HST Hornby dummy car).

Ported from the ESP32-C3 version (https://github.com/brianjobling/dcc-decoder).
The ATtiny85 port lives in the sibling `DCCDecoderATtiny85` project.

## Behaviour

Watches a single loco address and drives the LED from its F0/F1/direction:

| Throttle state | LED |
|---|---|
| F0 on, forward | white |
| F0 on, reverse | red |
| F0 (+ F1) on   | same colour, dimmed |
| F0 off         | off |

- **Instant-on:** the headlight is driven first thing in `setup()` (before
  serial/WiFi), and the last F0/F1/direction is restored from EEPROM so a
  momentary track dropout barely blinks the lights.
- **Watched address is programmable** three ways: write it on a programming
  track (`<W n>` in DCC-EX), throw accessory **999** then toggle F0 on the
  wanted loco (learn mode), or type the loco number into the serial monitor.

## Hardware

Feed the DCC signal through the 6N137 optocoupler isolation front-end from
https://www.digitaltown.co.uk/79DCCDecoderCircuit.php (3.3 V supply).
**Do not connect direct to the track.**

| D1 mini pin | Signal |
|---|---|
| D5 (GPIO14) | DCC signal in from the optocoupler |
| D1 (GPIO5)  | DRV8833 IN1 |
| D2 (GPIO4)  | DRV8833 IN2 |
| D4 (GPIO2)  | onboard status LED (active low) |
| D3 (GPIO0)  | optional learn button to GND |

## Notes

- Set `DEBUG_DCC 0` to silence the serial diagnostics for a live install.
- Sketch runs at 115200 baud. The ESP8266 also dumps its boot-ROM log at 74880
  baud on reset.
- When flashing/talking over USB, disconnect the buck-converter 5 V feed first —
  with both power sources on, inrush sags the rail and the board won't enumerate.
