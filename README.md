# dcc-accessory-decoder
ESP8266 and ATtiny85 DCC decoders that light a red/white bi-colour LED as directional head/tail lights for a model locomotive — white forward, red reverse, F0/F1 control, with instant-on EEPROM state restore.

## Sketches

- `esp8266_dcc_lights/` – Wemos D1 mini + DRV8833 H-bridge.
- `attiny85_dcc_lights/` – bare ATtiny85 (8 MHz internal), LED driven directly from PB0/PB1 via one series resistor (150–330 Ω). Program with Arduino-as-ISP using ATTinyCore (8 MHz internal). The sketch forces the clock prescaler to ÷1 and uses a pin-change interrupt plus software PWM so Timer0/`micros()` stay intact.

Both watch one loco address (default 10; changed by a CV1 write on a programming track) and decode F0/F1/direction: F0 + forward = white, F0 + reverse = red, F1 dims, F0 off = off. State is restored from EEPROM at power-up (default red/reverse). Always feed the DCC input through an optocoupler. Set `DEBUG_SERIAL 1` for TX-only debug (edges/s, packets, loco activity).
