
# smartrobot
# RPLidarA1Giga

Minimal Arduino library for running an RPLidar A1/A1M8 from an Arduino GIGA R1 WiFi.

What it does:
- basic serial protocol for `GET_DEVICE_INFO`, `GET_DEVICE_HEALTH`, `SCAN`, `STOP`, `RESET`
- standard scan mode decoding
- direct `MOTO_CTRL` PWM output from a GPIO pin
- example sketch with a human-readable, rotatable sector view

What it deliberately does **not** do yet:
- full Slamtec desktop SDK port
- express/HQ scan decoding
- async background threads
- every possible scan mode / config command

The example is meant as a bring-up and validation tool first.
