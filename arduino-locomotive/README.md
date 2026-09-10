# Arduino DUPLO Locomotive

A 3D-printed DUPLO-compatible locomotive powered by an Arduino Nano. Drive it
with an IR remote, or use automatic obstacle-aware speed control.

## Inside the locomotive

The electronics are packaged inside a 3D-printed DUPLO-compatible locomotive body. The Arduino Nano, motor driver, battery power hardware, sensors, LED wiring, and I2C expander are assembled on compact boards within the chassis.

![Locomotive electronics, view 1](docs/Inside%20of%20the%20Arduino%20locomotive%20_%20r_arduino_files/inside-of-the-arduino-locomotive-v0-ust0wejp8koh1.jpg)
*Motor and TCS34725 sensor on the bottom.*
![Locomotive electronics, view 2](docs/Inside%20of%20the%20Arduino%20locomotive%20_%20r_arduino_files/inside-of-the-arduino-locomotive-v0-avz914ohbkoh1.jpg)
*Battery pack: 2S 18650 cells with BMS, Arduino Nano, and MCP23008/MOSFET LED-control module.*
![Locomotive electronics, view 3](docs/Inside%20of%20the%20Arduino%20locomotive%20_%20r_arduino_files/inside-of-the-arduino-locomotive-v0-lus9s7xhdkoh1.jpg)
*TSOP4838 IR receiver.*
![Locomotive electronics, view 4](docs/Inside%20of%20the%20Arduino%20locomotive%20_%20r_arduino_files/inside-of-the-arduino-locomotive-v0-6howe1a39koh1.jpg)
*DRV8833 motor driver.*
![Locomotive electronics, view 5](docs/Inside%20of%20the%20Arduino%20locomotive%20_%20r_arduino_files/inside-of-the-arduino-locomotive-v0-bfgfo89t8koh1.jpg)
*LS-LISC-V3 USB charger for the 2S 18650 battery pack.*
![Locomotive electronics, view 6](docs/Inside%20of%20the%20Arduino%20locomotive%20_%20r_arduino_files/inside-of-the-arduino-locomotive-v0-h195qvd9akoh1.jpg)
*IR receiver, battery-level indicator, buzzer, and on/off button.*
![Locomotive electronics, view 7](docs/Inside%20of%20the%20Arduino%20locomotive%20_%20r_arduino_files/inside-of-the-arduino-locomotive-v0-hk36mkrfckoh1.jpg)
*Arduino Nano on perfboard with the tilt sensor and battery-voltage divider.*
![Locomotive electronics, view 8](docs/Inside%20of%20the%20Arduino%20locomotive%20_%20r_arduino_files/inside-of-the-arduino-locomotive-v0-nld4eoi69koh1.jpg)
*Component hidden beneath the battery pack.*
![Locomotive electronics, view 9](docs/Inside%20of%20the%20Arduino%20locomotive%20_%20r_arduino_files/inside-of-the-arduino-locomotive-v0-xtm4e78tbkoh1.jpg)
*VL53L0X laser distance sensor.*
![Locomotive electronics, view 10](docs/Inside%20of%20the%20Arduino%20locomotive%20_%20r_arduino_files/inside-of-the-arduino-locomotive-v0-xz6ehgmofkoh1.png)
*Inside of the Arduino locomotive.*

## What it does

- Drives forward and backward at three regular speed levels plus a timed boost.
- Supports manual driving and automatic obstacle-aware driving.
- Shows its status with headlights and rear/status LEDs.
- Plays a horn, siren, melodies, and battery alerts.
- Stops when tipped over and reacts to coloured track markers.
- Monitors the 2S battery pack and sleeps after inactivity until a remote command wakes it.

## What you need

- Arduino Nano-compatible board
- TSOP4838 IR receiver
- DRV8833 motor driver
- DC geared motor, about 3 V to 6 V
- 21-button IR remote
- Passive buzzer
- 2 common-anode RGB LEDs
- 1 green LED
- 2 red LEDs
- 2 x 18650 Li-ion cells
- 2S battery protection module
- 2S USB charger module
- Main power switch
- Fuse
- VL53L0X distance sensor
- SW-520D tilt sensor
- TCS34725 color sensor
- MCP23008 I2C GPIO expander
- LED resistors
- MOSFETs or transistors for custom LED wiring, if needed
- 3D-printed train body and mechanical parts
- Wires, headers, perfboard or PCB, connectors, and mounting hardware

## Safety

This project uses a 2S Li-ion battery pack. Use a protected pack, correct
polarity, a fuse, and suitable wiring. Incorrect battery wiring can damage the
electronics or create a fire risk.
