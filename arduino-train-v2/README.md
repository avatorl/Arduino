# Arduino DUPLO Train v2

A 3D-printed DUPLO-compatible train powered by an Arduino Nano, remote control, lights, sound, and optional sensors.

Original inspiration: <https://cults3d.com/en/3d-model/game/locomotora-sofia-controlada-por-infrarojos-con-control-de-velocidad-por-ultrasonidos-y-motor-superior-multiusos-lego-duplo>

Arduino code rewritten and extended by Andrzej Leszkiewicz.

3D-print profile: <https://makerworld.com/en/models/1854728-arduino-train-locomotive-remote-controlled#profileId-1983131>

## What this train can do

- Remote-controlled forward and backward driving
- 3 manual speed levels for normal driving
- Stop command with status lights
- Headlights / colored status lights
- Horn and siren sounds
- 8 melodies
- Mute / unmute
- Battery status announced by beeps
- Automatic sleep after inactivity, with wake-up from the IR remote
- Tilt protection: stops when the train is on its side
- Obstacle detection with automatic stop / restart when using the distance sensor
- Optional color-marker detection when using the color sensor

## What you need to buy

### Core electronics

- Arduino Nano 3.0 compatible board, ATmega328P
- DRV8833 motor driver module or board
- IR receiver module compatible with the HX1838 / TSOP4838 family
- 21-button IR remote, commonly sold as a "Car MP3" remote
- Passive buzzer
- 2 common-anode RGB LEDs
- 1 green LED
- DC geared motor, around 3 V to 6 V, 1:48 ratio
- 2 x 18650 Li-ion cells
- 2S battery protection board (BMS)
- 2S charging board / charger module
- Main power switch
- Fuse

### Sensors

- VL53L0X distance sensor module, for obstacle detection
- SW-520D tilt sensor, for tilt protection
- TCS34725 color sensor module, for color-marker actions

### LED control parts

- MCP23008 I2C GPIO expander
- 7 x NPN transistor switches or logic-level MOSFETs for LED channels
- LED resistors and small-signal resistors for the chosen LED driver stage

### Build materials

- 3D-printed train body and mechanical parts from the linked model/profile
- Wires, headers, small perfboard or PCB, connectors, and mounting hardware

## Which parts are optional

You can build a simpler version if you do not need every feature.

- Without the `VL53L0X`, the train still works, but there is no obstacle-aware auto driving.
- Without the `TCS34725`, the train still works, but there are no color-marker actions.
- Without the `SW-520D`, the train still works, but there is no tilt stop.
- Without the `MCP23008`, you need a different LED-driving approach.

## High-level wiring overview

- The motor is powered from the 2S battery pack through the `DRV8833`.
- The Arduino Nano is the main controller.
- The IR receiver handles the remote control.
- The buzzer provides horn, siren, and melodies.
- The LEDs provide headlights and status colors.
- The `VL53L0X`, `TCS34725`, and `MCP23008` share the I2C bus.

## Before you start

- Make sure you are comfortable working with Li-ion cells and a 2S battery pack.
- Confirm that your motor, motor driver, and battery wiring are correct before powering the train.
- If you are mixing parts from different sellers, double-check pinouts on every module.

## Project status

Current hardware and software support the features listed above.

Planned / future improvement:

- Motor overcurrent protection

## More details

- Public project note: [doc/README.md](doc/README.md)
- Internal notes: `docs-internal/`
