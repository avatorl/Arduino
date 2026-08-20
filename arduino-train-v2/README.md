# Arduino DUPLO Train v2

A 3D-printed DUPLO-compatible train powered by an Arduino Nano and controlled by an IR remote.

This project was rewritten from scratch and extends the original idea with more train behavior, more safety checks, and more feedback for the driver.

## What the train can do

- Drive forward and backward from the IR remote
- Use 3 normal speed levels plus boost
- Show headlights, backlights, and color status lights
- Play horn, siren, and 8 melodies
- Report battery level with beeps
- Sleep after inactivity and wake from the remote
- Stop when tilted onto its side
- Stop or slow down when an obstacle is detected
- React to colored track markers

## Hardware used

### Core electronics

- Arduino Nano 3.0 compatible board
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
- Buck converter to 5 V
- Main power switch
- Fuse

### Sensors

- VL53L0X distance sensor
- SW-520D tilt sensor
- TCS34725 color sensor

### LED control parts

- MCP23008 I2C GPIO expander
- LED resistors
- MOSFETs or transistors for custom LED wiring, if needed

### Build materials

- 3D-printed train body and mechanical parts
- Wires, headers, perfboard or PCB, connectors, and mounting hardware

## Optional parts

You can build a simpler version if you do not need every feature.

- No `VL53L0X` = no obstacle-aware auto driving
- No `TCS34725` = no color-marker actions
- No `SW-520D` = no tilt stop
- No `MCP23008` = a different LED-driving approach is needed

## Safety

This project uses a 2S Li-ion battery pack, so take battery wiring and protection seriously. Wrong wiring can damage the hardware or create a fire risk.

## Learn more

- Beginner guide: `LEARN.md`
