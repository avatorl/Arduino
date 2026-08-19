# Time of Flight Test

Minimal standalone Arduino sketch for testing a `VL53L0X` distance sensor before integrating it into the train project.

## What it does

- Initializes the sensor at its default I2C address `0x29` via `Adafruit_VL53L0X::begin()`
- Reads distance in single-shot mode
- Prints distance in millimeters and centimeters, or "Out of range" for phase failures / implausible readings

## Wiring

Use this wiring for an Arduino Nano / ATmega328P board. `XSHUT` is tied directly to the sensor's
power rail (always on); this sketch does not control it from a digital pin.

| Arduino | VL53L0X |
| --- | --- |
| `5V` | `VIN` |
| `GND` | `GND` |
| `A4` | `SDA` |
| `A5` | `SCL` |
| `5V` | `XSHUT` |

## Serial Monitor

- Baud rate: `9600`
- Example output:

```text
VL53L0X test
Reading single-shot distance in mm and cm.
Distance: 183 mm (18.3 cm)
Distance: 181 mm (18.1 cm)
Out of range
```

`RangeStatus == 4` from the Adafruit library means a phase failure (no target detected, also
shown with a large placeholder distance around `8190 mm`); anything else is treated as a valid
reading.

## Compile

From the workspace root:

```powershell
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 --libraries "e:\ARDUINO\Arduino-Code-GitHub\libraries" "e:\ARDUINO\Arduino-Code-GitHub\time-of-flight"
```