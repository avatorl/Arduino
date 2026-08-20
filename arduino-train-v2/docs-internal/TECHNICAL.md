# Arduino Train Technical Reference

This file holds the detailed project notes that do not belong in the public README.

## Sketch layout

- `arduino-train-v2.ino` - shared types, shared state, `setup()`, and `loop()`
- `10-ir-remote.ino` - IR codes, repeat handling, and command routing
- `20-motor.ino` - motor control, boost, reverse delay, and auto-distance speed
- `30-lights-and-sounds.ino` - LEDs, buzzer patterns, siren, and melodies
- `40-sensors.ino` - color, distance, and tilt sensor helpers
- `50-power-management.ino` - EEPROM logs, battery policy, and sleep/wake flow
- `config.h` - shared compile-time settings
- `melodies.h` - flash-resident melody tables

## Sleep, warning, and shutdown

- **Sleep** means the train is idle and saving power after inactivity.
- **Warning** means the battery is getting low, but the train can still run in a limited way.
- **Shutdown** means the battery is too low and the firmware locks out normal use until recharged.

The train uses timed signals and light patterns to show each state. The remote can wake it again, and the wake button press is reused as a normal command when possible.

## Electrical and power notes

- Add 100 nF from A0 to GND at the battery divider output.
- Place 100 uF bulk capacitance and 100 nF ceramic near the DRV8833 supply pins.
- Add motor-noise suppression capacitors if your motor does not already have them.
- The TSOP4838 IR receiver benefits from a local supply filter.
- Sleep current is dominated by board-level hardware, not only firmware.
- After the long low-battery shutdown beep, turn the train off and charge it.

## Build and verification

Compile the sketch with:

```powershell
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-train-v2 --warnings all
```

Run the compile matrix with:

```powershell
& "D:\GITHUB\Arduino\arduino-train-v2\docs-internal\compile-matrix.ps1"
```

Run the host-side logic tests with:

```powershell
& "D:\GITHUB\Arduino\arduino-train-v2\test\native\run-tests.ps1"
```

If `g++` is not on PATH, the test runner falls back to Python.

## Color marker calibration

The train's generated color-marker block comes from shared sensor capture data. After updating those captures, regenerate the block with:

```powershell
python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --target train
```

Replace only the generated-data region in `arduino-train-v2.ino`. The export maps supported marker labels to train actions.

## More details

- Internal beginner guide: `LEARN.md`
- Project status: the current hardware and software support the listed features, with motor overcurrent protection still planned as a future improvement.
