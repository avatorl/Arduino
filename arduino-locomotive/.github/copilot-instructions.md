# Arduino Train v2 Instructions

## Build, test, and lint

Build the complete multi-tab sketch for its production board, using the
repository-local libraries:

```powershell
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-train-v2 --warnings all
```

Exercise the compile-time configuration matrix (baseline, EEPROM disabled, and
each debug flag) with:

```powershell
& .\docs-internal\compile-matrix.ps1
```

Run the native logic suite with:

```powershell
& .\test\native\run-tests.ps1
```

It compiles `test\native\train_logic_test.cpp` with `g++` when available,
otherwise it runs the equivalent Python harness. To run one individual Python
logic case, import the fallback harness and invoke its helper with the intended
inputs; for example, the obstacle-distance midpoint case:

```powershell
@'
import sys
sys.path.insert(0, r"test\native")
import train_logic_test as test
test.expect_equal(
    "distance midpoint",
    test.motor_voltage_from_distance(
        30, test.AUTO_DISTANCE_STOP, test.AUTO_DISTANCE_RESTART,
        test.AUTO_DISTANCE_MAX_SPEED, test.VOLTAGE_STEPS[1],
        min(test.NORMAL_MAX_MOTOR_MV, test.VOLTAGE_STEPS[test.NORMAL_MAX_SPEED_STEP]),
    ),
    4718,
)
'@ | python -
```

No lint command is configured.

## Architecture

The Arduino IDE combines the `.ino` files in this directory into one sketch.
`arduino-locomotive.ino` declares the shared types, state, helper interfaces,
`setup()`, and the cooperative `loop()` scheduler. Numeric tab prefixes encode
subsystems: IR commands (`10-`), motor and obstacle driving (`20-`), lights and
sound (`30-`), TCS34725 color markers (`41-`), the selected time-of-flight
driver (`42-`), MPU6050 safety (`43-`), and battery, EEPROM, and sleep behavior
(`50-`).

`config.h` is the central compile-time hardware and policy definition. It
chooses exactly one distance backend with `USE_VL53L1X_DISTANCE_SENSOR`; the
VL53L0X and VL53L1X tabs compile conditionally. Both the selected distance
sensor and TCS34725 initially use I2C address `0x29`, so startup holds the
distance sensor on XSHUT, moves it to `0x2A`, then initializes the color sensor.

The main loop is deliberately non-blocking and services safety before user
commands: VCC/battery guards, motor faults, idle sleep, jog timeout, distance
control, tilt/accelerometer checks, IR input, color markers, then feedback
engines. Safety conditions latch safe motor/output states; preserve their
ordering and make new periodic behavior state- and `millis()`-driven rather
than inserting loop delays.

## Firmware conventions

- Include `config.h` before `IRremote.hpp`. It fixes IRremote to Timer1 so
  Timer2 remains available to `tone()`; do not assign PWM work to D9 or D10.
- Keep the hand-written, register-level TCS34725 and VL53L0X/VL53L1X drivers.
  They intentionally avoid full sensor libraries to fit the ATmega328P's
  32 KB flash and 2 KB SRAM. Do not reorder, simplify, or replace documented
  sensor initialization register sequences without hardware verification.
- Retain `PROGMEM`, `F()` strings, fixed-width integers, and integer arithmetic
  in memory-sensitive code. Read program-memory data with `pgm_read_*`; avoid
  adding large globals, stack buffers, or `String` allocations.
- Feature switches and debug flags use `#ifndef` defaults so compile commands
  can override them with `-DNAME=value`. Keep configuration safety
  `static_assert`s with the related thresholds.
- Use rollover-safe elapsed-time comparisons such as
  `millis() - startedAt >= intervalMs`. Interrupt handlers only set `volatile`
  state; they must not perform serial, I2C, delays, or other lengthy work.
- EEPROM boot and fault records are bounded ring buffers. Preserve write caps
  and use `EEPROM.update()` semantics; never add per-loop EEPROM writes.
- Keep motor, tilt, sensor, and power fault paths fail-safe. A drive command
  must not bypass a latched fault, battery restriction, or required physical
  recovery check.

## Supporting references

Read `README.md` for the current hardware and user-visible behavior,
`docs-internal\TECHNICAL.md` for subsystem details and calibrated build
commands, and `docs-internal\LEARN.md` for the scheduler, timer, I2C, EEPROM,
and AVR-memory rationale. Use `test-i2c\test-i2c.ino` for hardware I2C
diagnosis rather than modifying the production sketch to probe the bus.
