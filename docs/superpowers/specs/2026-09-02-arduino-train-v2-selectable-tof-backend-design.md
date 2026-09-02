# Selectable ToF Backend Design

## Goal

Make the Arduino DUPLO Train v2 compile and run with a VL53L0X by default while
retaining the VL53L1X implementation as a compile-time-selectable alternative.
The VL53L0X uses the existing A3 XSHUT wiring and is moved to I2C address `0x2A`
before the TCS34725 color sensor can use the shared default address `0x29`.

## Approach

`config.h` will expose one mutually exclusive compile-time backend switch. Its
default selects the VL53L0X; the VL53L1X can be selected explicitly for
compatible hardware. Sensor-neutral address, XSHUT-pin, timeout, sample-period,
and diagnostic names will replace VL53L1X-specific names where they are shared.
VL53L1X-only ROI configuration remains compiled only with that backend.

`42-distance-sensor.ino` will contain preprocessor-gated driver implementations:

- The existing VL53L0X driver from `90-legacy-vl53l0x.ino` becomes the active
  implementation when the default backend is selected.
- The existing VL53L1X driver remains available only under the VL53L1X branch.
- Shared runtime filtering, initialization, continuous-ranging control, and
  public lifecycle functions stay outside the implementation choice.

The public API remains `initDistanceSensorHardware()`,
`startDistanceSensorRanging()`, `setDistanceSensorRangingActive()`, and
`getDistanceReading()`. Motor, remote, sleep, and startup behavior therefore
continue to use the same non-blocking, median-filtered, fail-safe interface.

## Failure Handling

Both backends must move off `0x29` before any read that could conflict with the
TCS34725. Initialization failure, invalid measurements, or stalled ranging
continues to disable auto-distance mode and stop the motor after the configured
grace period. Diagnostics identify the compiled sensor backend.

## Documentation and Validation

The sketch header and README will identify the default VL53L0X and document the
single configuration switch. Compile the Nano sketch with the default VL53L0X
branch and with the VL53L1X branch selected. On hardware, verify address
reassignment, valid measurements, auto-distance obstacle behavior, and
sleep/wake reinitialization for the installed VL53L0X.
