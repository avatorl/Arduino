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

The implementation is split into three independent tabs:

- `42-distance-sensor-common.ino` owns filtering, fault handling, and the
  public distance-sensor lifecycle API.
- `42-distance-sensor-vl53l0x.ino` contains only the VL53L0X driver and its
  backend adapter functions, compiled when the default backend is selected.
- `43-distance-sensor-vl53l1x.ino` contains only the VL53L1X driver and its
  backend adapter functions, compiled only when the VL53L1X backend is
  selected.

The common tab talks to a small sensor-neutral adapter surface. Consequently,
neither sensor tab contains the other sensor's driver or branches through its
chip-specific protocol. Replacing the sensor only requires selecting the
matching whole-file backend; the inactive tab contributes no program code.

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
