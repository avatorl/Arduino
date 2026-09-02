# Selectable ToF Backend Design

## Goal

Make the Arduino DUPLO Train v2 compile and run with a VL53L0X by default while
retaining the VL53L1X implementation as a compile-time-selectable alternative.
The VL53L0X uses the existing A3 XSHUT wiring and is moved to I2C address `0x2A`
before the TCS34725 color sensor can use the shared default address `0x29`.

## Approach

`config.h` will expose `USE_VL53L1X_DISTANCE_SENSOR`, a compile-time switch
whose only valid values are `0` and `1`. It defaults to `0` for VL53L0X; `1`
selects VL53L1X. Each backend tab owns its outer `#if`:
`42-distance-sensor-vl53l0x.ino` compiles under `#if
!USE_VL53L1X_DISTANCE_SENSOR` and `43-distance-sensor-vl53l1x.ino` compiles
under `#if USE_VL53L1X_DISTANCE_SENSOR`. Sensor-neutral address, XSHUT-pin,
timeout, sample-period, and diagnostic names will replace VL53L1X-specific
names where they are shared. VL53L1X-only ROI configuration remains compiled
only with that backend.

The implementation is split into three independent tabs:

- `42-distance-sensor-common.ino` owns filtering, fault handling, and the
  public distance-sensor lifecycle API.
- `42-distance-sensor-vl53l0x.ino` contains only the VL53L0X driver and its
  backend adapter functions, compiled when the default backend is selected.
- `43-distance-sensor-vl53l1x.ino` contains only the VL53L1X driver and its
  backend adapter functions, compiled only when the VL53L1X backend is
  selected.

The common tab talks to a small sensor-neutral adapter surface:
`initializeDistanceSensorBackend()`, `startDistanceSensorBackend()`,
`stopDistanceSensorBackend()`, `isDistanceSensorSampleReady()`,
`readDistanceSensorMillimeters()`, and
`isDistanceSensorReadingValid()`. Initialization returns `false` on any
address-assignment, boot, or configuration failure. The readiness call must
not block; read returns a millimeter sample only after readiness is true; the
validity call reports whether that sample is usable. Consequently, neither
sensor tab contains the other sensor's driver or branches through its
chip-specific protocol. Replacing the sensor only requires selecting the
matching whole-file backend; the inactive tab contributes no program code.

The public API remains `initDistanceSensorHardware()`,
`startDistanceSensorRanging()`, `setDistanceSensorRangingActive()`, and
`getDistanceReading()`. Motor, remote, sleep, and startup behavior therefore
continue to use the same non-blocking, median-filtered, fail-safe interface.

## Failure Handling

`initSensorHardware()` must initialize the selected ToF backend before the
TCS34725. Each backend holds XSHUT low, waits, releases it, waits for boot, and
write-reassigns the ToF sensor from `0x29` to `0x2A` before its first read or
configuration transaction. Only then may color-sensor initialization or access
occur. Sleep/wake calls the same initialization path; failure leaves the sensor
undetected, keeps ranging inactive, and the existing auto-distance fail-safe
stops the motor. Invalid measurements or stalled ranging likewise disable
auto-distance mode and stop the motor after the configured grace period.
Diagnostics identify the compiled sensor backend.

## Documentation and Validation

The sketch header and README will identify the default VL53L0X and document the
single configuration switch. Compile the Nano sketch with the default VL53L0X
branch and with the VL53L1X branch selected. On hardware, verify address
reassignment, valid measurements, auto-distance obstacle behavior, and
sleep/wake reinitialization for the installed VL53L0X.
