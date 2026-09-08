# Idle-Sleep AUTO Restart Design

**Date:** 2026-09-08  
**Project:** `arduino-locomotive`

## Problem

AUTO works after a fresh boot and after a normal obstacle stop, but cannot
restart after the five-minute inactivity sleep. The remote acknowledges
Play/Pause, but AUTO safely leaves the motor stopped.

Idle sleep drives the VL53L0X XSHUT pin low. This hardware reset restores the
sensor's I2C address from the runtime address `0x2A` to its power-on default
`0x29`. The firmware's `distanceTof` object still holds `0x2A`, so the current
wake code tries to configure a device that is no longer present at that
address. The failed measurement is correctly treated as invalid and AUTO
stops.

## Chosen Approach

Recover the sensor exactly as at initial startup:

1. After releasing XSHUT during idle wake, reset the driver's target address
   to the sensor's physical default `0x29`.
2. Reuse the established ranging initializer to move the device to
   `distanceSensorAddress` (`0x2A`), apply its configured operating profile,
   reset its filter, and start continuous ranging when AUTO remains enabled.

The sleep power-saving behavior remains unchanged. AUTO will still wait for a
fresh valid measurement and will keep the motor stopped when recovery fails.

## Code Changes

### Distance-sensor tab

Add a narrow recovery helper in `42-distance-sensor-vl53l0x.ino`. It is called
only after XSHUT has been held low and then released. The helper updates the
driver object's address back to `0x29` before calling the existing
`startDistanceSensorRanging()` path.

The existing initializer continues to own all profile configuration,
filter-state clearing, and continuous-ranging activation. No duplicate sensor
register setup is introduced.

### Idle-sleep wake flow

In `50-power-management.ino`, replace the direct post-wake call to
`startDistanceSensorRanging()` with the recovery helper. Keep the current
10 ms boot delay, motor-driver wake sequencing, accelerometer wake, ADC
restoration, and captured-IR-command flow unchanged.

## Error Handling and Safety

- A failed recovery leaves `distanceTofDetected` false, as it does today.
- AUTO's existing invalid-distance path sets its stop reason and leaves the
  motor off.
- Later explicit AUTO activation can continue using the existing fault-retry
  behavior.
- The repair does not bypass battery, tilt, motor-fault, or reversal guards.

## Validation

Extend the native source-contract tests to verify:

1. A dedicated post-XSHUT recovery helper sets the driver address to `0x29`
   before calling `startDistanceSensorRanging()`.
2. `goToIdle()` invokes that helper after it raises
   `pinDistanceSensorXSHUT`.
3. The normal AUTO handoff contract remains intact.

Run the native test suite and compile the production Nano sketch with the
project's documented local-library command.
