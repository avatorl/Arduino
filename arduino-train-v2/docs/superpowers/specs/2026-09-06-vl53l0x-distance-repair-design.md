# VL53L0X Distance Repair Design

## Goal

Restore correct full-range VL53L0X readings in `arduino-train-v2`, provide
actionable sensor diagnostics, and retire all VL53L1X support from the active
firmware.

## Current problem

The VL53L0X driver converts every valid range result to centimetres and then
clamps it to `AUTO_DISTANCE_MAX_SPEED` (50 cm). Consequently, debugging can
never show a value greater than 50 cm even though the sensor measured it.
Also, continuous ranging only runs in auto-distance mode, so enabling
`DEBUG_DISTANCE_SENSOR` alone does not produce live samples.

## Design

### Active VL53L0X behavior

- Keep the compact, hand-written register-level VL53L0X driver. Do not add the
  Pololu or ST library because the ATmega328P firmware has a tight flash and
  SRAM budget.
- Preserve the existing XSHUT address handoff from the sensor's default `0x29`
  to `0x2A`, which prevents conflict with the TCS34725.
- Preserve the non-blocking measurement lifecycle and the current fail-safe
  fault behavior.
- Retain the median filter and 50 cm ceiling only for the value passed into
  auto-distance motor control. Do not apply that ceiling to raw measurement
  diagnostics.
- When `DEBUG_DISTANCE_SENSOR` is enabled and the TCS34725 illumination output
  on A2 is enabled, start VL53L0X continuous ranging and print live diagnostic
  samples. Stop debug-only ranging when that illumination output is disabled.
- With distance debugging disabled, retain the normal production behavior:
  ranging is active only during auto-distance mode.

### Diagnostics

Distance-sensor debug output must identify:

- XSHUT and I2C-address checkpoints, including ACK/NACK results.
- Model-ID read result and initialization/configuration stage failures.
- Ranging lifecycle transitions, including whether activation was requested by
  auto-distance control or A2-enabled diagnostics.
- Every ready sample's raw millimetres, unbounded centimetres, range status,
  interrupt status, validity, median-filter result, and
  motor-control-clamped centimetres.
- No-ready and invalid-range conditions, including the measured status and the
  grace/fault outcome.

Diagnostic-only state and strings must remain behind
`DEBUG_DISTANCE_SENSOR` so normal builds do not pay for them.

### VL53L1X retirement

- Remove the VL53L1X backend compile-time switch, validation, ROI settings,
  conditionals, and all active-source references.
- Move `42-distance-sensor-vl53l1x.ino` to
  `arduino-train-v2\_\42-distance-sensor-vl53l1x.ino`; Arduino builds do not
  compile sketch files in that subdirectory.
- Update active documentation to identify VL53L0X as the only supported
  distance-sensor backend and remove stale VL53L1X/legacy-tab descriptions.

## Validation

1. Compile the selected Arduino Nano sketch with repository-local libraries.
2. Run the documented compile-time configuration matrix, including
   `DEBUG_DISTANCE_SENSOR`.
3. Run the native train-logic test suite.
4. On hardware, confirm readings above 50 cm are printed as their full measured
   distance, while auto-distance control still treats 50 cm and greater as
   maximum-speed distance.
5. On hardware, enable A2 illumination with distance debugging enabled and
   confirm live diagnostics appear without enabling auto-distance mode.
