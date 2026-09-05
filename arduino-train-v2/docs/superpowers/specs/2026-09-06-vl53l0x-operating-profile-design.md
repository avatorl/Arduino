# VL53L0X Operating Profile Repair Design

## Goal

Make `arduino-train-v2` use the proven VL53L0X operating profile from
`SENSORS\time-of-flight`, eliminating repeat invalid status-4 / 8191 mm
samples without weakening the train's fail-safe distance behavior.

## Cause and chosen approach

The standalone sketch uses a 1.0 MCPS return-signal-rate limit and a 100 ms
continuous measurement period. The train's compact driver retains the
library's less selective 0.25 MCPS default and asks for 50 ms measurements.
The user has verified the hardware works with the standalone sketch.

The train will retain its compact register-level driver and adopt the two
proven settings. It will not add the Pololu library, which is unnecessary and
would consume scarce Nano flash and SRAM.

## Changes

- Define named VL53L0X constants for a 1.0 MCPS signal-rate limit and a 100 ms
  continuous-ranging period.
- Apply the signal-rate limit after sensor initialization and before continuous
  ranging. Initialization fails safely if that write fails.
- Use the 100 ms ranging period and ensure the polling cadence does not request
  readings more frequently than the completed-sample cadence.
- Preserve XSHUT address reassignment, the TCS34725 coexistence sequence,
  median filtering, the 50 cm auto-control ceiling, diagnostic status logging,
  the 250 ms last-good-sample grace interval, and the fail-safe stop/fault
  behavior.

## Validation

1. Compile the production Nano sketch and the project compile matrix.
2. Run the native train-logic suite.
3. On hardware, confirm normal targets produce status 0 readings in
   auto-distance mode using the 100 ms profile.
4. Confirm a rejected or absent reading still holds/stops the train according
   to the existing grace and fault policy.
