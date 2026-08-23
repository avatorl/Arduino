# Arduino Train v2 ToF Startup Diagnostics Design

## Goal

Make VL53L0X startup failures diagnosable without changing behavior when
`DEBUG_DISTANCE_SENSOR` is disabled.

## Scope

Add debug-only startup output in `arduino-train-v2/40-sensors.ino` that:

- probes I2C ACK status at `0x20`, `0x29`, and `0x2A`;
- reports the VL53L0X model-ID register value at the active address;
- identifies whether failure occurs before address reassignment, after
  reassignment, or during VL53L0X initialization.

## Design

Compile the diagnostic helpers and their calls only when
`DEBUG_DISTANCE_SENSOR == 1`. The helpers use `Wire` probes and a direct
read of VL53L0X register `0xC0`; they do not write registers, alter XSHUT
timing, or change the normal initialization order.

Diagnostics run at the existing sensor-startup checkpoints:

1. while VL53L0X XSHUT is low;
2. after XSHUT release, before address reassignment;
3. after reassignment to `0x2A`;
4. when initialization fails, including the model-ID result.

## Validation

- Build the sketch for Arduino Nano with `DEBUG_DISTANCE_SENSOR` enabled.
- Build it with the flag disabled and confirm the diagnostic helper/calls are
  compiled out.
- On hardware, verify output reports ACK state for all three addresses and
  reports model ID `0xEE` for a responding VL53L0X.
