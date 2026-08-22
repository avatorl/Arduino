# Arduino Train Critical Overvoltage Safety Design

## Goal

Safely latch the Arduino Train v2 when the measured 2S pack voltage exceeds
`BATTERY_MAX_VALID_MV` (8.5 V). The latch must require a physical power cycle
to recover, keep the rear red indicator on, stop the motor, disable every
other controllable output, and provide an audible critical-error indication.

The change also broadens power-management serial diagnostics and makes the
main sketch the single index of all user-visible error indications.

## Critical Overvoltage State

Add a dedicated, highest-priority critical-overvoltage state. It is distinct
from the existing low-battery warning and permanent low-battery shutdown:

- It latches on the first valid ADC conversion above
  `BATTERY_MAX_VALID_MV`; a reading of zero is not an overvoltage condition.
- It is detected from every direct battery measurement, including stopped
  battery guarding and the 250 ms loaded-voltage refresh used for motor PWM.
- On entry, it immediately cancels auto drive, jog, boost, siren, melodies,
  buzzer patterns, sensor mode, pending reversals, and idle-sleep behavior;
  then coasts and disables the DRV8833 through `pinMotorSleep`.
- It turns off the front RGB headlights, green status LED, color-sensor LED,
  distance sensor, and color-sensor core. The rear red indicator is turned on
  after all other LED outputs have been cleared.
- It ignores normal IR input and prevents all later runtime services from
  overwriting the critical output state.
- It remains awake rather than entering `LowPower.powerDown()`, because the
  rear red indicator must stay visibly lit. Only removing and restoring power
  clears the state.
- It plays one distinctive, finite critical-error tone sequence when entering
  the state, then remains silent. This avoids a continuous buzzer drain while
  retaining an unmistakable audible alert.
- It appends one EEPROM overvoltage event when EEPROM logging is enabled,
  subject to the existing per-boot write budget. The stored voltage byte uses
  the existing invalid-reading sentinel because a value above 8.5 V cannot be
  represented as a valid 2S voltage.

The state is checked early in `loop()` and before normal motor, automatic
distance, idle, sensor, and IR behavior. It takes precedence over all other
faults and visual modes.

## Power-Management Debugging

Replace the public configuration flag and corresponding macros
`DEBUG_VOLTAGE_METER` / `DBG[_LN]_VOLTAGE_METER` with
`DEBUG_POWER_MANAGEMENT` / `DBG[_LN]_POWER_MANAGEMENT`.

The new category continues the current rate-limited ADC average and decoded
voltage output. It additionally reports, once per transition:

- a measured voltage crossing the critical-overvoltage limit and critical
  latch entry;
- warning entry, periodic warning signal, recovery after recharge margin, and
  low-battery shutdown entry;
- the initial idle-sleep warning, the message immediately before idle sleep,
  wake completion, and the message immediately before permanent shutdown;
- skipped or invalid direct readings where that affects a power-safety
  decision.

All messages are emitted before a blocking sleep or permanent shutdown call so
they can be observed in Serial Monitor. The debug flag remains compile-time
gated and is included in `DEBUG_ANY`, so it adds no Serial initialization or
runtime output when disabled.

## Main-Sketch Indicator Reference

Add a compact comment block in `arduino-train-v2.ino`, near the shared enums
and runtime-state documentation. It will be the source-of-truth index for
every visible/audible status or error event, listing:

| Event | Motor policy | Lights | Sound | Recovery |
| --- | --- | --- | --- | --- |
| Normal stopped, forward, reverse, and auto-distance states | Existing drive policy | Existing red, white, blue, and green indications | Existing command cues | Existing controls |
| Low-battery warning and final low-battery shutdown | Existing restriction / shutdown policy | Existing rear-red sequence and shutdown outputs | Existing battery alert | Existing recharge or power-cycle policy |
| Critical overvoltage | Disabled | Rear red steady; all other LEDs off | One critical alert sequence | Power cycle only |
| DRV8833 driver fault | Disabled | Existing red fault indication | Existing fault pattern | Fresh motor command after the driver clears |
| Distance-sensor, tilt, idle-sleep, and sensor-mode events | Existing module-specific policy | Existing documented colors/blinks | Existing patterns | Existing controls |

The table documents actual behavior only; it does not alter unrelated
indicators. Any existing event whose current sound/color behavior is not
explicitly described will be traced and included before implementation.

## Implementation Boundaries

- `config.h`: rename the debug setting and add only the constants or event
  identifiers required for the overvoltage fault.
- `arduino-train-v2.ino`: rename debug macros, add the indicator reference,
  shared critical-state data, EEPROM event identifier, and loop precedence.
- `50-power-management.ino`: centralize direct-reading validation,
  overvoltage transition handling, expanded power debug messages, and
  EEPROM-event labeling.
- `30-lights-and-sounds.ino`: add the finite critical-alert pattern and an
  output helper that establishes the critical red-only state without being
  overridden by normal indicator gates.
- `10-ir-remote.ino` and `20-motor.ino`: only add guards necessary to reject
  commands and motor-drive attempts while the critical latch is set.

No unrelated refactoring or changes to existing low-battery thresholds are in
scope.

## Validation

1. Compile the sketch with `DEBUG_POWER_MANAGEMENT` both enabled and disabled;
   verify no obsolete `DEBUG_VOLTAGE_METER` references remain.
2. Unit-test pure voltage/state helpers where the project test setup permits:
   zero, 8.5 V exactly, 8.501 V, and ADC-full-scale readings.
3. On hardware or a controlled ADC test harness, induce an over-8.5 V
   reading and verify: motor outputs are zero, driver sleep is low, only rear
   red is on, one critical sound occurs, IR commands cannot change state, and
   only a power cycle restores operation.
4. Exercise warning, warning recovery, idle sleep/wake, and low-battery
   shutdown to verify their existing behavior is unchanged and each
   transition produces the expected power-debug line before sleep/shutdown.
5. Inspect the EEPROM debug summary after the fault to confirm exactly one
   overvoltage event was logged within the write budget.
