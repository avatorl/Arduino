# Arduino Train Debug Split Design

## Goal

Replace the catch-all `DEBUG_OTHER_SENSORS` switch in `arduino-train-v2.ino` with independent controls for the IR remote, tilt sensor, and voltage meter. Improve IR diagnostics so every non-repeat NEC command reports its decimal code and remote label, while command handlers report their actual assigned action.

## Debug Configuration

Add `DEBUG_IR_REMOTE`, `DEBUG_TILT_SENSOR`, and `DEBUG_VOLTAGE_METER` flags, each with matching `DBG_*` and `DBGLN_*` macros. When `DEBUG` is `1`, it overrides all individual module flags and enables every debug category. When `DEBUG` is `0`, each module flag independently controls its category.

Remove `DEBUG_OTHER_SENSORS` from `DEBUG_ANY` and replace it with the three new flags. Preserve existing flags and defaults for unrelated modules.

## IR Remote Diagnostics

On each newly decoded NEC command, `irReceive()` will print the decimal command code and a label derived from the documented Car MP3 remote button layout. The lookup covers all documented codes, including the currently unassigned `-` and `+` keys. Unknown command codes print an explicit `unknown` label.

The existing repeat behavior remains unchanged: repeat frames reuse the previous command for jog handling but do not emit duplicate press diagnostics.

`translateIR()` will print an action description through the IR remote debug category from the `switch` branch that processes the command. This includes successful actions and conditional rejections, so the serial monitor reflects what the train actually did. Melody button action descriptions will be printed by the existing melody dispatch path.

## Sensor Diagnostic Routing

Move tilt state-change messages to `DEBUG_TILT_SENSOR`.

Move startup battery measurement, periodic ADC readings, low-battery guard messages, and the battery-status command output to `DEBUG_VOLTAGE_METER`.

Move idle and sleep state messages to `DEBUG_IR_REMOTE`.

No train-control behavior, timing, safety interlocks, or existing serial output for unrelated debug modules changes.

## Validation

Verify the configuration macros compile correctly with each individual category enabled or disabled and with the `DEBUG=1` override. Inspect the sketch to confirm:

- every documented remote key logs its decimal code and label on a non-repeat press;
- each handled remote command logs a branch-specific action description;
- unassigned keys still log code and label without an action;
- repeat/jog behavior stays unchanged;
- tilt and voltage messages use only their new categories.
