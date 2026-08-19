# Arduino Train Debug Gating Update Design

## Goal

Change `arduino-train-v2\arduino-train-v2.ino` so `DEBUG` acts as a master gate:

- `DEBUG = 0` disables every debug feature, even if individual `DEBUG_*` flags are `1`
- `DEBUG = 1` allows debug output, but each category must still be enabled individually

This keeps debug behavior explicit and prevents accidental logging when the global debug switch is off.

## Debug Model

Each module-specific flag will default to `0` unless the sketch explicitly opts in. The compile-time macros will use `DEBUG && DEBUG_<CATEGORY>` for every debug category, including:

- `DEBUG_COLOR_SENSOR`
- `DEBUG_DISTANCE_SENSOR`
- `DEBUG_IR_REMOTE`
- `DEBUG_TILT_SENSOR`
- `DEBUG_VOLTAGE_METER`
- `DEBUG_MOTOR`
- `DEBUG_LEDS`
- `DEBUG_SOUND`
- `DEBUG_REMOTE`

`DEBUG_ANY` will also be updated so serial initialization only happens when `DEBUG` is enabled and at least one debug category is enabled.

## Expected Behavior

- With `DEBUG=0`, no debug macro expands to active `Serial.print` / `Serial.println` output.
- With `DEBUG=1` and a category flag set to `1`, that category prints normally.
- With `DEBUG=1` and a category flag set to `0`, that category stays silent.
- Existing non-debug runtime behavior stays unchanged.

## Validation

Verify the new gating by compiling the sketch with representative configurations:

1. `DEBUG=0` with all category flags enabled: no debug output should compile in.
2. `DEBUG=1` with one or more category flags enabled: only those enabled categories should compile in.
3. `DEBUG=1` with all category flags disabled: sketch should still compile, but debug macros should stay silent.

Also confirm the sketch still builds for the supported Arduino targets after the macro changes.
