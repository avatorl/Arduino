# Arduino Train v2 Library Slimming Design

## Goal

Reduce `arduino-train-v2` flash usage by aggressively replacing heavy device
libraries with sketch-local implementations, while preserving current
functionality and leaving music content unchanged.

## Scope

- Make a backup copy of `arduino-train-v2.ino` before implementation.
- Keep `arduino-train-v2.ino` as the main train sketch.
- Preserve existing user-visible behavior:
  - IR remote control
  - motor control and safety behavior
  - LEDs and status lights
  - buzzer patterns and melodies
  - sleep / wake flow
  - tilt detection
  - obstacle detection
  - color-marker detection
- Do not change or remove melody content in this phase.
- Keep changes local only; do not create Git commits.

## Targeted Library Replacements

Replace usage of these higher-overhead device libraries with smaller sketch-local
helpers built directly on `Wire`:

- `Adafruit_TCS34725`
- `Adafruit_MCP23X08`
- `VL53L0X`

Do not replace these in this phase:

- `IRremote`
- `LowPower`
- melody data / melody player

## Replacement Architecture

### TCS34725

Replace the current library usage with a minimal helper that supports only:

- device detect / init
- integration-time and gain configuration matching the current 50 ms / 4x setup
- enable
- disable
- raw RGBC read

The higher-level train code should keep the same intent and color-classification
flow, but call the smaller helper instead of the full library.

### MCP23008

Replace the current expander library usage with a minimal helper that supports
only:

- device detect / init
- pin direction configuration
- digital output writes

This helper only needs to cover the LED-expander use case present in the train.

### VL53L0X

Replace the current library usage with a minimal helper that supports only:

- detect / init
- I2C address set
- timeout and timing-budget configuration matching the current train flow
- start continuous ranging
- read current range
- timeout or invalid-reading reporting

The implementation may be narrower than the full library as long as the train's
current obstacle-detection behavior remains functionally equivalent.

## Compatibility Rules

- Preserve current wiring; no hardware rewiring should be required.
- Keep the current train logic structure wherever possible and swap only the
  device-access backend.
- If one library replacement proves too risky during implementation, that
  device may temporarily keep its current backend while the other replacements
  proceed.
- Any fallback must be explicit and documented.

## Validation

- Compile `arduino-train-v2` for Arduino Nano before and after the change.
- Compare flash usage before and after the replacements.
- The change succeeds only if flash usage is measurably lower than the current
  baseline and the Nano build still passes.
- Verify that the build still succeeds on the Nano target.
- Update documentation only if setup or build expectations change.
