# Arduino Multimeter Design

## Goal

Create a new standalone `arduino-multimeter` sketch that combines the existing
voltage-meter and capacitance-meter behavior into one serial-controlled tool
for Arduino Nano / Uno boards.

The first version should support:

- voltage measurement with automatic reference switching
- capacitance measurement with automatic range selection
- simple serial commands to request one reading at a time

## Scope

This project starts from the logic in:

- `SENSORS/voltage-meter-high-precision/voltage-meter-high-precision.ino`
- `SENSORS/test-capacitor/test-capacitor.ino`

The new sketch should live in a new `arduino-multimeter` folder and should not
modify the legacy sketches in `SENSORS/`.

## Architecture

Create multiple helper functions within multiple `.ino` files (one for each measurement mode: voltage, capacitance, and main file for shared logic)

Suggested helpers:

- `handleSerialCommand()` for parsing `v`, `c`, and `h`
- `measureVoltage()` for divider-based voltage reads
- `measureCapacitance()` for RC timing reads
- `dischargeCapacitor()` for the shared discharge routine

This keeps the control flow simple and makes it easy to verify each mode
independently.

## Hardware constants

Lock the first version to the same wiring used by the source sketches:

| Signal | Arduino pin | Notes |
|--------|-------------|-------|
| Voltage input | `A0` | Divider input from the existing voltage sketch |
| Capacitance input | `A0` | Shared analog sense point from the existing capacitor sketch |
| 10k charge path | `D13` | Small-capacitance charging resistor |
| 1k charge path | `D8` | Large-capacitance charging resistor |
| Discharge path | `D11` | Active-low discharge control |

Keep the existing divider and calibration constants from the voltage sketch and
the existing resistor values from the capacitance sketch unless later tuning
changes are explicitly requested.

## Voltage measurement design

The voltage mode should begin with the Arduino's 5V analog reference. That
first pass is used as a quick check to see whether the signal is low enough
that the 1.1V internal reference will give better resolution.

If the 5V-based estimate falls below the selected threshold, the sketch should
switch to the 1.1V reference and immediately re-read the input for the final
result. After every `analogReference()` change, discard the first ADC sample and
wait a short settling delay before using the second sample as the real value.
The threshold should be a single constant near the top of the sketch so it can
be tuned later without touching the measurement logic.

Voltage mode should leave the ADC reference in `DEFAULT` when it finishes so
capacitance mode always starts from a known reference state.

The voltage math should reuse the existing divider calculation from the current
high-precision sketch, including the calibration constant.

## Capacitance measurement design

Capacitance mode should reuse the existing dual-resistor timeout strategy:

- try the 10k charging path first for smaller capacitors
- if the charge time exceeds the timeout, discharge fully and retry with the
  1k charging path

This gives better precision for smaller capacitors while still supporting
larger ones. The output should report the measured value in nF or µF depending
on the final value.

Capacitance mode should always use the `DEFAULT` ADC reference and should set
it explicitly before starting the measurement so the shared analog input stays
consistent across commands.

## Serial interface

Supported commands:

- `v` — print one voltage reading
- `c` — print one capacitance reading
- `h` — print help text and command usage

The sketch should print one clear human-readable result per command and should
also print a short status message when it changes reference or range.

## Error handling

The sketch should avoid silent failures.

Expected failure cases to handle explicitly:

- capacitance timeout that still does not settle after the range switch
- incomplete discharge before a new capacitance measurement
- out-of-range or unstable voltage readings

Use these concrete rules:

- Treat voltage as out of range when the selected-reference ADC reading is
  within 5 counts of either rail after settling.
- Treat voltage as unstable when the 10-sample standard deviation exceeds
  0.05 V on the selected reference.
- Treat capacitance as failed if the 10k path exceeds a 100 ms timeout and the
  1k retry exceeds a 1 s timeout.
- Treat discharge as incomplete if the analog input remains above 5 counts for
  more than 100 ms while the discharge pin is held active low.

When a measurement cannot be trusted, print a clear text message instead of a
numeric result. Use stable command output such as `VOLTAGE: ...`,
`CAPACITANCE: ...`, or `ERROR: ...` so tests can match against it.

## Testing

Validate the sketch by compiling for Arduino Nano / Uno and then checking the
serial output for:

- a low-voltage input that forces the 5V → 1.1V switch
- a capacitor small enough to stay on the 10k path
- a capacitor large enough to force the 10k → 1k fallback

## Out of scope for the first version

- automatic continuous measurement loops
- display hardware
- logging to SD card or EEPROM
- calibration menus
- multiple sketch tabs
