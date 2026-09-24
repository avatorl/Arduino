# Resistance Command Design

## Goal

Add an `r` serial command to `arduino-multimeter.ino` that measures an
unpowered unknown resistor connected from A2 to GND. The reference-resistor
bank connects from GPIO pins to A2:

| Pin | Reference resistor | Resistance-mode use |
| --- | ---: | --- |
| D2 | 100 ohm | Never used; remains high impedance |
| D3 | 1 kohm | Auto-ranging reference |
| D4 | 10 kohm | Auto-ranging reference |
| D5 | 100 kohm | Auto-ranging reference |
| D6 | 1 Mohm | Auto-ranging reference |

Existing voltage, capacitance, zero-calibration, oscillograph, and help
commands must retain their current behavior.

## Measurement

The command will use D3 through D6 only. Before and after every sample, D2
through D6 will be configured as `INPUT` so all reference paths are
high-impedance. For each candidate range, the firmware will:

1. Configure its GPIO as `OUTPUT` and drive it HIGH.
2. Wait for the existing ADC settling interval.
3. Take the existing multi-sample ADC reading from A2.
4. Restore the GPIO to `INPUT`.

The sample nearest ADC mid-scale is selected because it provides the best
relative divider resolution. With `N` as its averaged ADC value and `Rref` as
the selected reference resistance, the calculated unknown resistance is:

```text
Rx = Rref * (1023 / N - 1)
```

The result uses ohms, kilohms, or megohms and identifies the chosen reference
range. Reference-resistor constants should be easy to replace with measured
values; nominal resistor values do not establish measurement accuracy.

## Safety and errors

The 100-ohm D2 range is intentionally excluded. A probe short would draw
approximately 50 mA from a 5 V Arduino GPIO through 100 ohms, above a typical
GPIO absolute maximum. D2 remains high impedance throughout resistance mode.

Before enabling a reference GPIO, the command will take a passive reading of
A2. A nonzero reading indicates a live or charged input and stops the
measurement. Firmware is not a substitute for hardware protection: the user
must measure only unpowered resistors.

The command reports explicit errors rather than a numeric result when:

- A2 indicates a live or charged input before excitation.
- All safe ranges are near zero, indicating a short or a resistance below the
  supported range.
- All safe ranges are near full scale, indicating an open circuit or a
  resistance above the supported range.

## Validation

Compile the sketch using the available Arduino board target and inspect the
targeted diff to confirm only the new `r` command and directly related
documentation change behavior.
