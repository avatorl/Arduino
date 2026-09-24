# Resistance Command Design

## Goal

Add an `r` serial command to `arduino-multimeter.ino` that measures an
unpowered unknown resistor connected from A2 to GND. The reference-resistor
bank connects from GPIO pins to A2:

| Pin | Reference resistor | Resistance-mode use |
| --- | ---: | --- |
| D2 | 100 ohm | Never used; remains high impedance |
| D3 | 1.0 kohm | Auto-ranging reference |
| D4 | 9.4 kohm | Auto-ranging reference |
| D5 | 71.7 kohm | Auto-ranging reference |
| D6 | 0.48 Mohm | Auto-ranging reference |

Existing voltage, capacitance, zero-calibration, oscillograph, and help
commands must retain their current behavior.

## Measurement

The command will use D3 through D6 only. Before and after every sample, it
will write `LOW` to D2 through D6 and then configure each as `INPUT`. Clearing
the output latch first prevents an inactive pin's internal pull-up from loading
A2. For each candidate range, the firmware will:

1. Configure its GPIO as `OUTPUT` and drive it HIGH.
2. Wait for the existing ADC settling interval.
3. Take the existing multi-sample ADC reading from A2.
4. Write the GPIO LOW and restore it to `INPUT`.

Resistance mode will select the `DEFAULT` (5 V) ADC reference before the
passive check and each range scan. Ranges with an ADC mean strictly greater
than 5 and strictly less than 1018 are valid candidates. The valid sample
nearest ADC mid-scale is selected because it provides the best relative divider
resolution. With `N` as its averaged ADC value and `Rref` as the selected
reference resistance, the calculated unknown resistance is:

```text
Rx = Rref * N / (1023 - N)
```

The result uses ohms, kilohms, or megohms and identifies the chosen reference
range. Reference-resistor constants should be easy to replace with measured
values; nominal resistor values do not establish measurement accuracy.

## Safety and errors

The 100-ohm D2 range is intentionally excluded. A probe short would draw
approximately 50 mA from a 5 V Arduino GPIO through 100 ohms, above a typical
GPIO absolute maximum. D2 remains high impedance throughout resistance mode.

Before enabling a reference GPIO, the command will take the existing
ten-sample passive reading of A2. A maximum sample above 5 ADC counts (about
24 mV with the explicitly selected 5 V reference) indicates a live or charged
input and stops the measurement. Firmware is not a substitute for hardware
protection: the user must measure only unpowered resistors.

The command reports explicit errors rather than a numeric result when:

- A2 indicates a live or charged input before excitation.
- Every safe range has a mean at or below 5 ADC counts, indicating a short or a
  resistance below the supported range.
- Every safe range has a mean at or above 1018 ADC counts, indicating an open circuit
  or a resistance above the supported range.

If one or more ranges are valid, the command reports the selected range and a
line beginning `RESISTANCE:`. Errors begin `ERROR:` and the help text adds
`r - measure resistance`.

## Validation

Compile the sketch using the available Arduino board target and inspect the
targeted diff to confirm only the new `r` command and directly related
documentation change behavior.
