# Shared Multimeter Sampling Design

**Goal:** Use one median/statistics helper in voltage, capacitance, and resistance measurement modes, while retaining mode-specific sample counts and acquisition methods.

## Shared statistics

`calculateSampleStats(float values[], int count)` will accept a float sample array and its count, sorting the caller's temporary array in place. All three modes will collect values into float arrays before calling it. It will calculate:

- median;
- arithmetic mean;
- standard deviation;
- minimum and maximum.

It will use insertion sort. For an odd count, the median is the one center reading. For an even count, it is the average of the two center readings. Standard deviation uses the existing population formula, dividing the sum of squared deviations by the sample count. The median is the only central value used for reported measurements, stability checks, and range decisions; mean and standard deviation are diagnostic Serial output.

## Mode integration

| Mode | Samples | Values collected | Measurement value |
| --- | ---: | --- | --- |
| Voltage | `VOLTAGE_SAMPLES` (10) | Settled ADC readings | Median raw ADC reading |
| Resistance | `RESISTANCE_SAMPLES` (32) | Junction ADC reading minus ground-reference ADC reading | Median corrected raw ADC reading |
| Capacitance | `CAPACITANCE_SAMPLES` (3) | Completed capacitance readings normalized to nF | Median nF value |

Each mode retains its existing hardware preparation, reference selection, timing, error handling, and range-selection behavior. Voltage's reported value intentionally changes from the existing arithmetic mean of 10 readings to their median. The capacitance baseline calibration will also use the helper to remove its duplicate median-of-three sort.

Capacitance samples are converted to nF before passing them to the helper. Because all acquisition errors return immediately, the returned median result sets `ok` to true. After the helper runs, the firmware will select the first completed `CapacitanceResult` whose nF value equals the median to retain its `usedPrecisionRange` field; ties select the first acquired sample. This requires an odd capacitance sample count, so `CAPACITANCE_SAMPLES` remains 3. The final result unit is re-derived from the median nF value with the existing `CAP_MICROFARAD_DISPLAY_THRESHOLD` rule.

## Diagnostics

Serial diagnostics will identify the `median`, `mean`, and `sd` values for each completed measurement. Mean and standard deviation do not influence the reported voltage, capacitance, or resistance.

## Validation

Compile the temporary Uno mapping:

```powershell
arduino-cli compile --fqbn arduino:avr:uno --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all
```

Then confirm source-level odd/even behavior: a three-value sample selects its center; a four-value sample averages its two center values.
