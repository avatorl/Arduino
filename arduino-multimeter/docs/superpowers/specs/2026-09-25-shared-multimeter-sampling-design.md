# Shared Multimeter Sampling Design

**Goal:** Use one median/statistics helper in voltage, capacitance, and resistance measurement modes, while retaining mode-specific sample counts and acquisition methods.

## Shared statistics

`calculateSampleStats(float values[], int count)` will accept a float sample array and its count, sorting the caller's temporary array in place. All three modes will collect values into float arrays before calling it. It will return a shared `SampleStats` structure with `medianRaw`, `meanRaw`, `stdDevRaw`, `minRaw`, and `maxRaw` fields.

- median;
- arithmetic mean;
- standard deviation;
- minimum and maximum.

It will use insertion sort. For an odd count, the median is the one center reading. For an even count, it is the average of the two center readings. Standard deviation uses the existing population formula, dividing the sum of squared deviations by the sample count. The median is the central value used for reported measurements and stability comparisons. Mean is diagnostic-only; standard deviation remains diagnostic except for the existing resistance range-quality scoring, which it continues to drive unchanged.

## Mode integration

| Mode | Samples | Values collected | Measurement value |
| --- | ---: | --- | --- |
| Voltage | `VOLTAGE_SAMPLES` (10) | Settled ADC readings | Median raw ADC reading |
| Resistance | `RESISTANCE_SAMPLES` (32) | Junction ADC reading minus ground-reference ADC reading | Median corrected raw ADC reading |
| Capacitance | `CAPACITANCE_SAMPLES` (3) | Completed capacitance readings normalized to nF | Median nF value |

Each mode retains its existing hardware preparation, reference selection, timing, error handling, and range-selection behavior. Voltage's reported value intentionally changes from the existing arithmetic mean of 10 readings to their median. The capacitance baseline calibration will also use the helper to remove its duplicate median-of-three sort.

Capacitance samples are converted to nF before passing them to the helper. Because all acquisition errors return immediately, the returned median result sets `ok` to true. After the helper runs, the firmware will select the first completed `CapacitanceResult` whose nF value equals the median to retain its `usedPrecisionRange` field; ties select the first acquired sample. This requires an odd capacitance sample count, so `CAPACITANCE_SAMPLES` remains 3. The final result first converts the median nF value to uF by dividing by 1000, then applies the existing `CAP_MICROFARAD_DISPLAY_THRESHOLD` rule; below that threshold it converts back to nF for display.

## Diagnostics

Serial diagnostics will identify the `median`, `mean`, and `sd` values for each completed measurement. Mean does not influence any measurement. Standard deviation does not influence reported voltage or capacitance; it retains its existing use in resistance range-quality scoring.

The oscillograph's existing reference-selection preview also calls the voltage sampler. It is outside this change's three measurement modes and will continue using `meanRaw` for that preview only.

## Validation

Compile the temporary Uno mapping:

```powershell
arduino-cli compile --fqbn arduino:avr:uno --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all
```

Then confirm source-level odd/even behavior: a three-value sample selects its center; a four-value sample averages its two center values.
