# Resistance Median Sampling Design

**Goal:** Make each resistance range measurement resistant to isolated ADC noise by using the median of 32 corrected readings.

## Scope

Only `arduino-multimeter.ino` resistance sampling changes. Voltage, capacitance, and oscilloscope sampling remain unchanged.

## Design

For each active D2-D6 reference-resistor range, the firmware already takes 32 paired readings. Each pair is corrected as:

```text
correctedReading = A2 junction reading - A3 ground-reference reading
```

The sampler will retain the 32 corrected integer readings in a local 64-byte `int` array, then use insertion sort to arrange it in ascending order. It will set its central value to the average of the two middle readings: sorted positions 16 and 17 in human counting (array indexes 15 and 16). This is the median for an even-sized set.

Minimum, maximum, and standard deviation continue to describe all 32 corrected readings. A separate local running-mean value remains the Welford variance accumulator; it must not use or alter the returned central-value field after that field is assigned the median.

The median becomes the central raw value used by the resistance calculation, range selection, stability comparison, and diagnostics. A resistance-only stats structure will carry this value in a `medianRaw` field, plus the existing minimum, maximum, and standard-deviation values. Its functions and call sites will use the resistance-only type.

The shared `SampleStats` structure and its `meanRaw` field remain unchanged because voltage sampling correctly uses an arithmetic mean. Resistance diagnostics will label the new central value as `median_raw`, while voltage diagnostics retain their existing mean terminology.

## Validation

Compile the temporary Uno mapping:

```powershell
arduino-cli compile --fqbn arduino:avr:uno --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all
```
