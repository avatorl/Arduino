# Dual-ADC Resistance Measurement Design

## Goal

Update only the resistance-meter path in `arduino-multimeter.ino` to use two
Arduino ADC inputs. This compensates for the voltage drop of the selected
Arduino GPIO excitation pin while retaining the existing auto-ranging
reference-resistor bank.

No voltage, capacitance, oscillograph, command, or pin behavior outside
resistance mode changes.

## Wiring

For every resistance range:

```text
selected GPIO ---- A2 ---- Rref ---- A3 ---- Rx ---- GND
```

- **A2** is the source node: the selected GPIO and upper end of the active
  reference resistor.
- **A3** is the divider junction: the lower end of the active reference
  resistor and upper end of the unknown resistor.
- **Rx** is the unpowered resistor under test between A3 and GND.

The existing D2-D6 range-switching topology remains in use. Inactive GPIOs
remain low and high impedance as implemented by `releaseResistanceCircuit()`.

## Measurement

Resistance mode continues to select the default AVCC ADC reference and enables
one reference range at a time. Once the active range has settled, the firmware
collects paired readings from A2 and A3. It must discard an initial ADC
conversion after changing between these ADC channels so the ADC sample-and-hold
capacitor settles before retaining a paired value.

For each retained pair:

```text
sourceRaw   = A2 ADC result
junctionRaw = A3 ADC result
referenceDropRaw = sourceRaw - junctionRaw
Rx = Rref * junctionRaw / referenceDropRaw
```

The calculation is ratiometric: it does not depend on nominal AVCC or the
selected GPIO's unloaded HIGH voltage. Measuring A2 accounts for the GPIO
output's load-dependent voltage drop across its output resistance.

The range selector continues to prefer the candidate with the best estimated
relative ADC resolution, while rejecting invalid, unstable, short/below-range,
and open/above-range cases. The existing lower and upper rail limits apply to
the A3 junction reading only; A2 is expected to be near full scale because it
is driven directly by the active GPIO. A candidate is invalid when the
source-to-junction difference is at or below the lower raw-value limit.

Stability checks apply to both endpoint readings and to their difference. A
pair is stable only when the source and junction samples meet the existing
noise, spread, and successive-window-change limits and their difference stays
positive. The existing cross-range consistency screen uses the calculated
resistance to predict each range's A3 junction reading.

## Safety and output

The existing resistance-mode cleanup and GPIO switching behavior remain
unchanged. Before enabling each reference GPIO, the firmware releases all
range pins, selects the default ADC reference, and reads settled samples at
**A3**, the test-resistor node. A maximum reading greater than the existing
lower raw-value limit is a live-or-charged-input failure: the firmware reports
an explicit error and does not enable that range. Resistance measurements
remain limited to unpowered components. Firmware live-input checks do not
replace external input protection.

Per-range `STATUS:` output reports the averaged A2 source value, A3 junction
value, and calculated estimate. The existing result and error formatting
remains otherwise unchanged.

## Validation

Compile only the `arduino-multimeter` sketch with its documented Arduino CLI
command. Bench test resistance mode with known resistors across the existing
reference ranges, an open socket, and a shorted A3-to-GND input. Confirm that
the non-resistance commands retain their current behavior.
