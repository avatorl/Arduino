# Resistance Command Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a safe, auto-ranging `r` serial command that measures an
unpowered resistance connected from A2 to GND using calibrated D3 through D6
reference resistors.

**Architecture:** Extend the existing single-sketch command dispatcher with a
small resistance-meter unit in `arduino-multimeter.ino`. It will scan each
safe reference range with all other pins released, select the valid ADC sample
closest to mid-scale, and report an explicit result or error. D2 remains
released and unused because its 100-ohm reference is unsafe under a probe
short.

**Tech Stack:** Arduino C++ (`Arduino.h`), ATmega-style GPIO/ADC APIs,
`arduino-cli`.

---

## File Structure

| File | Responsibility |
| --- | --- |
| `arduino-multimeter/arduino-multimeter.ino` | Pin/range constants, `r` command dispatch, high-impedance cleanup, ADC range scan, resistance calculation, and serial output. |
| `arduino-multimeter/FUTURE-DEVELOPMENT-NOTES.md` | Replace the resistance feature's future-tense implementation note with the actual D3–D6 wiring, calibrated references, safety constraint, and command behavior. |
| `arduino-multimeter/docs/superpowers/specs/2026-09-25-resistance-command-design.md` | Approved behavioral contract; do not alter during implementation unless requirements change. |

### Task 1: Add safe resistance-meter primitives

**Files:**
- Modify: `arduino-multimeter/arduino-multimeter.ino:36-150`
- Test: Arduino CLI compilation of `arduino-multimeter`

- [ ] **Step 1: Add resistance pin, calibrated range, threshold, and result definitions**

Add constants for A2, D2, and the four measured reference values. Keep the
safe reference bank in a pin/value struct so a scan cannot accidentally select
D2:

```cpp
const uint8_t RESISTANCE_ANALOG_PIN = A2;
const uint8_t RESISTANCE_UNUSED_PIN = 2;
const float RESISTANCE_LIVE_INPUT_MAX_RAW = 5.0F;
const float RESISTANCE_OPEN_INPUT_MIN_RAW = 1018.0F;
const ResistanceRange RESISTANCE_RANGES[] = {
    {3, 1000.0F},
    {4, 9400.0F},
    {5, 71700.0F},
    {6, 480000.0F},
};
```

Define an explicit status enum and a `ResistanceResult` containing `ok`,
status, selected range index, and calculated ohms. Add prototypes for the
measurement, range scan, cleanup, formatter, and result printer near the
existing function declarations.

- [ ] **Step 2: Implement and inspect high-impedance cleanup**

Add `releaseResistanceCircuit()` that processes D2 through D6. For every pin,
call `digitalWrite(pin, LOW)` **before** `pinMode(pin, INPUT)`. This ordering
is mandatory: it clears any output latch so a released pin cannot enable its
internal pull-up and load A2.

Call this cleanup from `setup()` and from every resistance measurement exit
path. Do not modify `releaseCapacitanceCircuit()` or the existing command
implementations.

- [ ] **Step 3: Compile the sketch to verify the new declarations build**

First identify the connected board FQBN:

```powershell
arduino-cli board list
```

Then compile with the matching FQBN (for example only, `arduino:avr:nano`):

```powershell
arduino-cli compile --fqbn <connected-board-fqbn> --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all
```

Expected: successful compilation with no errors.

- [ ] **Step 4: Commit the primitives**

```powershell
git add -- arduino-multimeter/arduino-multimeter.ino
git commit -m "feat: add resistance meter primitives" -m "Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

### Task 2: Implement auto-ranging measurement and serial output

**Files:**
- Modify: `arduino-multimeter/arduino-multimeter.ino:150-724`
- Test: Arduino CLI compilation and bench-test matrix

- [ ] **Step 1: Add the `r` and `R` command cases and help text**

Add `r - measure resistance` to `printHelp()`. Add `r` and `R` cases to
`handleSerialCommands()` that call `printResistanceResult(measureResistance())`.
Keep every existing switch case exactly as-is.

- [ ] **Step 2: Implement the passive live-input guard**

At the beginning of `measureResistance()`:

1. Release D2 through D6.
2. Select the `DEFAULT` ADC reference, which is the 5 V AVCC rail used to
   drive the divider.
3. Read `readSettledSamples(RESISTANCE_ANALOG_PIN)`.
4. If its `maxRaw` is greater than 5, return the live-input status without
   enabling any GPIO range.

This must use `maxRaw`, not the mean, so one meaningful live/charged sample
cannot be averaged away.

- [ ] **Step 3: Implement one-range scan and range selection**

For each entry in `RESISTANCE_RANGES`:

1. Release all resistance pins.
2. Configure only that entry's pin as `OUTPUT`, then drive it `HIGH`.
3. Wait `ADC_SAMPLE_SPACING_MS`, then call
   `readSettledSamples(RESISTANCE_ANALOG_PIN)`.
4. Release all resistance pins.
5. Treat the mean as valid only when it is strictly greater than 5.0 and
   strictly less than 1018.0. Of valid candidates, preserve the one with the
   smallest `fabs(meanRaw - 511.5F)`.

If no candidate is valid, classify as too-low only if every candidate mean is
at or below 5.0; otherwise classify as open/too-high only if every candidate
mean is at or above 1018.0. The result contract permits no unclassified
outcome: if a future threshold edit makes neither condition true, return a
dedicated measurement-failed error instead of reporting a false value.

- [ ] **Step 4: Calculate and format the resistance result**

For the selected range use:

```cpp
result.ohms = referenceOhms * selectedMean / (1023.0F - selectedMean);
```

Do not use a measured or nominal VCC in this formula; the ADC reference and
the divider excitation are both AVCC, so it cancels. Format values below
1,000 as `ohm`, below 1,000,000 as `kohm`, and the rest as `Mohm`, using the
existing precision helper where appropriate. Print a normal result beginning:

```text
RESISTANCE: <value> <unit> | REF: <calibrated reference>
```

For every failure, print an `ERROR:` message that identifies live/charged
input, short/too-low, open/too-high, or an unexpected scan failure. In all
cases, leave D2 through D6 released and restore `DEFAULT` before returning.

- [ ] **Step 5: Compile to verify the full command**

```powershell
arduino-cli compile --fqbn <connected-board-fqbn> --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all
```

Expected: successful compilation with no errors.

- [ ] **Step 6: Run the physical bench-test matrix**

With the stated wiring, test only unpowered resistors connected between A2 and
GND. Send `r` at 115200 baud and record the result:

| Test input | Expected behavior |
| --- | --- |
| A2 open | `ERROR:` for open/too-high; all range pins released afterward |
| A2 shorted to GND | `ERROR:` for short/too-low; D2 is never driven |
| Known resistor near 1 kΩ | `RESISTANCE:` and D3 reference |
| Known resistor near 10 kΩ | `RESISTANCE:` and D4 reference |
| Known resistor near 100 kΩ | `RESISTANCE:` and D5 reference |
| Known resistor near 0.48–1 MΩ | `RESISTANCE:` and D6 reference |
| Small external voltage on A2 | `ERROR:` for live/charged input before any range is enabled |
| `v`, `c`, `z`, `o`, `h` | Existing command behavior unchanged |

Do not use D2 as a reference range in any test.

- [ ] **Step 7: Commit the command implementation**

```powershell
git add -- arduino-multimeter/arduino-multimeter.ino
git commit -m "feat: add auto-ranging resistance command" -m "Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

### Task 3: Update the project resistance documentation

**Files:**
- Modify: `arduino-multimeter/FUTURE-DEVELOPMENT-NOTES.md:25-98`
- Test: Markdown review and `git diff --check`

- [ ] **Step 1: Update the resistance-meter section**

Replace the future-tense generic range proposal with the implemented wiring:
A2 as the sense node, unknown resistor from A2 to GND, and D3=1.0 kΩ,
D4=9.4 kΩ, D5=71.7 kΩ, D6=0.48 MΩ as auto-ranging references. State the
correct divider equation `Rx = Rref * ADC / (1023 - ADC)`.

Document that D2's 100-ohm resistor is deliberately unused because a short
would demand approximately 50 mA from an Arduino GPIO. Record the `r` command,
the unpowered-resistor requirement, and the live-input guard's limitation:
physical protection is still required against energized circuits.

- [ ] **Step 2: Check documentation whitespace and scope**

```powershell
git diff --check -- arduino-multimeter/FUTURE-DEVELOPMENT-NOTES.md
git diff -- arduino-multimeter/FUTURE-DEVELOPMENT-NOTES.md
```

Expected: only resistance-mode documentation changes; no whitespace errors.

- [ ] **Step 3: Commit the documentation**

```powershell
git add -- arduino-multimeter/FUTURE-DEVELOPMENT-NOTES.md
git commit -m "docs: document multimeter resistance mode" -m "Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

### Task 4: Final integration validation

**Files:**
- Modify: `arduino-multimeter/arduino-multimeter.ino` only if the compile or
  bench-test matrix exposes a resistance-mode defect
- Test: Arduino CLI compilation and targeted diff

- [ ] **Step 1: Compile the final sketch**

```powershell
arduino-cli compile --fqbn <connected-board-fqbn> --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all
```

Expected: successful compilation with no errors.

- [ ] **Step 2: Verify the targeted change set**

```powershell
git diff --check HEAD~3..HEAD -- arduino-multimeter
git status --short
```

Expected: no whitespace errors; only the planned resistance implementation and
documentation commits, plus any pre-existing user changes that were not
staged or overwritten.

- [ ] **Step 3: Commit any validation-driven correction**

Only if a correction was necessary:

```powershell
git add -- arduino-multimeter/arduino-multimeter.ino arduino-multimeter/FUTURE-DEVELOPMENT-NOTES.md
git commit -m "fix: refine resistance measurement" -m "Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```
