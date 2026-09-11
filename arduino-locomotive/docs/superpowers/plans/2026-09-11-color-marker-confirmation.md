# Color Marker Confirmation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make color-marker actions require confirmed readings, keep sensing active during configurable visual feedback, and configure each marker's feedback LED color in `config.h`.

**Architecture:** The confirmation state machine remains owned by `41-color-sensor.ino`. Marker feedback duration and marker-to-LED-color mapping move into typed configuration in `config.h`; the sensor tab reads the flash-resident mapping instead of hard-coding a switch. The active implementation continues using `Adafruit_TCS34725`.

**Tech Stack:** Arduino C++, Adafruit TCS34725, AVR `PROGMEM`, Arduino CLI, existing native PowerShell test runner

---

## File Structure

- Modify `arduino-locomotive/config.h`: add confirmation parameters, comment out the unused custom-driver address constant, preserve all clear/saturation threshold behavior, and improve relevant comments.
- Modify `arduino-locomotive/41-color-sensor.ino`: use the configured feedback duration and marker-color table instead of hard-coded values.
- Modify `arduino-locomotive/arduino-locomotive.ino`: remove the duplicate `RgbColor` definition after moving the type into `config.h`.
- Modify `arduino-locomotive/.github/copilot-instructions.md`: state that the color sensor uses Adafruit TCS34725 while distance sensors remain register-level.
- Modify `arduino-locomotive/docs-internal/LEARN.md`: replace stale hand-written color-driver, file-name, and color-sensor pin descriptions.
- Modify `arduino-locomotive/docs-internal/TECHNICAL.md`: describe the color tab as Adafruit-library integration rather than a custom driver.

### Task 1: Add Color Confirmation Configuration

**Files:**
- Modify: `arduino-locomotive/config.h:85-90`
- Modify: `arduino-locomotive/config.h:222-251`

- [ ] **Step 1: Remove obsolete configuration**

Comment out `tcs34725Address` and explain that the active Adafruit library uses
the TCS34725's fixed default `0x29` address. Do not remove or change
`colorPresenceClearThreshold`, `colorSaturationClearThreshold`, cluster
`minClearThreshold` values, or their classification logic.

- [ ] **Step 2: Add typed confirmation constants**

Add these constants beside `colorSensorReadEveryMs`:

```cpp
constexpr uint8_t colorMarkerConfirmationSamples = 2;
constexpr uint8_t colorMarkerLeaveSamples = 2;
constexpr unsigned long colorMarkerFeedbackDurationMs = 1000UL;
```

Explain that the first constant controls both initial and different-color
confirmation, while the second controls confirmed marker leave detection.
Both operate on consecutive samples without a time window.

- [ ] **Step 3: Move the shared RGB color type into configuration**

Move the existing `enum class RgbColor : uint8_t` definition from
`arduino-locomotive.ino` into the color-sensor configuration section before
the marker feedback table. Preserve all existing enum values and ordering.

- [ ] **Step 4: Add the configurable marker feedback table**

Add:

```cpp
const RgbColor markerFeedbackColors[] PROGMEM = {
  RgbColor::Off,      // MarkerUnknown
  RgbColor::White,    // MarkerWhite
  RgbColor::Blue,     // MarkerBlue
  RgbColor::Green,    // MarkerGreen
  RgbColor::Magenta,  // MarkerMagenta
  RgbColor::Yellow,   // MarkerYellow
  RgbColor::Red       // MarkerRed
};
constexpr uint8_t markerFeedbackColorCount =
  sizeof(markerFeedbackColors) / sizeof(markerFeedbackColors[0]);
static_assert(markerFeedbackColorCount == MarkerClassCount,
              "markerFeedbackColors must contain one entry per marker class.");
```

Add `MarkerClassCount` after `MarkerRed` in `TrackMarkerClass`. It is a count
sentinel, not a detectable marker. The assertion prevents a future marker from
silently indexing beyond the table.

- [ ] **Step 5: Improve calibration comments without changing sample labels**

Explain the 24 ms integration/4x gain setting, normalized RGB values, the
existing presence/saturation checks, the per-cluster clear threshold, and that
normalized components may total approximately 1000 because of rounding.
Preserve every existing
`// printed`, `// original`, and empty `//` marker-table comment exactly.

- [ ] **Step 6: Review the scoped configuration diff**

Run `git --no-pager diff -- arduino-locomotive\config.h` and distinguish the
new confirmation edits from the user's existing uncommitted calibration work.
Do not stage or commit the dirty working-tree files.

### Task 2: Implement Confirmed Marker Entry and Leave

**Files:**
- Modify: `arduino-locomotive/41-color-sensor.ino:11-173`
- Modify: `arduino-locomotive/41-color-sensor.ino:247-329`
- Modify: `arduino-locomotive/arduino-locomotive.ino:224-231`

- [ ] **Step 1: Replace the single previous-class variable with explicit state**

Replace `lastTrackMarkerClass` with:

```cpp
uint8_t confirmedTrackMarkerClass = MarkerUnknown;
uint8_t candidateTrackMarkerClass = MarkerUnknown;
uint8_t candidateTrackMarkerSamples = 0;
uint8_t markerLeaveSamples = 0;
```

Add small reset helpers for pending entry, pending leave, and all detection
state. Comments must explain why confirmed state and candidate state are
separate.

- [ ] **Step 2: Add the sample-processing state machine**

Add a helper that accepts the classified sample.

When no marker is confirmed:

```cpp
if (markerClass == MarkerUnknown) {
  resetMarkerCandidate();
  return;
}

if (candidateTrackMarkerClass != markerClass
    || candidateTrackMarkerSamples == 0) {
  candidateTrackMarkerClass = markerClass;
  candidateTrackMarkerSamples = 1;
  return;
}

++candidateTrackMarkerSamples;
if (candidateTrackMarkerSamples >= colorMarkerConfirmationSamples) {
  confirmedTrackMarkerClass = markerClass;
  resetMarkerCandidate();
  showMarkerFeedbackAndRunAction(markerClass);
}
```

When a marker is already confirmed, the same known reading resets pending leave
and different-color confirmation. Unknown readings must be consecutive; once
`colorMarkerLeaveSamples` is reached, clear the confirmed marker. A different
known color resets the leave count and starts or continues candidate
confirmation; once `colorMarkerConfirmationSamples` is reached, directly
replace the confirmed marker and run the new action.

When `momentaryActive` pauses color reads, clear pending candidate and leave
counts before returning. Keep `confirmedTrackMarkerClass` unchanged so a
half-complete sequence cannot span the jog pause.

- [ ] **Step 3: Extract confirmed-marker feedback**

Move the marker-to-`RgbColor` switch, one-second feedback setup, and
`handleTrackMarkerAction()` call into a focused helper used only when entry is
confirmed. Do not pause sampling while `markerColorBlinkActive` is true.

- [ ] **Step 4: Preserve clear and saturation classification**

Leave the `rawC > colorSaturationClearThreshold` branch and all existing
cluster-specific minimum-clear behavior unchanged.

- [ ] **Step 5: Remove dead color data and unreachable code**

Remove `BalancedRgbs::c`, remove the `c` argument and assignment from
`applyWhiteBalance()`, update its two callers, and remove the duplicate
unreachable `break` in the red-marker action.

- [ ] **Step 6: Correct implementation comments**

Document that:

- the constructor uses 24 ms integration and 4x gain;
- Adafruit `getRawData()` performs library-managed channel reads and waits for
  an integration interval;
- the periodic updater is cooperative except for that library delay and must
  not be described as non-blocking;
- confirmation prevents isolated readings from executing actions;
- confirmed absence is required before the same marker can run again, while
  two consecutive samples of a different known color may directly run the new
  marker action.

- [ ] **Step 7: Review the scoped detection diff**

Run `git --no-pager diff -- arduino-locomotive\41-color-sensor.ino arduino-locomotive\arduino-locomotive.ino`.
Do not stage or commit these files because they already contain the user's
uncommitted work.

### Task 3: Use Configurable Marker Feedback

**Files:**
- Modify: `arduino-locomotive/41-color-sensor.ino:267-292`
- Modify: `arduino-locomotive/arduino-locomotive.ino:208-223`

- [ ] **Step 1: Remove the duplicate RGB enum**

Delete the `RgbColor` definition from `arduino-locomotive.ino`; `config.h` is
included first and now owns the shared type.

- [ ] **Step 2: Read the configured marker color**

Replace the hard-coded marker-class switch in
`showMarkerFeedbackAndRunAction()` with:

```cpp
RgbColor markerColor = (RgbColor)pgm_read_byte(
  &markerFeedbackColors[markerClass]
);
```

The state machine calls this helper only for confirmed known marker values, but
retain the `RgbColor::Off` check so configuration can intentionally suppress a
marker's visual feedback without disabling its action.

- [ ] **Step 3: Use the configured duration**

Replace:

```cpp
markerColorBlinkEndsAt = millis() + 1000UL;
```

with:

```cpp
markerColorBlinkEndsAt = millis() + colorMarkerFeedbackDurationMs;
```

- [ ] **Step 4: Update beginner comments**

Explain that `markerFeedbackColors` chooses visual feedback independently from
the marker action, and `colorMarkerFeedbackDurationMs` controls its duration
without pausing sensor sampling.

- [ ] **Step 5: Review the scoped feedback diff**

Confirm the switch and hard-coded duration are gone, the mapping remains in
`PROGMEM`, and no threshold or action behavior changed.

### Task 4: Decouple Blink Lifetime From Sensor Sampling

**Files:**
- Modify: `arduino-locomotive/41-color-sensor.ino:44-91`
- Modify: `arduino-locomotive/41-color-sensor.ino:247-329`

- [ ] **Step 1: Service visual expiry before sensor guards**

At the start of `updateColorSensor()`, capture `millis()` and clear expired
marker feedback before checking `ColorSensorOnOff`, detection status, or jog
state. Keep the existing rollover-safe target-time comparison.

- [ ] **Step 2: Continue sampling during feedback**

Remove the return from the active-blink branch. An unexpired blink should leave
the displayed color in place while classification and entry/leave state
continue advancing.

- [ ] **Step 3: Clear blink and detection state on disable**

In the `enabled == false` path of `setColorSensorEnabled()`:

```cpp
markerColorBlinkActive = false;
resetTrackMarkerDetectionState();
```

Perform this before `refreshDriveLights()` so the refresh cannot be blocked by
stale blink state. Also reset detection state when enabling so each enabled
session begins from fresh samples.

- [ ] **Step 4: Review required sequences directly**

Trace the implementation for:

- two consecutive matching samples confirm once;
- a conflicting color restarts entry confirmation;
- two consecutive unknown samples rearm;
- `unknown -> confirmed marker -> unknown` does not rearm;
- two consecutive readings of a different known color directly confirm it;
- a single different-color reading followed by the confirmed color does not
  transition;
- a momentary-jog pause clears partial counts but preserves the confirmed
  marker;
- a blink does not stop entry or leave confirmation;
- disabling during a blink clears the blink and restores normal lights.

Do not add native color-specific tests.

- [ ] **Step 5: Recheck the active color-sensor diff**

Confirm that the blink fix changes only marker-feedback timing and detection
state. Do not stage or commit the dirty source file.

### Task 5: Correct Color Sensor Documentation

**Files:**
- Modify: `arduino-locomotive/.github/copilot-instructions.md:50-80`
- Modify: `arduino-locomotive/docs-internal/LEARN.md:1-230`
- Modify: `arduino-locomotive/docs-internal/TECHNICAL.md:1-60`

- [ ] **Step 1: Correct project implementation guidance**

State that the active TCS34725 implementation uses the repository-local
Adafruit TCS34725 and Adafruit BusIO libraries. Keep the instruction to retain
register-level distance-sensor drivers.

- [ ] **Step 2: Correct the beginner guide**

Update stale references to `arduino-train-v2.ino`, `40-sensors.ino`, D4 color
LED wiring, and a hand-written color driver. Describe `41-color-sensor.ino`,
A2, the Adafruit library, 24 ms integration, confirmation, and confirmed-leave
rearming in beginner-friendly language.

- [ ] **Step 3: Correct the technical reference**

Describe `41-color-sensor.ino` as Adafruit TCS34725 integration plus marker
classification and action handling.

- [ ] **Step 4: Review documentation changes**

Inspect the documentation diff for accurate Adafruit-library, A2 wiring, and
confirmation descriptions. Do not stage or commit project files unless the
user separately requests a commit.

### Task 6: Validate the Complete Firmware

**Files:**
- Verify: `arduino-locomotive/config.h`
- Verify: `arduino-locomotive/41-color-sensor.ino`
- Verify: `arduino-locomotive/arduino-locomotive.ino`
- Verify: `arduino-locomotive/.github/copilot-instructions.md`
- Verify: `arduino-locomotive/docs-internal/LEARN.md`
- Verify: `arduino-locomotive/docs-internal/TECHNICAL.md`

- [ ] **Step 1: Compile the production sketch**

```powershell
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-locomotive --warnings all
```

Expected: compilation succeeds for the Arduino Nano. Existing warnings outside
the color-sensor change may remain, but no new color-sensor warnings are
introduced.

- [ ] **Step 2: Run the existing native suite**

```powershell
& D:\GITHUB\Arduino\arduino-locomotive\test\native\run-tests.ps1
```

Expected: the existing suite passes. Do not add color-specific native tests.

- [ ] **Step 3: Inspect the final scoped diff**

```powershell
git --no-pager diff -- arduino-locomotive\config.h arduino-locomotive\41-color-sensor.ino arduino-locomotive\arduino-locomotive.ino arduino-locomotive\.github\copilot-instructions.md arduino-locomotive\docs-internal\LEARN.md arduino-locomotive\docs-internal\TECHNICAL.md
```

Expected: the approved color-marker changes coexist with the user's existing
uncommitted edits; `arduino-locomotive\backup` remains untouched. Review the
specific edited hunks rather than treating the full dirty diff as newly
created by this implementation.
