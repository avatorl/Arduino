# RGB Color Sensor LED Feedback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive a common-anode RGB LED with a stable representative palette for each confirmed TCS34725 sample, holding it for five seconds after detection.

**Architecture:** Keep classification and calibration unchanged in the existing sketch. Add a small RGB output layer for pin configuration, common-anode PWM inversion, sample-to-palette lookup, and rollover-safe timeout state; call it only after the current two-of-three detection consensus.

**Tech Stack:** Arduino C++, Adafruit TCS34725 library, Arduino AVR core, `arduino-cli`.

---

## File Structure

- Modify: `rgb-color-sensor/rgb-color-sensor.ino` — Defines the new D2/D3/D5/D6 pin assignments, common-anode RGB helpers, explicit palette values, non-blocking five-second hold state, and the integration points in `setup()` and `loop()`.
- Create: none — the repository has no Arduino sketch test framework, so validate with an Uno compilation and manual hardware behavior checks.

### Task 1: Define RGB LED output primitives

**Files:**
- Modify: `rgb-color-sensor/rgb-color-sensor.ino: pin constants and helper functions near PrototypeRgb`
- Test: manual Uno hardware verification

- [ ] **Step 1: Add the explicit pin and timing constants**

Replace the current sensor illumination pin assignment with D2. Add D3, D5,
and D6 constants for red, green, and blue; add a `5000UL` RGB hold duration.
Keep the existing sensor LED active-high constants. Add a small RGB value
struct or constants suitable for the fixed five-color palette.

- [ ] **Step 2: Add the common-anode output helper**

Implement a helper that accepts red, green, and blue brightness values and
writes `255 - value` to D3, D5, and D6 with `analogWrite`. Implement an
off helper that writes 255 to all three channels through that helper.

- [ ] **Step 3: Add the sample palette lookup**

Implement a helper that maps the existing sample indexes to exactly these
values: Jade White `(255, 255, 255)`, Bambu Green `(0, 255, 80)`, Cyan
`(0, 255, 255)`, Pumpkin Orange `(255, 85, 0)`, and Maroon Red
`(128, 0, 0)`. Return failure for an invalid index rather than lighting an
arbitrary color.

- [ ] **Step 4: Compile the sketch for Uno**

Run: `arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor`

Expected: compilation succeeds with no type or missing-symbol errors.

### Task 2: Integrate confirmed-detection output and timeout

**Files:**
- Modify: `rgb-color-sensor/rgb-color-sensor.ino: setup() and loop()`
- Test: manual Uno hardware verification

- [ ] **Step 1: Add LED state**

Add an `unsigned long` timestamp for the last confirmed sample and a boolean
that records whether an RGB output is active. Initialize both to a safe
off-state value.

- [ ] **Step 2: Initialize hardware output**

In `setup()`, configure D3, D5, and D6 as outputs and call the off helper
before sensor initialization. Preserve the sensor LED setup with D2. Change
the successful initialization message to exactly `Sensor LED on D2 enabled`.

- [ ] **Step 3: Update output only after consensus**

In the existing `detectedSampleIndex >= 0` branch, apply the representative
palette, set the active flag, and store `millis()` as the last-confirmed
timestamp. Do not change the output or timestamp in the skipped-batch branch,
so a skipped batch does not cancel the hold.

- [ ] **Step 4: Add the rollover-safe timeout**

Immediately after serial polling and before any color-sensor or calibration
early return, if output is active and
`millis() - lastConfirmedMs >= 5000UL`, turn the RGB LED off and clear the
active flag. This lets the timeout expire during calibration as well as during
ordinary detection, without blocking sensor sampling, calibration, or serial
polling.

- [ ] **Step 5: Compile the final sketch for Uno**

Run: `arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor`

Expected: compilation succeeds.

- [ ] **Step 6: Verify the hardware behavior**

Upload the sketch to the Uno and present each existing calibration sample.
Confirm a two-of-three consensus lights its documented palette color; confirm
the color remains through skipped batches; confirm it turns off on the first
loop iteration at or after five seconds; confirm another detection within
five seconds changes or refreshes the output; and confirm `cal` and `stop`
continue to work.

- [ ] **Step 7: Keep changes local**

Do not create a Git commit. The user explicitly requested local changes only.
