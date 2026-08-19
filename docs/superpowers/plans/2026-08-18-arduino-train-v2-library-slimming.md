# Arduino Train v2 Library Slimming Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the train sketch's heavy device-library usage with smaller sketch-local helpers so Nano flash usage drops while existing train functionality stays intact.

**Architecture:** Make a backup of the current sketch first, then keep `arduino-train-v2.ino` as the single runtime sketch while swapping the TCS34725, MCP23008, and VL53L0X backends from third-party library objects to minimal `Wire`-based helpers. Keep higher-level train behavior stable and validate both correctness and flash reduction with Nano builds after each replacement pass.

**Tech Stack:** Arduino C++, AVR Arduino core, `Wire`, existing `IRremote` and `LowPower` libraries, `arduino-cli`.

---

## File Structure

- Create: `arduino-train-v2\arduino-train-v2.backup-before-library-slimming.ino` — backup copy of the pre-slimming sketch.
- Modify: `arduino-train-v2\arduino-train-v2.ino` — replace device-library usage with sketch-local helpers while preserving behavior.
- Modify: `arduino-train-v2\README.md` only if setup/build expectations materially change.
- Test: `arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2`

### Task 1: Back up the current sketch and add the MCP23008 helper

**Files:**
- Create: `arduino-train-v2\arduino-train-v2.backup-before-library-slimming.ino`
- Modify: `arduino-train-v2\arduino-train-v2.ino`
- Test: `arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2`

- [ ] **Step 1: Capture the baseline flash size**

Run:

```text
arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2
```

Record the reported flash usage as the pre-slimming baseline for later
comparison.

- [ ] **Step 2: Create the backup copy**

Copy the current `arduino-train-v2.ino` verbatim to
`arduino-train-v2.backup-before-library-slimming.ino` before changing device
backends.

- [ ] **Step 3: Add a minimal MCP23008 helper**

Implement sketch-local `Wire` helpers for:
- device presence check
- register write
- GPIO direction setup
- digital output write

Limit the helper to the train's current LED-expander use case only.

- [ ] **Step 4: Replace MCP23008 object usage**

Swap the current `Adafruit_MCP23X08` calls in LED initialization and output
paths to the new helper while keeping the existing LED-routing behavior.

- [ ] **Step 5: Remove the old MCP23008 library dependency**

Delete the `Adafruit_MCP23X08` include and any no-longer-used global expander
object declaration once the helper fully replaces its call sites.

- [ ] **Step 6: Compile after the MCP23008 swap**

Run:

```text
arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2
```

Expected: build succeeds and flash usage is not worse than the current baseline.

- [ ] **Step 7: Document any MCP23008 fallback explicitly if needed**

If the MCP23008 replacement proves too risky, restore only that device's
original backend and note the fallback decision in the README or an adjacent
implementation note before continuing.

### Task 2: Replace the TCS34725 library with a minimal helper

**Files:**
- Modify: `arduino-train-v2\arduino-train-v2.ino`
- Test: `arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2`

- [ ] **Step 1: Add a minimal TCS34725 helper**

Implement sketch-local `Wire` helpers for:
- device ID/probe
- integration-time and gain setup matching 50 ms / 4x
- enable / disable
- raw RGBC reads

- [ ] **Step 2: Replace current sensor lifecycle calls**

Swap `begin()`, `enable()`, `disable()`, and `getRawData()` usage to the new
helper while preserving the current sensor on/off and marker-detection flow.

- [ ] **Step 3: Remove the old TCS34725 library dependency**

Delete the `Adafruit_TCS34725` include and any no-longer-used global sensor
object declaration once the helper fully replaces its call sites.

- [ ] **Step 4: Preserve current classification inputs**

Verify the new helper still feeds the same raw RGBC path into the train's
cluster-based classifier without changing higher-level marker logic.

- [ ] **Step 5: Compile after the TCS34725 swap**

Run:

```text
arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2
```

Expected: build succeeds and flash usage drops versus the prior pass.

- [ ] **Step 6: Document any TCS34725 fallback explicitly if needed**

If the TCS34725 replacement proves too risky, restore only that device's
original backend and note the fallback decision explicitly before continuing.

### Task 3: Replace the VL53L0X library with a minimal helper

**Files:**
- Modify: `arduino-train-v2\arduino-train-v2.ino`
- Test: `arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2`

- [ ] **Step 1: Add a minimal VL53L0X helper**

Implement sketch-local `Wire` helpers for:
- probe/init
- timeout configuration
- address set to the train's runtime value
- timing-budget setup
- continuous ranging start
- current range read
- timeout/invalid-read detection

- [ ] **Step 2: Support repeated re-initialization paths**

Preserve the current wake/rearm behavior by making sure the helper supports
the same address-set and continuous-start flow at startup, after sleep, and
after motor-driver rearm paths.

- [ ] **Step 3: Replace current VL53L0X method calls**

Swap the existing Pololu library calls to the helper while keeping obstacle
speed control behavior intact.

- [ ] **Step 4: Remove the old VL53L0X library dependency**

Delete the `VL53L0X` include and any no-longer-used global object declaration
once the helper fully replaces its call sites.

- [ ] **Step 5: Compile after the VL53L0X swap**

Run:

```text
arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2
```

Expected: build succeeds and flash usage is measurably lower than the starting
baseline.

- [ ] **Step 6: Document any VL53L0X fallback explicitly if needed**

If the VL53L0X replacement proves too risky, restore only that device's
original backend and note the fallback decision explicitly before continuing.

### Task 4: Validate flash reduction and document any changed expectations

**Files:**
- Modify: `arduino-train-v2\README.md` only if necessary
- Test: `arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2`

- [ ] **Step 1: Compare before/after flash usage**

Record the final Nano compile size and compare it to the pre-slimming
baseline.

- [ ] **Step 2: Update README only if needed**

If the implementation changes any hardware/setup/build expectation, document it
briefly in `README.md`. Otherwise leave the README untouched.

- [ ] **Step 3: Run the final Nano build**

Run:

```text
arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2
```

Expected: build succeeds and is smaller than the baseline.

- [ ] **Step 4: Keep changes local**

Do not create a Git commit. The user explicitly wants local changes only.
