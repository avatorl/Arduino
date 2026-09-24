# Arduino Multimeter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a serial-controlled Arduino multimeter that measures voltage with automatic 5V/1.1V reference switching and capacitance with automatic 10k/1k range selection.

**Architecture:** Keep the first version in one sketch file so the wiring, serial interface, and measurement logic stay easy to reason about. The sketch will centralize hardware constants, provide a tiny serial command parser, and isolate voltage and capacitance measurement into separate helpers that share the same analog input.

**Tech Stack:** Arduino C++, `arduino-cli`, AVR `analogReference()`, `analogRead()`, `micros()`, `Serial`

---

### Task 1: Replace the empty sketch with shared constants and command handling

**Files:**
- Modify: `arduino-multimeter/arduino-multimeter.ino`

- [ ] **Step 1: Add the failing skeleton**

Create named constants for the Nano/Uno wiring from the source sketches, plus the serial baud rate and the voltage/capacitance thresholds. Add `setup()` and `loop()` scaffolding that prints help and waits for commands, but leave the measurement helpers as forward declarations so the file compiles only after the later tasks add them.

- [ ] **Step 2: Run a compile check to confirm the sketch still fails for missing helpers**

Run: `arduino-cli compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all`
Expected: FAIL because the measurement helpers are not implemented yet.

- [ ] **Step 3: Implement the minimal command parser**

Add `v`, `c`, and `h` command handling with a short help banner and clear `ERROR:` output for unknown commands. Keep the parser small and single-character based so serial use stays simple.

- [ ] **Step 4: Re-run the compile check**

Run: `arduino-cli compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all`
Expected: PASS once the file contains the full skeleton and parser.

- [ ] **Step 5: Commit the scaffold**

```bash
git add arduino-multimeter/arduino-multimeter.ino
git commit -m "feat: scaffold arduino multimeter sketch"
```

### Task 2: Implement automatic voltage reference switching

**Files:**
- Modify: `arduino-multimeter/arduino-multimeter.ino`

- [ ] **Step 1: Write a voltage test harness in the sketch**

Add a `measureVoltage()` helper that reads the divider using the default 5V reference first, calculates the estimated input voltage, and decides whether to retry on the 1.1V reference. Use the existing divider ratio and calibration constant from `SENSORS/voltage-meter-high-precision/voltage-meter-high-precision.ino`.

- [ ] **Step 2: Make the voltage helper fail-safe**

After every `analogReference()` change, discard the first ADC read and wait for settling before using the second read. Return the selected reference and the computed voltage so the serial layer can print an explicit status line such as `VOLTAGE: ...`.

- [ ] **Step 3: Verify the voltage code compiles**

Run: `arduino-cli compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all`
Expected: PASS.

- [ ] **Step 4: Commit the voltage mode**

```bash
git add arduino-multimeter/arduino-multimeter.ino
git commit -m "feat: add automatic voltage reference switching"
```

### Task 3: Implement capacitance auto-ranging and discharge handling

**Files:**
- Modify: `arduino-multimeter/arduino-multimeter.ino`

- [ ] **Step 1: Port the dual-resistor capacitance flow**

Add `measureCapacitance()` using the existing 10k-first timeout logic from `SENSORS/test-capacitor/test-capacitor.ino`, then fall back to the 1k path if the 10k path times out. Preserve the original wiring constants and timing threshold, but express them as named constants.

- [ ] **Step 2: Add a shared discharge helper**

Create `dischargeCapacitor()` so the timeout recovery and the post-measurement cleanup use the same logic. Have it actively verify the discharge state before returning an error when the capacitor does not settle within the spec timeout.

- [ ] **Step 3: Make the capacitance output serial-friendly**

Format the result as `CAPACITANCE: ... nF` or `CAPACITANCE: ... uF`, and print `ERROR:` for timeout or discharge failures. Keep the ADC reference explicitly at `DEFAULT` for this mode.

- [ ] **Step 4: Re-run the compile check**

Run: `arduino-cli compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all`
Expected: PASS.

- [ ] **Step 5: Commit the capacitance mode**

```bash
git add arduino-multimeter/arduino-multimeter.ino
git commit -m "feat: add capacitance auto-ranging"
```

### Task 4: Verify the finished sketch and serial contract

**Files:**
- Modify: `arduino-multimeter/arduino-multimeter.ino`

- [ ] **Step 1: Add final serial output polish**

Confirm the sketch prints a stable help message, success lines, and error lines so serial output is predictable during manual testing.

- [ ] **Step 2: Compile the final sketch**

Run: `arduino-cli compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries D:\GITHUB\Arduino\arduino-multimeter --warnings all`
Expected: PASS.

- [ ] **Step 3: Record the final state**

If the sketch builds cleanly, leave the code in its final committed state and report the command set (`v`, `c`, `h`) and the automatic range/reference behavior in the completion summary.
