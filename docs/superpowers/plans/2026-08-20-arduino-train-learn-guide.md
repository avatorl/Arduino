# Arduino Train Beginner Learning Guide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create `arduino-train-v2\docs-internal\LEARN.md`, a concise project-linked beginner reference for the Arduino and C++ concepts used by Arduino Train v2.

**Architecture:** Add one standalone Markdown document under the train's internal documentation directory. Organize it from project orientation through memory, C++, Arduino hardware, timing, preprocessor usage, embedded patterns, and troubleshooting; use small examples and source-file pointers rather than duplicating implementation. Keep the document approximately 1,500-2,500 words and readable in about 10 minutes.

**Tech Stack:** Markdown, Arduino Nano ATmega328P/AVR concepts, C++11-compatible Arduino sketch code, existing project source and README.

---

## File map

- Create: `arduino-train-v2\docs-internal\LEARN.md` — the beginner learning guide.
- Reference: `arduino-train-v2\README.md` — project features, sketch layout, build commands, hardware overview.
- Reference: `arduino-train-v2\arduino-train-v2.ino` — shared types, pin map, I2C helpers, preprocessor/debug setup, `setup()`/`loop()`.
- Reference: `arduino-train-v2\config.h` — compile-time flags, `constexpr`, timer selection, pins, `static_assert`.
- Reference: `arduino-train-v2\10-ir-remote.ino` — NEC decoding, `switch`, repeat flags, `F()`, command routing.
- Reference: `arduino-train-v2\20-motor.ino` — PWM, direction safety, `millis()` timing, integer scaling.
- Reference: `arduino-train-v2\30-lights-and-sounds.ino` — `PROGMEM`, `tone()`, non-blocking patterns, I2C-backed outputs.
- Reference: `arduino-train-v2\40-sensors.ino` — I2C register access, address reassignment, bit operations, sensor state.
- Reference: `arduino-train-v2\50-power-management.ino` — ADC readings, EEPROM, sleep, watchdog, persistent ring buffers.
- Reference: `arduino-train-v2\melodies.h` — flash-resident melody arrays.
- Reference: `arduino-train-v2\test\native\train_logic_test.cpp` — host-side pure C++ logic tests and portable helper patterns.

### Task 1: Inventory the concepts and project facts

- [ ] **Step 1: Build a source-backed checklist**

  Inspect the files in the file map and record only concepts demonstrably used by the current sketch: memory areas, types, macros, timers, pins, I2C addresses, EEPROM behavior, timing patterns, and safety mechanisms.

- [ ] **Step 2: Verify hardware facts before writing**

  Confirm the guide's project-specific values from source comments and constants:
  - D5/D6 motor PWM;
  - D9/D10 Timer1 limitation from IRremote;
  - D3/D11 Timer2/tone interaction;
  - D2 IR receiver/wake input;
  - A4/A5 shared I2C;
  - TCS34725 and VL53L0X initially at `0x29`;
  - VL53L0X XSHUT sequencing and reassigned `0x2A`;
  - EEPROM persistence and bounded write policy.

- [ ] **Step 3: Carry the verified checklist into the writing task**

  No source changes or commit are required for this inventory. Use the checklist directly while writing `LEARN.md`; do not add a separate planning artifact to the repository.

### Task 2: Write the beginner guide

**Files:**
- Create: `arduino-train-v2\docs-internal\LEARN.md`

- [ ] **Step 1: Add orientation and reading order**

  Explain that Arduino `.ino` tabs form one sketch and provide a small module map pointing readers to the relevant source files.

- [ ] **Step 2: Add memory guidance**

  Explain Nano flash versus SRAM, why this project uses `PROGMEM` for melody and register tables, how `pgm_read_byte()`/`pgm_read_word()` retrieve values, when `F("...")` is appropriate for `Serial`, and why `const` data cannot be edited in flash. Explain EEPROM as persistent but small and wear-limited, including `EEPROM.update()` and the project's write budget.

- [ ] **Step 3: Add the C++ essentials used here**

  Give short examples and beginner explanations for declarations versus definitions, definitions and scope, `bool`, signed/unsigned integers, `uint8_t`/`uint16_t`/`uint32_t`, `unsigned long`, `const`, pointers, references, arrays, functions, `struct`, `enum class`, plain enums, `constexpr`, templates, `nullptr`, casts, and `static_assert`. Include the practical reason for avoiding overflow in arithmetic and using wider intermediates.

- [ ] **Step 4: Add Arduino I/O and hardware constraints**

  Explain `pinMode`, `digitalRead`, `digitalWrite`, `analogRead`, and `analogWrite`/PWM. Include a compact project pin/timer/I2C table covering D2 IR/wake, D3 VL53L0X XSHUT, D4 color-sensor LED control, D5/D6 motor PWM, D7 motor sleep, D8 active-low motor fault, D9 active-low tilt input, D12 buzzer, A0 battery sense, and A4/A5 I2C. Clearly state that PWM is not a true analog voltage. Explain active-low signals and `INPUT_PULLUP`.

- [ ] **Step 5: Add timer and PWM conflict guidance**

  Describe the project’s deliberate timer arrangement: IRremote uses Timer1, `tone()` uses Timer2, motor PWM is on D5/D6, D9/D10 must not be used for PWM, and Timer2 activity can affect PWM behavior on D3/D11. Include a “check timer ownership before moving a pin or library” rule.

- [ ] **Step 6: Add I2C and lightweight driver guidance**

  Explain SDA/SCL, 7-bit addresses, `beginTransmission`/`write`/`endTransmission`, `requestFrom`/`read`, shared buses, and error returns. Use the actual MCP23008/TCS34725/VL53L0X arrangement, including the two initial `0x29` devices, XSHUT sequencing, and moving the VL53L0X to `0x2A`.

- [ ] **Step 7: Add timing and state-machine guidance**

  Explain `setup()` versus `loop()`, why `delay()` is avoided for normal behavior, how `millis()`-based elapsed-time checks work, and why subtraction-based comparisons survive rollover. Use the motor reverse cooldown, buzzer pattern player, sleep, sensor polling, and boost cooldown as examples.

- [ ] **Step 8: Add preprocessor and embedded patterns**

  Explain `#include`, `#define`, `#if`, `#ifdef`/`defined`, `#ifndef`, `#pragma once`, feature switches, and conditional debug macros. Contrast macros with typed `constexpr`. Briefly explain bit masks/shifts, lookup tables/interpolation, ring buffers, and safety latches/error returns.

- [ ] **Step 9: Add common mistakes and source map**

  Include practical warnings: reading PROGMEM as RAM, placing PWM on a timer-owned pin, blocking the loop with long delays, using `String` casually on a small AVR, using narrow arithmetic for products, forgetting active-low polarity, and assuming every I2C failure is a software bug. End with a recommended reading order and source links.

- [ ] **Step 10: Keep the document within scope**

  Edit for approximately 1,500-2,500 words. Remove generic Arduino material not needed to understand this project and avoid long sensor-register dumps.

- [ ] **Step 11: Commit the guide**

  ```powershell
  git add -- arduino-train-v2/docs-internal/LEARN.md
  git commit -m "Add Arduino train beginner learning guide

  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
  ```

### Task 3: Review and validate the documentation

**Files:**
- Review: `arduino-train-v2\docs-internal\LEARN.md`
- Compare: all source files listed in the file map

- [ ] **Step 1: Check every project-specific claim**

  Re-read the relevant constants and comments to ensure pin numbers, timer ownership, memory sizes, addresses, and behavior match the current source.

- [ ] **Step 2: Check beginner usability**

  Confirm every specialized term is defined before use, examples are short, headings are scannable, and the document explains when and why—not only what—a feature does.

- [ ] **Step 3: Check Markdown and scope**

  Verify links and code fences, confirm the document is in `docs-internal`, and ensure the word count is within the target range.

  ```powershell
  $p = "D:\GITHUB\Arduino\arduino-train-v2\docs-internal\LEARN.md"
  (Get-Content $p -Raw | Measure-Object -Word).Words
  ```

  Expected: a count between 1,500 and 2,500 words.

- [ ] **Step 4: Run the existing documentation-adjacent checks**

  No firmware test is required for a documentation-only change. If the guide exposes an inconsistency in source comments or constants, correct the guide to current behavior and report the inconsistency rather than changing firmware as part of this task.

- [ ] **Step 5: Commit any review corrections**

  ```powershell
  git add -- arduino-train-v2/docs-internal/LEARN.md
  git commit -m "Refine Arduino train learning guide

  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
  ```
