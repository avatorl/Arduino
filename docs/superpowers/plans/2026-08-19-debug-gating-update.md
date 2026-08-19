# Arduino Train Debug Gating Update Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `DEBUG` the master gate for every train debug feature so `DEBUG = 0` disables all debug output and `DEBUG = 1` only enables categories that are explicitly turned on.

**Architecture:** Keep the change local to `arduino-train-v2\arduino-train-v2.ino`. Update the compile-time macro block so every debug category is guarded by `DEBUG && DEBUG_<CATEGORY>`, default the category flags to `0`, and make `DEBUG_ANY` reflect the same master-gate rule. Then refresh the nearby in-source comment so the code itself explains the new behavior. Validate with compile-time checks on both supported boards and with representative `DEBUG` / per-category combinations.

**Tech Stack:** Arduino C++, Arduino CLI, AVR Nano / Uno targets.

---

## File Structure

| File | Responsibility |
|---|---|
| `arduino-train-v2\arduino-train-v2.ino` | Debug macro defaults, master-gate logic, and the comment block that explains how debug flags work. |

### Task 1: Switch the sketch to master-gated debug macros

**Files:**
- Modify: `arduino-train-v2\arduino-train-v2.ino:158-252`
- Modify: `arduino-train-v2\arduino-train-v2.ino` all remaining `#if DEBUG || DEBUG_<CATEGORY>` sites
- Test: `arduino-cli compile --fqbn arduino:avr:uno arduino-train-v2`
- Test: `arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2`

- [ ] **Step 1: Update the debug defaults and gate expressions**

Change the debug macro block so:

- `DEBUG` stays the top-level switch
- every `DEBUG_*` category defaults to `0`
- every category macro uses `#if DEBUG && DEBUG_<CATEGORY>`
- `DEBUG_ANY` only becomes true when `DEBUG` is enabled and at least one category is enabled

Then replace every remaining `#if DEBUG || DEBUG_<CATEGORY>` check in the sketch body with `#if DEBUG && DEBUG_<CATEGORY>` so the whole file follows the same master-gate rule.

Keep the macro names and the rest of the sketch structure unchanged.

- [ ] **Step 2: Update the in-source comment to match the new rule**

Rewrite the comment above the debug macros so it clearly says:

- `DEBUG = 0` disables all debug output
- `DEBUG = 1` only allows debug output for categories you explicitly enable

Avoid adding new categories or changing runtime behavior.

- [ ] **Step 3: Compile the sketch for both supported boards**

Run:

```powershell
arduino-cli compile --fqbn arduino:avr:uno arduino-train-v2
arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2
```

Expected: both builds succeed with the updated debug macro gating.

- [ ] **Step 4: Compile the explicit debug-matrix cases**

Run:

```powershell
arduino-cli compile --fqbn arduino:avr:nano --build-property 'compiler.cpp.extra_flags=-DDEBUG=0 -DDEBUG_COLOR_SENSOR=1 -DDEBUG_DISTANCE_SENSOR=1 -DDEBUG_IR_REMOTE=1 -DDEBUG_TILT_SENSOR=1 -DDEBUG_VOLTAGE_METER=1 -DDEBUG_MOTOR=1 -DDEBUG_LEDS=1 -DDEBUG_SOUND=1 -DDEBUG_REMOTE=1' arduino-train-v2
arduino-cli compile --fqbn arduino:avr:nano --build-property 'compiler.cpp.extra_flags=-DDEBUG=1 -DDEBUG_COLOR_SENSOR=0 -DDEBUG_DISTANCE_SENSOR=0 -DDEBUG_IR_REMOTE=1 -DDEBUG_TILT_SENSOR=0 -DDEBUG_VOLTAGE_METER=0 -DDEBUG_MOTOR=0 -DDEBUG_LEDS=0 -DDEBUG_SOUND=0 -DDEBUG_REMOTE=0' arduino-train-v2
arduino-cli compile --fqbn arduino:avr:nano --build-property 'compiler.cpp.extra_flags=-DDEBUG=1 -DDEBUG_COLOR_SENSOR=0 -DDEBUG_DISTANCE_SENSOR=0 -DDEBUG_IR_REMOTE=0 -DDEBUG_TILT_SENSOR=0 -DDEBUG_VOLTAGE_METER=0 -DDEBUG_MOTOR=0 -DDEBUG_LEDS=0 -DDEBUG_SOUND=0 -DDEBUG_REMOTE=0' arduino-train-v2
```

Expected: the all-enabled `DEBUG=0` build produces no active debug output, the single-category `DEBUG=1` build compiles, and the all-disabled `DEBUG=1` build still compiles cleanly.

- [ ] **Step 5: Spot-check the macro block statically**

Run:

```powershell
rg -n "#if DEBUG \|\||#define DEBUG_ANY|#define DEBUG_|DEBUG_OTHER_SENSORS" arduino-train-v2\arduino-train-v2.ino
```

Expected: the entire sketch shows the new `DEBUG && DEBUG_*` pattern, `DEBUG_ANY` reflects the master gate, and there is no legacy `DEBUG_OTHER_SENSORS` usage.

- [ ] **Step 6: Commit the debug-gating update**

```powershell
git add -- arduino-train-v2\arduino-train-v2.ino
git commit -m "Make debug flags master-gated"
```
