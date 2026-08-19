# Arduino Train Debug Split Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split the train's legacy other-sensor debug category into independent IR remote, tilt, and voltage-meter categories and make IR diagnostics describe every button press and its processed action.

**Architecture:** Keep compile-time debug routing in the sketch's existing macro block. Add a small IR command-to-label helper beside the remote code declarations; decode-time logging will use it for every new NEC command, while branch-specific action text remains in the `translateIR()` switch that owns the behavior. No control-flow or remote repeat semantics change.

**Tech Stack:** Arduino C++, Arduino Nano AVR (`arduino:avr:nano`), IRremote, Arduino CLI.

---

## File Structure

| File | Responsibility |
|---|---|
| `arduino-train-v2/arduino-train-v2.ino` | Debug flags/macros, remote label lookup and action logs, plus reassignment of tilt, voltage, idle, and sleep diagnostics. |
| `docs/superpowers/specs/2026-08-18-arduino-train-debug-split-design.md` | Approved behavioral design; do not modify unless implementation exposes a design conflict. |

### Task 1: Split and Route Debug Categories

**Files:**
- Modify: `arduino-train-v2/arduino-train-v2.ino:90-160`
- Modify: `arduino-train-v2/arduino-train-v2.ino:1359-1361, 1475-1484, 1509-1532, 2149-2163, 2436-2449`
- Test: Arduino CLI compilation of `arduino-train-v2/arduino-train-v2.ino`

- [ ] **Step 1: Establish a compile baseline**

Run:

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn arduino:avr:nano --libraries .\libraries .\arduino-train-v2
```

Expected: a successful Nano build before the debug-routing change.

- [ ] **Step 2: Replace the legacy debug configuration**

In the macro block, remove `DEBUG_OTHER_SENSORS`. Define the three new flags:

```cpp
#ifndef DEBUG_IR_REMOTE
#define DEBUG_IR_REMOTE 1
#endif
#ifndef DEBUG_TILT_SENSOR
#define DEBUG_TILT_SENSOR DEBUG
#endif
#ifndef DEBUG_VOLTAGE_METER
#define DEBUG_VOLTAGE_METER DEBUG
#endif
```

Make `DEBUG_ANY` include `DEBUG` directly as well as all module flags, including the three new categories. Update every existing category gate (`DEBUG_COLOR_SENSOR`, `DEBUG_DISTANCE_SENSOR`, `DEBUG_MOTOR`, `DEBUG_LEDS`, `DEBUG_SOUND`, and `DEBUG_REMOTE`) and define the matching new `DBG_IR_REMOTE`, `DBGLN_IR_REMOTE`, `DBG_TILT_SENSOR`, `DBGLN_TILT_SENSOR`, `DBG_VOLTAGE_METER`, and `DBGLN_VOLTAGE_METER` macros with the established enabled/empty-macro pattern. Every enabled condition must be `DEBUG || DEBUG_<CATEGORY>` so `DEBUG=1` overrides an explicitly disabled individual flag and guarantees `Serial.begin()` is called.

- [ ] **Step 3: Route all former other-sensor logs**

Replace every use of `DBG_OTHER_SENSORS` and `DBGLN_OTHER_SENSORS`:

| Location | Replacement category |
|---|---|
| startup battery measurement; periodic ADC output; low-battery warning/shutdown; button 9 voltage/percent output | `DEBUG_VOLTAGE_METER` |
| tilt active/idle transitions | `DEBUG_TILT_SENSOR` |
| idle shutdown and entering-sleep messages | `DEBUG_IR_REMOTE` |

Use no legacy `DEBUG_OTHER_SENSORS` identifier after this task.

- [ ] **Step 4: Compile the three independent configurations**

Run one compile per configuration:

```powershell
$cli = 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
& $cli compile --fqbn arduino:avr:nano --libraries .\libraries --build-property 'compiler.cpp.extra_flags=-DDEBUG=0 -DDEBUG_IR_REMOTE=1 -DDEBUG_TILT_SENSOR=0 -DDEBUG_VOLTAGE_METER=0' .\arduino-train-v2
& $cli compile --fqbn arduino:avr:nano --libraries .\libraries --build-property 'compiler.cpp.extra_flags=-DDEBUG=0 -DDEBUG_IR_REMOTE=0 -DDEBUG_TILT_SENSOR=1 -DDEBUG_VOLTAGE_METER=0' .\arduino-train-v2
& $cli compile --fqbn arduino:avr:nano --libraries .\libraries --build-property 'compiler.cpp.extra_flags=-DDEBUG=0 -DDEBUG_IR_REMOTE=0 -DDEBUG_TILT_SENSOR=0 -DDEBUG_VOLTAGE_METER=1' .\arduino-train-v2
```

Expected: all three builds succeed, proving each category can be compiled independently.

- [ ] **Step 5: Compile the legacy override configuration**

Run:

```powershell
$cli = 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
& $cli compile --fqbn arduino:avr:nano --libraries .\libraries --build-property 'compiler.cpp.extra_flags=-DDEBUG=1 -DDEBUG_COLOR_SENSOR=0 -DDEBUG_DISTANCE_SENSOR=0 -DDEBUG_IR_REMOTE=0 -DDEBUG_TILT_SENSOR=0 -DDEBUG_VOLTAGE_METER=0 -DDEBUG_MOTOR=0 -DDEBUG_LEDS=0 -DDEBUG_SOUND=0 -DDEBUG_REMOTE=0' .\arduino-train-v2
```

Expected: successful build with `Serial.begin()` and every existing and new logging macro active through the global override.

- [ ] **Step 6: Commit the debug category split**

```powershell
git add -- arduino-train-v2/arduino-train-v2.ino
git commit -m "Split train sensor debug categories"
```

### Task 2: Add IR Button and Action Diagnostics

**Files:**
- Modify: `arduino-train-v2/arduino-train-v2.ino:270-305`
- Modify: `arduino-train-v2/arduino-train-v2.ino:1792-1809`
- Modify: `arduino-train-v2/arduino-train-v2.ino:1961-2200`
- Test: Arduino CLI compilation of `arduino-train-v2/arduino-train-v2.ino`

- [ ] **Step 1: Add the command-to-label helper**

Add a `const __FlashStringHelper* remoteButtonLabel(uint8_t code)` declaration with the other forward declarations and define it near the remote button constants. It must return the exact labels documented in the preceding remote-layout comment:

```cpp
case buttonCHminus: return F("CH-");
case buttonCH: return F("CH");
case buttonCHplus: return F("CH+");
case buttonBackward: return F("<<");
case buttonForward: return F(">>");
case buttonPlayPause: return F("Play/Pause");
case buttonEQ: return F("EQ");
case button0: return F("0");
case button100plus: return F("100+");
case button200plus: return F("200+");
case button1: return F("1"); // through button9
default: return F("unknown");
```

The unassigned documented `-` (`7`) and `+` (`21`) codes must be explicit helper cases, despite having no behavior in `translateIR()`.

- [ ] **Step 2: Log every newly decoded NEC press**

In `irReceive()`, after assigning a non-repeat command and before `IrReceiver.resume()`, print the decimal value and helper label through the IR remote macros:

```cpp
DBG_IR_REMOTE(F("IR button: code="));
DBG_IR_REMOTE(received);
DBG_IR_REMOTE(F(", label="));
DBGLN_IR_REMOTE(remoteButtonLabel(received));
```

Do not add this output to the repeat-frame branch. Preserve `lastWasRepeat`, `lastIRCommand`, return values, and `IrReceiver.resume()` placement.

- [ ] **Step 3: Add handler-owned action descriptions**

Add one concise `DBGLN_IR_REMOTE(F("..."))` message inside each `switch` case (or existing melody dispatch branch) that identifies the assigned action. For conditional controls, print the branch’s real outcome, such as ignored-during-jog, ignored-while-auto, successful jog start, auto enable/disable, sound toggle, horn, siren toggle, color-sensor toggle, and battery-status request.

Add messages for:

```text
CH- / CH / CH+; << / >>; 0; Play/Pause; EQ; 100+; 200+; 9; 1–8
```

Before the existing early return when a motor-control command fails `rearmMotorDriver()`, print an IR-remote action line such as `Motor command rejected: driver fault still active`; this is the handler outcome for the affected CH-/CH/CH+/jog/Play-Pause commands and does not reach the `switch`.

Do not print an action for unassigned `-` and `+`; their decode-time code/label line is the required output. Keep existing module-specific logs (motor, sound, and so on), and do not use them as substitutes for the new IR remote action line.

- [ ] **Step 4: Inspect the IR logging behavior statically**

Run:

```powershell
rg -n "remoteButtonLabel|IR button: code=|case button|tryPlayMelodyForButton|DEBUG_OTHER_SENSORS" arduino-train-v2\arduino-train-v2.ino
```

Expected: the label helper covers every documented key, decode logging appears only on the non-repeat path, each handled command has an IR-remote action output, melody handling has its own action output, and `DEBUG_OTHER_SENSORS` has no matches.

- [ ] **Step 5: Compile the final sketch**

Run:

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn arduino:avr:nano --libraries .\libraries .\arduino-train-v2
```

Expected: successful Nano build with no new warnings or errors.

- [ ] **Step 6: Commit the IR diagnostics**

```powershell
git add -- arduino-train-v2/arduino-train-v2.ino
git commit -m "Add train remote debug diagnostics"
```
