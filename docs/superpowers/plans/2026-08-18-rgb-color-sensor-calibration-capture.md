# RGB Color Sensor Calibration Capture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a named timed-capture calibration workflow for `rgb-color-sensor`, save raw calibration datasets on a host computer, and generate a copy-paste calibration block for the main sketch.

**Architecture:** Extend the existing Arduino sketch in place so it keeps the guided `cal` flow while adding a second `cal "<label>" <seconds>` path with a stable CSV-like Serial protocol. Add one focused Python tool under `rgb-color-sensor` that can both capture Serial samples into CSV files and analyze those CSV files into sketch-ready calibration constants and profile rows.

**Tech Stack:** Arduino C++, Adafruit TCS34725 library, Python 3, standard-library `csv`/`argparse`/`pathlib`, `arduino-cli`.

---

## File Structure

- Modify: `rgb-color-sensor/rgb-color-sensor.ino` — Add timed-capture parsing, active-mode state, stable Serial protocol output, and integration with the current live-detection/calibration flow.
- Create: `rgb-color-sensor\calibration_tool.py` — Capture Serial data, write per-color and combined CSV datasets, and generate the copy-paste calibration block.
- Modify: `rgb-color-sensor\REFERENCES.md` — Expand the notes into actionable calibration/training workflow guidance and point to the Python tool.
- Test: compile `rgb-color-sensor` for Arduino Uno and run `python -m py_compile rgb-color-sensor\calibration_tool.py`.

### Task 1: Refactor serial command parsing for dual calibration modes

**Files:**
- Modify: `rgb-color-sensor/rgb-color-sensor.ino: global command-buffer declarations, handleSerialCommand(), pollSerialInput()`
- Test: `arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor`

- [ ] **Step 1: Add the new command-mode constants and buffers**

Increase the serial command buffer to 96 bytes. Add explicit limits for
timed-capture labels (48 visible characters) and duration (1-300 seconds).
Add a dedicated buffer for the active capture label so the sketch preserves
the original label text after parsing.

- [ ] **Step 2: Split command routing by mode**

Keep plain `cal`, `calibrate`, and `start` mapped to the existing guided
calibration entry point. Add a second parsing path that recognizes the exact
shape `cal "<label>" <seconds>` without lowercasing the label content.

- [ ] **Step 3: Stop lowercasing quoted label content at input time**

Update `pollSerialInput()` so the command buffer no longer applies
`tolower()` to every incoming character. Store characters inside quoted label
content verbatim, while still allowing case-insensitive matching for the
leading command token and non-label control commands such as `cal`, `stop`,
and `exit`.

- [ ] **Step 4: Add busy-mode rejection rules**

If guided calibration is active, reject timed-capture commands with a clear
Serial error. If timed capture is active, reject any new `cal` command and
allow only `stop` to interrupt it.

- [ ] **Step 5: Preserve legacy command behavior**

Keep `stop` and `exit` working for guided calibration, and make them stop the
timed-capture mode too. Keep the existing unknown-command help text, but
expand it to mention the quoted timed-capture form.

- [ ] **Step 6: Compile after parser refactor**

Run: `arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor`

Expected: compilation succeeds and the sketch still builds before capture
streaming is added.

### Task 2: Add timed raw-data capture and stable Serial protocol

**Files:**
- Modify: `rgb-color-sensor/rgb-color-sensor.ino: calibration state near globals, setup(), loop(), new timed-capture helpers`
- Test: `arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor`

- [ ] **Step 1: Add timed-capture runtime state**

Add booleans and timestamps for timed capture activity, start time, end time,
sample count, and whether the mode was cancelled. Add helpers such as
`startTimedCapture()`, `stopTimedCapture()`, and `updateTimedCapture()`.

- [ ] **Step 2: Add CSV-safe label printing**

Implement a helper that prints a quoted CSV label field and doubles embedded
double quotes. Reuse this helper everywhere the machine-readable protocol
prints a label.

- [ ] **Step 3: Emit the exact capture protocol**

At capture start, print:

```text
CAPTURE_BEGIN,"Maroon Red",15
```

For each 100 ms sample, print:

```text
CAPTURE_SAMPLE,230,"Maroon Red",512,180,260,640,512,253,648,362,179,459
```

where the columns are:

```text
elapsed_ms,label,raw_r,raw_g,raw_b,raw_c,balanced_r,balanced_g,balanced_b,normalized_r,normalized_g,normalized_b
```

At capture end, print:

```text
CAPTURE_END,"Maroon Red",150,15021
```

Emit `-1` for unavailable derived values so the column count never changes.

- [ ] **Step 4: Integrate timed capture into the main loop**

Call `updateTimedCapture()` from `loop()` before normal live detection.
While timed capture is active, pause the ordinary classification output and
run only the 100 ms capture path.

- [ ] **Step 5: Print a final human-readable summary**

After `CAPTURE_END`, print a summary that includes label, requested seconds,
elapsed milliseconds, sample count, and whether the capture finished normally
or was cancelled. Keep this separate from the machine-readable lines so the
Python parser can ignore it safely.

- [ ] **Step 6: Compile after capture integration**

Run: `arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor`

Expected: compilation succeeds with the new mode, helper functions, and
protocol output.

### Task 3: Build the Python capture-and-analysis tool

**Files:**
- Create: `rgb-color-sensor\calibration_tool.py`
- Test: `python -m py_compile rgb-color-sensor\calibration_tool.py`

- [ ] **Step 1: Define the CLI surface**

Implement subcommands:

```text
python rgb-color-sensor\calibration_tool.py capture --port COM3 --baud 9600 --out rgb-color-sensor\data
python rgb-color-sensor\calibration_tool.py analyze --input rgb-color-sensor\data --output rgb-color-sensor\data\generated-calibration.txt
```

Use `argparse` and keep defaults obvious. The capture command should create
the output directory if needed.

- [ ] **Step 2: Parse only the machine-readable protocol**

Ignore all human-readable lines. Accept only `CAPTURE_BEGIN`,
`CAPTURE_SAMPLE`, and `CAPTURE_END`. Parse quoted labels with Python's CSV
reader so embedded quotes round-trip correctly.

- [ ] **Step 3: Write per-color and combined CSV files**

Create one file per sanitized label and one combined dataset file. Write this
exact header row to both formats:

```text
capture_label,elapsed_ms,raw_r,raw_g,raw_b,raw_c,balanced_r,balanced_g,balanced_b,normalized_r,normalized_g,normalized_b
```

Sanitize file names by lowercasing, replacing each run of non-alphanumeric
characters with `_`, and trimming leading or trailing `_`.

- [ ] **Step 4: Generate the calibration block**

For each label, compute profile centers from the saved rows. Derive:
- white-balance gains from the `white` class if present
- clear-threshold guidance from `black` and `nothing` if present
- per-class normalized profile rows
- match-threshold guidance from class separation in normalized space

Print an Arduino-ready block containing updated constants and sample/profile
rows that can be pasted into `rgb-color-sensor.ino`.

- [ ] **Step 5: Keep the analysis deterministic**

If the required calibration labels are missing, exit with a clear error
instead of silently guessing. Keep the output ordering stable by sorting
labels consistently before generating the block.

- [ ] **Step 6: Validate the Python script syntax**

Run: `python -m py_compile rgb-color-sensor\calibration_tool.py`

Expected: no output and exit code 0.

### Task 4: Document the capture workflow and validate the end-to-end surface

**Files:**
- Modify: `rgb-color-sensor\REFERENCES.md`
- Modify: `rgb-color-sensor/rgb-color-sensor.ino` usage text in `setup()` / command help
- Test: compile + script syntax check

- [ ] **Step 1: Document the workflow next to the references**

Expand `REFERENCES.md` so it explains:
- why the two Eloquent Arduino links are relevant
- how to run the Python capture tool
- example capture commands for `white`, `black`, `nothing`, and a named color
- how to run analysis and paste the generated calibration block back into the sketch

- [ ] **Step 2: Update on-device usage hints**

Update the sketch's startup/help text so a user sees both calibration modes:
plain guided `cal` and timed `cal "<label>" <seconds>`.

- [ ] **Step 3: Run the final validations**

Run:

```text
arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor
python -m py_compile rgb-color-sensor\calibration_tool.py
```

Expected: both commands succeed.

- [ ] **Step 4: Keep changes local**

Do not create a Git commit. The user explicitly requested local changes only.
