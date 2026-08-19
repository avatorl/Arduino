# RGB Color Sensor Recognition Approaches Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add three recognition-only RGB sensor sketches, implement the multi-cluster Approach B path end-to-end, and keep the existing capture/calibration workflow intact.

**Architecture:** Leave `rgb-color-sensor.ino` as the capture/calibration sketch and extend `calibration_tool.py` with a new `analyze-clusters` command that emits a pasteable cluster block for a separate recognition-only sketch. Add compilable placeholder sketches for the future ML and baseline approaches, and a real `rgb-color-sensor-recognition-clusters.ino` sketch that classifies samples by matching them against generated per-color cluster regions in normalized RGB space.

**Tech Stack:** Arduino C++, Python 3, standard-library `csv`/`argparse`/`pathlib`, Adafruit TCS34725 library, `arduino-cli`.

---

## File Structure

- Modify: `rgb-color-sensor/calibration_tool.py` — preserve existing `analyze`, add `analyze-clusters`, deterministic per-color clustering, and Arduino/C++ cluster-block generation.
- Create: `rgb-color-sensor/rgb-color-sensor-recognition-clusters.ino` — recognition-only sketch that consumes generated Approach B cluster data and performs live color matching.
- Create: `rgb-color-sensor/rgb-color-sensor-recognition-ml.ino` — compilable stub for the future generated-model path.
- Create: `rgb-color-sensor/rgb-color-sensor-recognition-baseline.ino` — compilable stub for the future single-prototype baseline path.
- Modify: `rgb-color-sensor/REFERENCES.md` — document the three approaches and the new `analyze-clusters` generation workflow.
- Test: `python -m py_compile rgb-color-sensor\calibration_tool.py`
- Test: `python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --output rgb-color-sensor\data\generated-clusters.txt`
- Test: `arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor\rgb-color-sensor-recognition-clusters.ino`

### Task 1: Add deterministic cluster generation to the Python tool

**Files:**
- Modify: `rgb-color-sensor/calibration_tool.py`
- Test: `python -m py_compile rgb-color-sensor\calibration_tool.py`
- Test: `python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --output rgb-color-sensor\data\generated-clusters.txt`

- [ ] **Step 1: Keep the existing baseline generator untouched**

Leave `analyze` and `build_calibration_block()` available for the current
single-prototype workflow so the baseline path still exists.

- [ ] **Step 2: Add cluster-analysis helpers**

Implement deterministic helpers for:
- Manhattan / L1 distance
- cluster-center recomputation
- deterministic center initialization
- 1-to-3 cluster candidate fitting per color
- mean within-cluster distance scoring
- 95th-percentile radius derivation with safety margin `0`

- [ ] **Step 3: Add cluster-count selection**

For each color, try `k=1`, `k=2`, and `k=3`. Keep the simpler count unless
the next count reduces the mean within-cluster L1 distance by at least 20%.

- [ ] **Step 4: Add Arduino block generation for Approach B**

Generate a copy-paste block with:
- white-balance gains
- dark/empty thresholds
- label table
- cluster table with label index, center, and radius
- cluster/label counts

- [ ] **Step 5: Add the new CLI entry point**

Add `analyze-clusters` as a separate subcommand instead of replacing
`analyze`.

- [ ] **Step 6: Add explicit cluster-path validation**

For `analyze-clusters`, reuse required-label validation and fail clearly if a
color does not have enough samples to build a valid cluster radius or cluster
set.

- [ ] **Step 7: Validate the tool**

Run:

```text
python -m py_compile rgb-color-sensor\calibration_tool.py
python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --output rgb-color-sensor\data\generated-clusters.txt
python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --output rgb-color-sensor\data\generated-clusters-2.txt
```

Expected: syntax check passes, the generated cluster file is written, and both
cluster outputs are identical for the same input data.

### Task 2: Create the Approach B recognition-only sketch

**Files:**
- Create: `rgb-color-sensor/rgb-color-sensor-recognition-clusters.ino`
- Test: `arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor\rgb-color-sensor-recognition-clusters.ino`

- [ ] **Step 1: Copy only the recognition-safe pieces from the current sketch**

Reuse the sensor setup, white-balance helpers, normalization logic, RGB LED
helpers, and two-of-three consensus idea as needed, but remove all calibration
command handling and timed-capture logic.

- [ ] **Step 2: Define the generated data contract**

Add explicit structs for:
- labels
- cluster entries
- any global thresholds the generated block supplies

Leave a clearly marked generated-data section where the user pastes the output
from `generated-clusters.txt`.

- [ ] **Step 3: Implement cluster-based classification**

For each sample:
- reject dark/empty cases using the generated thresholds
- apply white balance
- normalize RGB
- scan all generated clusters using L1 distance
- accept only clusters whose distance is within their radius
- return the owning label of the nearest valid cluster

- [ ] **Step 4: Keep manual test output simple**

Print detected labels to Serial in a stable format suitable for manual
hardware testing. Keep the sketch recognition-only.

- [ ] **Step 5: Compile the new sketch**

Run:

```text
arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor\rgb-color-sensor-recognition-clusters.ino
```

Expected: compilation succeeds for Uno.

### Task 3: Add future-approach stubs and document usage

**Files:**
- Create: `rgb-color-sensor/rgb-color-sensor-recognition-ml.ino`
- Create: `rgb-color-sensor/rgb-color-sensor-recognition-baseline.ino`
- Modify: `rgb-color-sensor/REFERENCES.md`
- Test: `arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor\rgb-color-sensor-recognition-ml.ino`
- Test: `arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor\rgb-color-sensor-recognition-baseline.ino`

- [ ] **Step 1: Create a compilable ML placeholder sketch**

Add a stub sketch with `setup()` and `loop()` that clearly states the
generated-model approach is planned but not implemented yet.

- [ ] **Step 2: Create a compilable baseline placeholder sketch**

Add a stub sketch with `setup()` and `loop()` that clearly states the
single-prototype baseline sketch is planned but not implemented yet.

- [ ] **Step 3: Document the three approaches**

Update `REFERENCES.md` to explain:
- what Approach A, B, and C mean
- that only B is implemented now
- how to run `analyze-clusters`
- where to paste the generated block

- [ ] **Step 4: Compile both stubs**

Run:

```text
arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor\rgb-color-sensor-recognition-ml.ino
arduino-cli compile --fqbn arduino:avr:uno rgb-color-sensor\rgb-color-sensor-recognition-baseline.ino
```

Expected: both placeholder sketches compile.

- [ ] **Step 5: Keep changes local**

Do not create a Git commit. The user explicitly wants local changes only.
