# Train Cluster Calibration Synchronization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate a train-compatible cluster calibration block from the existing
Python analysis tool and insert the current generated block into the train
sketch without changing calibration captures or train marker behavior.

**Architecture:** Keep the existing generic `analyze-clusters` output unchanged
for the standalone recognition sketch. Add an explicit train-target formatter
that maps supported capture labels to existing `TrackMarkerClass` constants and
emits the train's generated declaration block. The train continues to own
classification and marker-action logic; only its generated constants and
cluster table are replaced.

**Tech Stack:** Python 3 standard library, Arduino C++ for ATmega328P,
`arduino-cli`.

---

### Task 1: Add a train-target cluster formatter

**Files:**
- Modify: `rgb-color-sensor/calibration_tool.py:400-450`

- [ ] **Step 1: Add a label-map assertion command**

Run:

```powershell
python -c "from pathlib import Path; import sys; sys.path.insert(0, 'rgb-color-sensor'); import calibration_tool as tool; rows = tool.load_captures(Path('rgb-color-sensor/data')); output = tool.build_train_cluster_calibration_block(rows); assert 'MarkerWhite' in output and 'MarkerYellow' in output; assert 'const MarkerClusterDefinition markerClusters[]' in output"
```

Expected: FAIL because `build_train_cluster_calibration_block` does not exist.

- [ ] **Step 2: Implement the fixed train label mapping**

Add a module-level mapping from canonical capture-label keys to the
existing train enum constants:

```python
TRAIN_MARKER_CLASS_BY_LABEL = {
    "white": "MarkerWhite",
    "brown": "MarkerBrown",
    "cyan": "MarkerCyan",
    "green": "MarkerGreen",
    "grey": "MarkerGrey",
    "magenta": "MarkerMagenta",
    "orange": "MarkerOrange",
    "red": "MarkerRed",
    "yellow": "MarkerYellow",
}
```

Add `build_train_cluster_calibration_block(rows_by_label)`. Reuse
`ensure_required_labels`, `compute_white_balance_gains`, `enrich_rows`,
`ordered_profile_labels`, `derive_thresholds`, and `build_clusters_for_label`;
do not duplicate clustering logic. Emit the five generated threshold/gain
declarations, `MarkerClusterDefinition markerClusters[]`, and
`markerClusterCount`. Look up each profile's canonical `label` field in the
mapping and raise `SystemExit` with the unsupported display label when it is
absent.

- [ ] **Step 3: Add an explicit CLI target**

Extend the `analyze-clusters` parser with a `--target` option whose default is
the existing generic output. For `--target train`, call the new formatter;
reject any other target through `argparse` choices. Keep existing
`analyze-clusters` output byte-for-byte unchanged when no target is supplied.

- [ ] **Step 4: Run formatter verification**

Run:

```powershell
python -c "from pathlib import Path; import sys; sys.path.insert(0, 'rgb-color-sensor'); import calibration_tool as tool; rows = tool.load_captures(Path('rgb-color-sensor/data')); output = tool.build_train_cluster_calibration_block(rows); assert output.count('Marker') == 20; assert 'MarkerUnknown' not in output; assert 'colorClearMinThreshold = 160' in output; assert 'colorMatchClearThreshold = 304' in output"
python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --target train | Select-String 'MarkerWhite|MarkerYellow|markerClusterCount'
$expected = Get-Content rgb-color-sensor\data\generated-clusters.txt; $actual = python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data; $diff = Compare-Object -ReferenceObject $expected -DifferenceObject $actual; if ($diff) { $diff | Format-Table -AutoSize; throw 'Generic cluster output changed.' }
```

Expected: formatter assertions pass, the train target reports marker entries,
and generic output has no differences from `generated-clusters.txt`.

- [ ] **Step 5: Commit**

```powershell
git add rgb-color-sensor\calibration_tool.py
git commit -m "feat: add train cluster calibration export"
```

### Task 2: Insert the train export and document regeneration

**Files:**
- Modify: `arduino-train-v2/arduino-train-v2.ino:1115-1148`
- Modify: `arduino-train-v2/README.md`

- [ ] **Step 1: Generate the current train-target block**

Run:

```powershell
python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --target train --output arduino-train-v2\train-clusters.generated.tmp
```

Expected: a valid Arduino declaration block containing the 19 current marker
clusters is written as the named temporary repository artifact.

- [ ] **Step 2: Replace only the generated train region**

In `arduino-train-v2.ino`, replace the declarations between the existing
generated-data delimiters with the generated file contents. Keep the
`MarkerClusterDefinition` type, `TrackMarkerClass` enum, classifier,
`trackMarkerLabel`, and `handleTrackMarkerAction` unchanged. Update the source
comment to give the exact command:

```powershell
python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --target train
```

- [ ] **Step 3: Document the regeneration contract**

Add a concise Color Marker Calibration section to
`arduino-train-v2/README.md`. State that the train block is generated from the
shared capture dataset, give the exact `--target train` command, identify the
generated region, and explain that a newly captured unsupported label produces
a generator error until a train action/enum mapping is deliberately added.

- [ ] **Step 4: Verify exact output and compile**

Run:

```powershell
$trainBlock = 'arduino-train-v2\train-clusters.generated.tmp'
python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --target train --output $trainBlock
$sketch = Get-Content arduino-train-v2\arduino-train-v2.ino -Raw
$generated = Get-Content $trainBlock -Raw
if (-not $sketch.Contains($generated.Trim())) { throw 'Train generated region does not match train-target output.' }
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 arduino-train-v2
```

Expected: exact generated-region match and a successful Nano compile.

- [ ] **Step 5: Clean temporary generated data**

Run:

```powershell
Remove-Item arduino-train-v2\train-clusters.generated.tmp
```

Expected: only the named session artifact is removed.

- [ ] **Step 6: Commit**

```powershell
git add arduino-train-v2\arduino-train-v2.ino arduino-train-v2\README.md
git commit -m "feat: sync train cluster calibration"
```
