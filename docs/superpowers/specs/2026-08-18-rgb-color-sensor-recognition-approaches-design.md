# RGB Color Sensor Recognition Approaches Design

## Goal

Evolve the current RGB color sensor workflow from a single-prototype matcher to
three recognition-only sketch variants fed by the existing captured datasets:

1. **Approach A** — generated ML model sketch (document now, do not implement now)
2. **Approach B** — multi-cluster profile sketch (implement now)
3. **Approach C** — simple baseline prototype sketch (document now, do not implement now)

The current calibration/capture workflow remains the data-collection source.
The new recognition sketches are not responsible for calibration commands.

## Scope

- Keep the current `rgb-color-sensor.ino` focused on capture/calibration.
- Add three new recognition-only `.ino` files under `rgb-color-sensor`.
- Extend the Python calibration tool so it can generate recognition data for
  the new Approach B sketch from saved CSV captures in this phase.
- Implement only Approach B in this phase.
- Document Approaches A and C in the repository but do not complete their
  runtime inference implementations in this phase.
- Do not create Git commits; changes remain local only.

## File Responsibilities

- `rgb-color-sensor/rgb-color-sensor.ino`
  - remains the calibration/capture sketch
- `rgb-color-sensor/rgb-color-sensor-recognition-clusters.ino`
  - recognition-only sketch using generated multi-cluster profile data
- `rgb-color-sensor/rgb-color-sensor-recognition-ml.ino`
  - documented future generated-model path only
- `rgb-color-sensor/rgb-color-sensor-recognition-baseline.ino`
  - documented future simple-baseline path only
- `rgb-color-sensor/calibration_tool.py`
  - imports saved logs, captures Serial data, keeps the existing baseline
    analysis output, and adds the generated multi-cluster data block for
    Approach B
- `rgb-color-sensor/REFERENCES.md`
  - explains the three approaches and the generation/paste workflow

These filenames are fixed and should remain stable so the user can compile and
test them separately.

## Approach A — Generated ML Model

- Use Python and scikit-learn to train a classifier from the captured datasets.
- Export inference data or generated inference code into a sketch-ready asset.
- The recognition-only sketch for this approach will eventually include that
  generated model and call it for inference.
- In this phase, only document the path and create any minimal placeholder
  needed to reserve the file location and explain intended use. That
  placeholder must be a compilable stub sketch with `setup()` and `loop()`
  plus a clear top-of-file note that the ML path is not implemented yet.

## Approach B — Multi-Cluster Profile Recognition

- Use the captured CSV datasets to build a richer color-space description than
  a single center per color.
- For each recognized color, derive multiple clusters in balanced/normalized
  RGB space.
- Use a deterministic k-means-style clustering pass per color on normalized
  balanced RGB values only.
- Try cluster counts from 1 to 3 for each color and choose the smallest count
  that reduces the mean within-cluster distance by at least 20% over the next
  simpler option. If no such improvement exists, keep the simpler option.
- Use Manhattan / L1 distance for cluster assignment, within-cluster scoring,
  cluster-radius calculation, and runtime matching so the logic stays aligned
  with the current sketch's distance style.
- Each generated cluster record must include:
  - owning color label or color index
  - cluster center
  - acceptance radius
- Compute each cluster radius as the 95th percentile of training-sample
  distances from that cluster center, plus a fixed safety margin of 0.
- Do not generate extra per-cluster guards in this phase; keep low-light and
  empty-sample rejection as global threshold logic.
- The Approach B sketch must:
  1. read raw sensor values
  2. apply white balance
  3. normalize RGB
  4. reject dark/empty samples using generated threshold guidance
  5. compare the sample against all generated clusters
  6. choose the nearest valid cluster within its allowed threshold
  7. return the owning color label for the winning cluster

This approach models a color as multiple allowed regions in RGB space instead
of forcing each color into a single global center.

## Approach C — Simple Baseline Recognition

- Keep the current simple prototype-center concept as a baseline comparator.
- The recognition-only sketch for this approach will eventually contain the
  simpler center-based matcher with generated thresholds.
- In this phase, only document the path and create any minimal placeholder
  needed to reserve the file location and explain intended use. That
  placeholder must be a compilable stub sketch with `setup()` and `loop()`
  plus a clear top-of-file note that the baseline path is not implemented yet.

## Generated Output for Approach B

The Python tool must emit a copy-paste Arduino/C++ block for the Approach B
sketch instead of a black-box binary artifact.

That generated block must include:

- white-balance gains
- clear / neutral / empty detection thresholds
- color label table used by the recognition-only sketch
- cluster table for all colors, with stable ordering
- any constants the cluster-matching logic needs

The generated block must be deterministic for the same input CSV data.

The current `analyze` command must remain available for the existing simple
prototype output. Add a separate Approach B command, such as
`analyze-clusters`, instead of silently replacing the current `analyze`
behavior.

Approaches A and C do not require new Python generation logic in this phase
beyond preserving the current baseline `analyze` behavior and documenting the
future ML path.

## Data Flow

1. User records captures with the existing calibration/capture sketch.
2. Python tool imports or captures Serial output into CSV files.
3. Python tool analyzes the CSV files and emits the Approach B generated block.
4. User pastes that generated block into the Approach B recognition-only sketch.
5. User compiles and manually tests the recognition-only sketch on hardware.

## Error Handling

- If required calibration labels are missing, the Python tool must fail with a
  clear error.
- If the dataset is insufficient to generate valid clusters for a requested
  color, the Python tool must fail clearly rather than silently guessing.
- The Approach B sketch must reject unmatched samples instead of forcing a
  color decision.

## Validation

- `python -m py_compile rgb-color-sensor/calibration_tool.py`
- run the Python tool against the existing captured dataset and confirm it
  emits the generated Approach B data block
- compile the new Approach B recognition-only sketch for Arduino Uno
- ensure the existing calibration/capture sketch still remains the place where
  raw capture is performed
