# RGB Color Sensor Calibration Capture Design

## Goal

Add a repeatable raw-data calibration workflow for `rgb-color-sensor` that:

1. Keeps the current guided `cal` calibration mode.
2. Adds a timed capture command that requires a color name, for example
   `cal "Maroon Red" 15`.
3. Streams all captured raw sensor readings to Serial during the capture and
   prints a complete summary when the timer ends.
4. Supports saving each color's readings to files on a connected computer.
5. Produces a copy-paste calibration block for the main `.ino` that improves
   future color recognition without requiring a black-box embedded model.

## Command Behavior

- Preserve the existing plain `cal` command as the current guided,
  multi-sample calibration mode.
- Add a separate timed capture form: `cal "<color name>" <seconds>`.
- The color name is required and may contain spaces inside quotes. Support up
  to 48 visible label characters.
- Preserve the color label text exactly as entered for Serial output, CSV
  content, and generated calibration output.
- The duration is required, expressed in whole seconds.
- Reject zero or negative durations. Cap accepted durations at a practical
  upper bound of 300 seconds so timed capture cannot run indefinitely from
  malformed input.
- `stop` must cancel either the guided calibration flow or an active timed
  capture.
- While a timed capture is active, normal live detection pauses until the
  capture completes or is cancelled.
- If guided calibration or timed capture is already active, reject any new
  `cal` command with an error message. Only `stop` may interrupt an active
  capture mode.
- The current serial input path must be updated so timed-capture commands are
  not lowercased, truncated, or parsed only by whole-line `strcmp`.
- Grow the serial command buffer enough to support quoted labels plus the
  duration argument for typical human-readable color names. A 96-byte command
  buffer is sufficient for this feature.

## Sketch Responsibilities

- Parse the new quoted command format without breaking the current command set.
- During a timed capture, sample the TCS34725 using the existing raw read path
  and a fixed 100 ms cadence, matching the current guided-calibration sampling
  density instead of the slower live-detection loop cadence.
- Emit a machine-readable Serial protocol with:
  - a capture-begin line containing the requested color label and duration
  - one sample line per reading
  - a capture-end line containing count and timing
  - a final human-readable summary block
- Each sample line must contain enough information for later training:
  timestamp offset, label, raw `r,g,b,c`, and the currently derived balanced
  and normalized values when they can be computed from the sketch's current
  calibration constants.
- Keep memory usage bounded by streaming records immediately instead of trying
  to store all samples in SRAM.

### Timed Capture Serial Protocol

Use CSV with a fixed record type in column 1 and quote the label field so the
Python parser can rely on one stable contract:

- `CAPTURE_BEGIN,"<label>",<duration_seconds>`
- `CAPTURE_SAMPLE,<elapsed_ms>,"<label>",<raw_r>,<raw_g>,<raw_b>,<raw_c>,<balanced_r>,<balanced_g>,<balanced_b>,<normalized_r>,<normalized_g>,<normalized_b>`
- `CAPTURE_END,"<label>",<sample_count>,<elapsed_ms>`

The sketch must escape embedded double quotes in labels consistently before
printing them. The Python tool must parse this exact protocol instead of
guessing field boundaries from human-readable text. Use standard CSV escaping:
double any embedded `"` character inside the quoted label field.
If balanced or normalized values cannot be computed from the current sketch
state, the sketch must still emit the fixed columns and use `-1` as the
placeholder value for each unavailable derived field.

## Host-Side Python Tool

- Add a Python script under `rgb-color-sensor` that connects to the Arduino
  over Serial.
- The script listens for the machine-readable timed-capture protocol and saves:
  - one CSV file per color label
  - one combined CSV file containing all captures
- Sanitize file names derived from labels while preserving the original label
  inside the CSV content. Convert file names to lowercase, replace each run of
  non-alphanumeric characters with `_`, and trim leading or trailing `_`.
- The tool should support common calibration labels such as `white`, `black`,
  `nothing`, and named colors like `Maroon Red`.
- Timed-capture labels are an open label space: they do not need to match the
  guided `calibrationSamples[]` array in advance.
- Use the same CSV header for per-color and combined exports:
  `capture_label,elapsed_ms,raw_r,raw_g,raw_b,raw_c,balanced_r,balanced_g,balanced_b,normalized_r,normalized_g,normalized_b`

## Derived Calibration Output

- The Python tool analyzes the captured CSV files and emits a copy-paste
  calibration block for the sketch instead of a generated `model.h`.
- The generated block should stay transparent and editable in the main sketch.
- The block must include:
  - per-class profile rows for the recognized labels
  - updated white-balance guidance derived from the `white` samples
  - clear-threshold guidance derived from `black` and `nothing`
  - distance or match-threshold guidance derived from class separation
- The output format should be valid Arduino/C++ so it can be pasted directly
  near the current calibration constants and sample definitions.

## Recognition Strategy

- Base recognition improvements on real captured data rather than hand-picked
  prototypes.
- Use raw `r,g,b,c` plus derived balanced and normalized RGB features.
- Treat `white`, `black`, and `nothing` as explicit calibration classes because
  they materially affect thresholding and neutral/empty detection.
- Prefer a generated profile matrix and thresholds over immediate scikit-learn
  model export for the first implementation, because it is easier to inspect,
  debug, and maintain inside the current sketch.
- Structure the Python analysis so future comparison against a scikit-learn
  classifier remains possible from the same CSV datasets.

## Files

- Update `rgb-color-sensor/rgb-color-sensor.ino`
- Add a Python calibration tool under `rgb-color-sensor`
- Add or update focused documentation in `rgb-color-sensor` describing:
  - the timed capture command
  - how to run the Python tool
  - how to collect datasets for `white`, `black`, `nothing`, and target colors
  - how to paste the generated calibration block back into the sketch

## Validation

- Verify the sketch still supports the existing guided `cal` flow.
- Verify `cal "<name>" <seconds>` starts a timed capture, streams records, and
  stops automatically after the requested duration.
- Verify `stop` cancels an active timed capture cleanly.
- Verify the Python tool writes per-color CSV files and a combined export.
- Verify the generated calibration block is copy-paste ready for the main sketch.
