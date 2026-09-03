# RGB Color Sensor References

## Sensor calibration and color identification

- [Color identification on Arduino](https://eloquentarduino.github.io/2019/12/color-identification-on-arduino/)

## Classifier training

- [How to train a classifier in scikit-learn](https://eloquentarduino.github.io/2019/11/how-to-train-a-classifier-in-scikit-learn/)

## Recommended workflow for this project

The two Eloquent Arduino articles are useful here for two different reasons:

- the color-identification article shows the right data-collection pattern:
  collect many raw sensor readings per class, including an empty or "nothing"
  class
- the scikit-learn article shows how to structure one CSV file per class so the
  same dataset can later feed a simple ML pipeline if the hand-tuned profile
  approach stops being good enough

For this sketch, the current recommended path is:

1. Upload `rgb-color-sensor.ino`.
2. Run one capture command per label so the tool both sends the Serial command
   and saves the output:
   - `python rgb-color-sensor\calibration_tool.py capture --port COM3 --out rgb-color-sensor\data --command 'cal "white" 15'`
   - `python rgb-color-sensor\calibration_tool.py capture --port COM3 --out rgb-color-sensor\data --command 'cal "black" 15'`
   - `python rgb-color-sensor\calibration_tool.py capture --port COM3 --out rgb-color-sensor\data --command 'cal "nothing" 15'`
   - `python rgb-color-sensor\calibration_tool.py capture --port COM3 --out rgb-color-sensor\data --command 'cal "Maroon Red" 15'`
3. Repeat for every color you want recognized.
4. Generate a new calibration block:
   `python rgb-color-sensor\calibration_tool.py analyze --input rgb-color-sensor\data --output rgb-color-sensor\data\generated-calibration.txt`
5. Copy the generated constants and `calibrationSamples[]` block back into the
   top of `rgb-color-sensor.ino`.

Notes:

- install `pyserial` first if capture mode says it is missing:
  `pip install pyserial`
- if you already have `CAPTURE_*` lines in Serial Monitor, save or paste that
  output into a text file and import it with:
  `python rgb-color-sensor\calibration_tool.py import-log --log rgb-color-sensor\my-captures.txt --out rgb-color-sensor\data`
- `black` and `nothing` are still worth recording even though they are mostly
  used to derive low-light and empty-view thresholds
- the generated output is intentionally plain Arduino/C++, so you can inspect
  and tune it before pasting it into the sketch

## Recognition approaches

There are now three recognition-only sketch paths:

- `rgb-color-sensor-recognition-clusters\rgb-color-sensor-recognition-clusters.ino` — **Approach B**, implemented now.
  It uses multiple generated clusters per color, so one color can occupy
  several regions in RGB space instead of just one center.
- `rgb-color-sensor-recognition-ml\rgb-color-sensor-recognition-ml.ino` — **Approach A**, planned future path
  for a generated ML model.
- `rgb-color-sensor-recognition-baseline\rgb-color-sensor-recognition-baseline.ino` — **Approach C**, planned future
  path for the simpler single-center matcher.

Generate the multi-cluster block for Approach B with:

`python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --output rgb-color-sensor\data\generated-clusters.txt`

Then paste the output block into the `// BEGIN GENERATED CLUSTER DATA` section
of `rgb-color-sensor-recognition-clusters\rgb-color-sensor-recognition-clusters.ino`.
