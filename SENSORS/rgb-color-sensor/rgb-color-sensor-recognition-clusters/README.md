# RGB Color Sensor Recognition Clusters

This sketch is the **Approach B** recognition-only version for the RGB color
sensor project.

## How to use it

Upload and test it with:

```powershell
arduino-cli compile -p COM8 --fqbn arduino:avr:uno --upload rgb-color-sensor\rgb-color-sensor-recognition-clusters
```

To regenerate cluster data later after new captures:

```powershell
python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --output rgb-color-sensor\data\generated-clusters.txt
```

Then paste the generated output into the `// BEGIN GENERATED CLUSTER DATA`
section of:

`rgb-color-sensor\rgb-color-sensor-recognition-clusters\rgb-color-sensor-recognition-clusters.ino`

## Turning pasted capture text into cluster config

If you already have a text file that contains lines like `CAPTURE_SAMPLE,...`,
the calibration tool does **not** read that file directly. It expects the
captured data to exist as CSV files inside `rgb-color-sensor\data`.

Use this workflow:

1. Save the pasted capture text as a log file, for example
	`rgb-color-sensor\data\maroon-red-log.txt`.
2. Import that log into CSV files:

	```powershell
	python rgb-color-sensor\calibration_tool.py import-log --log rgb-color-sensor\data\maroon-red-log.txt --out rgb-color-sensor\data
	```

3. Generate the multi-cluster recognition block from the CSV files:

	```powershell
	python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --output rgb-color-sensor\data\generated-clusters.txt
	```

4. Paste the generated output into the `// BEGIN GENERATED CLUSTER DATA`
	section of the `.ino` file listed above.

For the Maroon Red captures in the pasted sample, the generated Red block is:

```cpp
// Red: 1 cluster(s), mean_distance=0.0
{ 7, { 623, 161, 216 }, 30 },
```
