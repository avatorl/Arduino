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
