# RGB Color Sensor LED Feedback Design

## Goal

Show the RGB LED a representative color for each color-sensor sample that is
confirmed by the existing detection logic. Hold the displayed color for five
seconds after the most recent confirmed detection, then turn it off.

## Hardware Configuration

- TCS34725 illumination LED: D2
- RGB LED red channel: D3
- RGB LED green channel: D5
- RGB LED blue channel: D6

The sketch will retain the project's common-anode LED convention, inverting
PWM values before writing them. The RGB LED's common leg must be connected to
the positive supply through the appropriate circuit and each color channel
must use its current-limiting resistor.

Initialize the RGB LED in its off state: `analogWrite(3, 255)`,
`analogWrite(5, 255)`, and `analogWrite(6, 255)`. Use those same three PWM
values whenever turning it off. The sensor-success status message must be
exactly `Sensor LED on D2 enabled`.

## Behavior

1. Keep the existing three-reading, two-match classification process.
2. On a confirmed sample, display its fixed representative RGB palette color
   and set the last-confirmed timestamp to `millis()`.
3. On an unrecognized batch, leave the currently displayed color unchanged.
4. During each main-loop iteration, only while an LED output is active, turn
   the RGB LED off at the first iteration where at least five seconds have
   elapsed since the last confirmed sample.
5. A new confirmed sample before the timeout replaces the displayed color and
   restarts the five-second timeout.

The representative palette is independent of raw sensor brightness so the LED
output remains stable:

| Sample | RGB value |
| --- | --- |
| Jade White | 255, 255, 255 |
| Bambu Green | 0, 255, 80 |
| Cyan | 0, 255, 255 |
| Pumpkin Orange | 255, 85, 0 |
| Maroon Red | 128, 0, 0 |

## Implementation Boundaries

Add a focused helper for applying an RGB value to the LED and a helper that
maps a confirmed sample index to its palette color. Store only the timestamp
and whether an output is active. Use the rollover-safe comparison
`millis() - lastConfirmedMs >= 5000UL` for the timeout.

Calibration and serial commands remain unchanged. The sensor initialization
status text must accurately report the new D2 illumination LED pin.

## Validation

Compile the sketch for the Uno/Nano target. Verify on hardware that each
recognized sample illuminates its representative color, refreshed detections
extend the hold period, and the LED switches off after five seconds without
blocking sensor sampling or serial calibration.
