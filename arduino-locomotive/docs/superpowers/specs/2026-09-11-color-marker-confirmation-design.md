# Color Marker Confirmation Design

## Goal

Make track-marker actions resistant to noisy readings without making the color
sensor blind while marker feedback is displayed. A marker action must run once
per physical marker encounter and must not rearm until the marker has been
confirmed absent.

## Configuration

Add beginner-readable color-sensor constants in `config.h`:

- `colorMarkerConfirmationSamples = 2`
- `colorMarkerLeaveSamples = 2`
- `colorMarkerConfirmationWindowMs = 100`

Both entry and leave confirmation use the same short time window. The existing
sample-origin comments (`// printed`, `// original`, and empty `//`) remain
unchanged because they distinguish calibration samples.

Remove `colorSaturationClearThreshold` and its classification check. Marker
presence remains controlled by each cluster's `minClearThreshold`, falling
back to `colorPresenceClearThreshold` when the cluster value is zero.

Retain the active `Adafruit_TCS34725` implementation. Do not restore or create
a hand-written TCS34725 driver. Remove obsolete custom-driver remnants from
active project files when they are proven unused, including the unused
`tcs34725Address` configuration constant. Ignore the `backup/` directory.

## Detection State Machine

Keep separate state for:

- the currently confirmed marker;
- the current candidate marker;
- the number of consecutive candidate samples;
- the time at which the candidate sequence began;
- the number and start time of consecutive unknown samples.

When no marker is confirmed:

1. An unknown reading clears the pending candidate.
2. A known color starts or continues a candidate sequence.
3. A different known color replaces the candidate and restarts its count.
4. A sequence older than 100 ms restarts at the current sample.
5. Two matching samples within 100 ms confirm the marker and run its visual
   feedback and action exactly once.

When a marker is confirmed:

1. Further readings of that marker do not execute another action and reset any
   pending leave sequence.
2. Readings of another known color also do not execute another action and reset
   the pending leave sequence.
3. Unknown readings start or continue the leave sequence.
4. A leave sequence older than 100 ms restarts at the current sample.
5. Two unknown readings within 100 ms confirm that the marker was left, clear
   the confirmed marker, and permit a future marker confirmation.

Requiring confirmed absence prevents `marker -> unknown -> marker` noise from
retriggering an action while the train remains over the same marker.

## Sampling and Visual Feedback

The one-second marker-color display is independent of color sampling.
`updateColorSensor()` continues reading and advancing confirmation state while
the blink is active.

Blink expiry is serviced before checking whether color sensing is enabled, so
turning the sensor off cannot leave `markerColorBlinkActive` or the displayed
marker color stuck. Disabling the sensor also clears pending and confirmed
detection state so a later enable starts with a fresh observation.

The Adafruit library's `getRawData()` call contains an integration-time delay,
so comments must describe the updater as periodic rather than non-blocking.

## Comments and Cleanup

Update relevant beginner-facing comments to:

- state that the active sensor driver is the Adafruit TCS34725 library;
- describe the actual 24 ms integration time;
- avoid claiming RGBC is read in one I2C burst;
- explain candidate confirmation and marker-left rearming;
- explain why visual feedback must not pause sensing;
- explain the meaning and units of each new configuration constant;
- remove saturation-threshold documentation;
- correct active project documentation that still claims the color sensor uses
  a hand-written driver;
- preserve calibration sample-origin comments exactly as requested.

Remove the unreachable duplicate `break` in the red-marker action. Remove
color-sensor state that is proven unused only when doing so does not affect
unrelated work already present in the dirty worktree.

## Verification

Do not add native color-sensor tests as part of this change. Review the state
transitions directly against these required sequences while implementing:

- one known sample does not trigger;
- two matching known samples within 100 ms trigger once;
- samples outside the window restart confirmation;
- conflicting known colors restart confirmation;
- one unknown sample does not rearm;
- two unknown samples within 100 ms rearm;
- `unknown -> confirmed marker -> unknown` does not rearm;
- any known color during leave confirmation resets the leave sequence;
- a confirmed marker cannot retrigger without confirmed leave;
- readings taken during an active marker blink advance confirmation state and
  may trigger the next eligible action;
- disabling during a blink clears the blink and detection state.

Compile the complete Nano sketch with warnings and run the existing native
suite without adding new color-specific cases.
