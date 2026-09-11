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
- `colorMarkerFeedbackDurationMs = 1000UL`

Confirmation is based only on consecutive samples, with no elapsed-time
window. The existing sample-origin comments (`// printed`, `// original`, and
empty `//`) remain unchanged because they distinguish calibration samples.
`colorMarkerFeedbackDurationMs` controls how long the confirmed marker color is
shown without affecting sensor sampling.

Move the existing `RgbColor` enum into `config.h` so color configuration can
use readable named values. Add a flash-resident `markerFeedbackColors[]` table
indexed by `TrackMarkerClass`, including an `RgbColor::Off` entry for
`MarkerUnknown`. Keep the current same-color defaults:

| Marker | Feedback LED color |
| --- | --- |
| Unknown | Off |
| White | White |
| Blue | Blue |
| Green | Green |
| Magenta | Magenta |
| Yellow | Yellow |
| Red | Red |

`41-color-sensor.ino` reads the configured value with `pgm_read_byte()` when a
marker is confirmed. Remove the hard-coded marker-to-LED-color switch. Marker
actions remain separate from this visual mapping.

Do not change any clear-channel thresholds or classification behavior related
to brightness or saturation. Preserve `colorPresenceClearThreshold`,
`colorSaturationClearThreshold`, every cluster's `minClearThreshold`, and all
existing fallback and saturation checks.

Retain the active `Adafruit_TCS34725` implementation. Do not restore or create
a hand-written TCS34725 driver. Remove obsolete custom-driver remnants from
active project files when they are proven unused. Keep `tcs34725Address` as a
commented-out reference with an explanation that the active Adafruit library
uses the sensor's fixed default `0x29` address. Ignore the `backup/` directory.

## Detection State Machine

Keep separate state for:

- the currently confirmed marker;
- the current candidate marker;
- the number of consecutive candidate samples;
- the number of consecutive unknown samples.

When no marker is confirmed:

1. An unknown reading clears the pending candidate.
2. A known color starts or continues a candidate sequence.
3. A different known color replaces the candidate and restarts its count.
4. Two consecutive matching samples confirm the marker and run its visual
   feedback and action exactly once.

When a marker is confirmed:

1. Further readings of that marker do not execute another action and reset
   pending leave and different-color confirmation.
2. Unknown readings start or continue the leave sequence and clear a pending
   different-color candidate.
3. Two consecutive unknown readings confirm that the marker was left, clear
   the confirmed marker, and permit a future marker confirmation.
4. A different known color resets the leave count and starts or continues a
   different-color candidate.
5. Two consecutive readings of the same different known color directly replace
   the confirmed marker and run the new marker's action without requiring an
   intervening unknown reading.

Requiring two consecutive samples prevents isolated color or unknown readings
from changing the confirmed state.

If color sampling is paused for a momentary jog, clear only the pending entry,
leave, and different-color counts. Keep the already confirmed marker latched,
but do not combine a pre-jog sample with a post-jog sample as one consecutive
sequence.

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
- accurately explain the existing clear and saturation thresholds without
  changing their values or behavior;
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
- two consecutive matching known samples trigger once;
- conflicting known colors restart confirmation;
- one unknown sample does not rearm;
- two consecutive unknown samples rearm;
- `unknown -> confirmed marker -> unknown` does not rearm;
- two consecutive readings of a different known color directly confirm and run
  the new marker;
- the confirmed color during leave confirmation resets the leave sequence;
- a jog pause clears partial confirmation counts without clearing the confirmed
  marker;
- a confirmed marker cannot retrigger without confirmed leave;
- readings taken during an active marker blink advance confirmation state and
  may trigger the next eligible action;
- disabling during a blink clears the blink and detection state.

Compile the complete Nano sketch with warnings and run the existing native
suite without adding new color-specific cases.
