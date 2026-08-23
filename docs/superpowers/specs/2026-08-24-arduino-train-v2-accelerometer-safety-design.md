# Arduino Train v2 Accelerometer Safety Design

## Goal

Add MPU-6050 accelerometer-based tilt and crash safety without changing the
existing SW-520D tilt behavior or using the MPU interrupt pin.

## Scope

Add a direct I2C MPU-6050 driver at address `0x68`, a `DEBUG_ACCELEROMETER`
category, and configurable accelerometer safety thresholds. Do not add an
external library, EEPROM events, wake-on-motion, gyroscope use, or `INT` pin
handling.

## Design

### MPU-6050 access

`40-sensors.ino` will contain a small register-level driver. During normal
sensor initialization it probes `WHO_AM_I`, wakes the MPU-6050, selects the
default +/-2 g accelerometer scale, and reads six accelerometer bytes every
20 ms. If initialization or a later read fails, accelerometer safety is
disabled for that boot; existing train behavior remains unchanged.

`config.h` will define the I2C address, read cadence, acceleration scale,
tilt thresholds, crash threshold, and signed forward-axis mapping. The
forward axis is configurable because the module has not yet been mounted.

### Tilt safety

The code calculates chassis tilt from the gravity vector rather than from a
boot-time baseline. It therefore measures against real gravity even when the
train starts on an incline.

At or above 45 degrees continuously for the existing `TILT_STABLE_MS` (1000
ms) debounce interval, a separate accelerometer tilt latch performs the
current tilt action: stop and reset the motor selection, leave auto mode,
cancel jog, show red, and play the tilt warning. It blocks IR motor-control
commands while active. It clears only below 30 degrees. The existing
`tiltStopLatched` and SW-520D update path are untouched; motor commands remain
blocked while either tilt latch is set.

The 45/30-degree comparisons use squared raw acceleration components, avoiding
floating-point trigonometry on the AVR. The calculation assumes the module is
mounted flat relative to the chassis; a fully inverted chassis remains a tilt
fault.

### Crash safety

While `Speed > 0`, the code compares consecutive samples on the configured
forward axis. If the signed change is at least 600 mg opposite the current
motor direction, it treats the event as a crash: stop/reset motor selection,
cancel jog, disable auto mode, show red, and enable the existing siren.

A crash latch prevents automatic restart. The next IR motor-control command
clears that latch and silences the siren, then follows the command's existing
normal behavior. The crash algorithm detects sudden deceleration, not proof of
continuous train movement; it intentionally does not claim to detect a
quietly stalled or derailed train.

### Diagnostics

`DEBUG_ACCELEROMETER` defaults to off and is included in `DEBUG_ANY`. When
enabled, it reports initialization status, periodic raw acceleration samples,
tilt state changes, and crash decisions. Periodic output is rate limited.

## Error Handling

MPU initialization and read failures must be explicitly logged when its debug
category is enabled. They must not latch a fault, stop the train, change the
SW-520D behavior, or silently substitute fabricated acceleration data.

## Validation

- Build the sketch with `DEBUG_ACCELEROMETER` enabled and disabled.
- Verify an absent MPU-6050 leaves normal SW-520D and motor behavior intact.
- With the module mounted flat, verify a 45-degree tilt held for
  `TILT_STABLE_MS` stops the motor and that reducing below 30 degrees clears
  only the accelerometer latch.
- Verify the SW-520D still latches and clears exactly as before.
- Tune and verify the forward-axis configuration with debug data, then produce
  a controlled sudden-stop event while driving to confirm the crash siren and
  next-command recovery.
