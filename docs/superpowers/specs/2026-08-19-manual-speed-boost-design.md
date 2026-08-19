# Manual Speed Boost Design

## Goal

Add a fourth manual speed level for either direction: 7 V boost for at most 10
seconds, followed by a 50-second boost-only lockout.

## Behavior

- `CH+` and `CH-` continue to select manual levels regardless of direction.
- Normal levels remain stop, 3.5 V, 4.5 V, and 6 V.
- From level 3, `CH+` selects level 4 only when boost is available.
- Level 4 targets 7 V and runs for at most `BOOST_DURATION_MS` (10 seconds).
- Expiry automatically returns to normal level 3 in the current direction.
- The lockout begins when boost ends, including an early cancel by `CH-`, stop,
  mode switch, or motor fault. It lasts `BOOST_COOLDOWN_MS` (50 seconds).
- Cancelling a normal level never starts a boost lockout; only an active boost
  does.
- During lockout, `CH+` at level 3 keeps normal level 3. `CH-`, stop,
  direction, jog, color sensor, and other existing controls retain their
  current behavior.
- Auto-distance and momentary jog modes never select boost.

## Implementation

Keep normal and boost voltage limits separate constants. Extend the manual
speed array to five entries and make auto-distance explicitly use the normal
level-3 voltage rather than the final manual array entry. Track boost active,
expiry, and cooldown timestamps with non-blocking wrap-safe `millis()` checks.

## Validation

- Compile for Arduino Nano ATmega328P.
- Verify normal steps and auto-distance still cap at 6 V.
- Verify boost returns to level 3 after 10 seconds and cannot be requested for
  50 seconds, in either direction.
- Verify each early-cancel path returns to normal behavior and starts lockout
  only when boost was active.
