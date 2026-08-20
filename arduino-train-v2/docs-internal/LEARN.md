# Learn the Arduino Train v2 Code

This guide is a map for learning the `arduino-train-v2` sketch while reading the
real project files. It assumes an Arduino Nano with an ATmega328P, but it marks
the places where the code is AVR/Nano-specific. The goal is not to explain all
of C++ first. Instead, each idea is connected to something the train actually
does.

## 1. Project and file map

The Arduino IDE treats the `.ino` tabs as one sketch, even though the code is
split into files:

- `arduino-train-v2.ino` owns shared types and state, `setup()`, and `loop()`.
- `10-ir-remote.ino` decodes NEC IR frames and routes remote commands.
- `20-motor.ino` drives the DRV8833, speed steps, boost, faults, and distance
  based driving.
- `30-lights-and-sounds.ino` controls LEDs, `tone()`, patterns, and melodies.
- `40-sensors.ino` contains small register-level TCS34725 and VL53L0X drivers,
  plus color, distance, and tilt processing.
- `50-power-management.ino` contains battery policy, EEPROM logs, and sleep.
- `config.h` contains shared pins, feature switches, thresholds, and
  compile-time checks.
- `melodies.h` contains constant note tables.
- `test/native/` contains host-side tests for pure logic.

Start with `README.md` for the hardware story, then read `config.h`, the shared
state in `arduino-train-v2.ino`, and the feature tabs in the order above.

## 2. Pin, timer, and I2C cheat sheet

| Connection | Pin or address | Purpose |
| --- | --- | --- |
| IR receiver | D2 | NEC remote input and AVR wake interrupt |
| VL53L0X XSHUT | D3 | Hold the distance sensor in reset |
| Color sensor LED | D4 | Marker illumination |
| Motor IN1 / IN2 | D5 / D6 | DRV8833 PWM motor outputs |
| DRV8833 nSLEEP | D7 | Enable or disable motor driver |
| DRV8833 nFAULT | D8 | Active-low fault input |
| Tilt switch | D9 | Active-low/open input with `INPUT_PULLUP` |
| Buzzer | D12 | `tone()` output |
| Battery divider | A0 | ADC battery measurement |
| I2C SDA / SCL | A4 / A5 | MCP23008, TCS34725, and VL53L0X |

This table describes the current Nano wiring, not a universal Arduino rule.
`config.h` forces IRremote to Timer1. Therefore do not use PWM on D9 or D10:
those pins belong to Timer1 on the ATmega328P. `tone()` uses another hardware
timer in this project. Timer allocation is an AVR/Nano concern; portable C++
does not promise these pin-to-timer mappings.

Both the TCS34725 and VL53L0X power up at I2C address `0x29`. The train holds
the VL53L0X low on XSHUT, starts the color sensor at `0x29`, wakes the
VL53L0X, and changes its address to `0x2A`. XSHUT is a hardware sequencing
signal, not an I2C command.

## 3. `setup()`, `loop()`, and one-button flow

`setup()` runs once after reset. It clears and disables the AVR watchdog,
starts debug serial when enabled, configures pins, initializes the I2C sensors,
measures the battery, writes a boot record, applies battery policy, sets lights
and sound, builds speed steps, and starts the IR receiver. The AVR watchdog is
then enabled for normal operation.

`loop()` is a cooperative scheduler. It must return quickly so every subsystem
gets regular service. The current order checks the motor fault, idle sleep and
battery guard, jog timeout, automatic distance speed, boost expiry, tilt,
remote input, color sensing, sound, lights, and other updates. A long `delay()`
inside this path would make the train slow to react. A few short delays remain
in hardware startup or deliberate power-down paths.

A useful one-button trace is the remote `CH+` speed command:

1. `translateIR()` calls `irReceive()`.
2. IRremote decodes a NEC frame and returns its 8-bit command.
3. The command router matches `buttonCHplus`.
4. Guards reject it if the battery is in shutdown, the train is tilted, or a
   motor fault is latched.
5. The speed step changes and `applySpeedStep()` computes safe PWM.
6. `analogWrite()` updates D5/D6, and the next loop continues monitoring safety.

Held buttons generate NEC repeat frames. The code remembers the last real
command so momentary forward/backward jogs can continue, then stops when
repeats stop arriving.

## 4. Memory on a small Nano

The ATmega328P has about 32 KB of flash and 2 KB of SRAM. Flash normally holds
program code; SRAM holds variables, stack, and temporary buffers. A constant
table can still consume SRAM unless it is explicitly placed in flash.

`PROGMEM` is an AVR-specific way to keep read-only melody, lookup, and register
tables in flash. `melodies.h` declares `const int16_t melodyDemo[] PROGMEM`.
The player reads one value at a time with `pgm_read_word()` instead of copying
the whole song into RAM. On a non-AVR platform, `PROGMEM` may be unavailable or
may behave like an ordinary attribute, so guard it with `#if defined(__AVR__)`
when writing portable code.

`F("text")` keeps a debug string in flash. Use it with Arduino `Serial.print`
and `Serial.println`; do not assume it is an ordinary `const char*` on AVR.
The project uses flash strings for status and diagnostic text because SRAM is
needed for live sensor and motor state.

EEPROM is different from flash: it is small persistent storage that survives
reset and power loss. The project uses `EEPROM.update()` so a byte is written
only when its value changes, and `EEPROM.get()` / `EEPROM.put()` for typed
records. The boot and event logs are ring buffers. They advance a bounded slot
index and cap runtime fault writes, reducing wear. Never write an EEPROM record
on every pass through `loop()`.

## 5. C++ basics used here

Declarations tell the compiler a name and type. A forward declaration such as
`void updateMelody();` lets one function call another before its definition.
Definitions provide the body later. In a multi-tab Arduino sketch, all tabs
are combined, but explicit declarations still make dependencies readable.

Scope controls where a name exists. A variable inside a function is local and
temporary; a global is shared state that survives between loop calls. Keep
shared state global only when multiple modules really need it.

Use fixed-width types when the size matters: `uint8_t` is an unsigned 8-bit
value, `uint16_t` is 16-bit, and `uint32_t` is 32-bit. `unsigned long` is the
Arduino type commonly returned by `millis()`. `const` prevents accidental
changes. `constexpr` creates a typed compile-time constant, such as
`pinMotor_IN1` or `BOOST_DURATION_MS`, and is preferable to an untyped macro
for ordinary values.

An `enum class` gives strongly scoped names such as `RgbColor::Red`. A plain
`enum` is convenient when values must behave like integers or bit flags. A
`struct` groups related data and behavior; the sensor helpers are structs with
register constants, address state, and methods.

Pointers store addresses. References provide another name for an existing
object. Arrays hold a fixed sequence, and a pointer often refers to its first
element. The melody player stores a pointer to either RAM or PROGMEM and a flag
telling it how to read. Templates allow one algorithm to work with several
types; use `static_assert` inside or beside template-like code to reject bad
configuration at compile time. `nullptr` means a pointer intentionally points
nowhere and is safer than integer zero.

Use casts deliberately. The battery code widens sums to `uint32_t` before
multiplication so ADC totals cannot overflow a 16-bit value. Convert back only
after checking bounds. This wider-arithmetic pattern is portable C++, while
`pgm_read_word()` and AVR watchdog registers are platform-specific.

## 6. Digital I/O, PWM, and timers

`pinMode()` selects input or output behavior. `digitalRead()` samples a pin;
`digitalWrite()` drives it. `INPUT_PULLUP` makes an input normally HIGH and
expects a switch to connect it to ground, so the tilt and fault signals are
active-low. Always document whether LOW means safe, active, or fault.

`analogWrite()` on a PWM-capable pin changes duty cycle, not a true analog
voltage. The motor code converts a desired millivolt target into bounded PWM
using integer arithmetic and the measured battery voltage. D5 and D6 are the
motor PWM pins in this design. Hardware timers generate PWM, IR timing, and
tones, so changing a pin or library can create conflicts.

`tone()` starts a square wave on the buzzer and `noTone()` stops it. It is not a
general timer API. Read `config.h` before changing timer-related libraries.

## 7. I2C and register-level drivers

I2C is a shared two-wire bus: SDA carries data and SCL carries the clock.
`Wire.beginTransmission(address)`, `Wire.write(register)`, and
`Wire.endTransmission()` send a register request. A repeated start,
`endTransmission(false)`, followed by `requestFrom()` reads without releasing
the bus between the register selection and response.

The project uses small drivers instead of complete sensor libraries. This saves
flash and SRAM but requires checking every transaction result. TCS34725 color
values arrive as low and high bytes; combine them with a cast, shift, and
bitwise OR. The VL53L0X driver checks its model ID, writes its initialization
sequence, sets the address, and reports explicit `false` on failures.

The two-sensor address collision explains the XSHUT sequence in section 2.
When adding another I2C device, verify its address, pull-ups, voltage levels,
and startup behavior. I2C addresses are 7-bit values; do not shift them unless
the API specifically expects an 8-bit address byte.

## 8. Timing and state machines

Prefer a state machine over `delay()`. Store a state, a timestamp, and enough
data to resume later. The melody player stores the current pair index and
`melodyStepStarted`; each loop checks whether the note duration elapsed. LED
patterns, battery warning signals, boost cooldown, debounce, and obstacle
handling use the same idea.

For rollover-safe elapsed time, subtract timestamps:

```cpp
if (millis() - startedAt >= intervalMs) {
  // The interval has elapsed.
}
```

Unsigned subtraction remains correct when `millis()` wraps. For a target time,
the project uses `(long)(millis() - target) >= 0` where appropriate. Do not
compare `millis() >= target` for deadlines that must survive the roughly
49-day AVR rollover.

The sleep path is intentionally different. It disables outputs, attaches an
interrupt to D2, and uses `LowPower.powerDown()` in timed chunks. The ISR only
sets a `volatile` flag; it does not print, delay, or perform I2C. After wake,
the code restores sensors and reuses the captured IR command when possible.

## 9. Preprocessor and embedded patterns

`#pragma once` prevents repeated header inclusion. `#ifndef` defaults in
`config.h` let a build override a feature before the file supplies its default.
`#if defined(__AVR__)` protects AVR registers, watchdog calls, and PROGMEM code
from non-AVR host tests.

Debug output uses conditional variadic macros. When `DEBUG_MOTOR` is zero, the
corresponding macro expands away at compile time, avoiding runtime branches and
many strings. The `do { ... } while (0)` form makes a multi-statement macro act
like one statement.

Bit masks and shifts pack flags into small bytes. Lookup tables replace long
chains of calculations. Integer interpolation avoids unnecessary floating
point work. Safety latches remember a fault until a deliberate safe clear;
stopping once is not enough if later code could restart the motor.

Return `false`, zero, or an invalid marker explicitly when hardware access
fails. Callers can then refuse to drive or log the fault. The AVR watchdog is
disabled during setup and enabled for normal operation; `wdt_reset()` is called
each loop. Sleep temporarily disables it because timed low-power sleep reuses
the watchdog hardware. These watchdog details are AVR-specific.

## 10. Common mistakes

- Putting large constant arrays in RAM instead of `PROGMEM`.
- Printing ordinary string literals repeatedly instead of using `F()`.
- Writing EEPROM every loop, exhausting its write endurance.
- Using D9/D10 PWM while IRremote owns Timer1.
- Leaving both I2C sensors at `0x29`.
- Calling `delay()` in the cooperative scheduler.
- Testing `millis() >= deadline` instead of subtracting timestamps.
- Doing long work inside an interrupt service routine.
- Treating active-low fault or tilt inputs as active-high.
- Multiplying ADC values in a narrow integer type.
- Clearing a safety latch without checking the physical fault.
- Forgetting that `.ino` tabs share one generated translation unit.

## 11. Suggested reading order

Read `README.md`, then `config.h`. Next trace `setup()` and one pass through
`loop()` in `arduino-train-v2.ino`. Follow one IR command in
`10-ir-remote.ino`, one motor update in `20-motor.ino`, and one melody in
`30-lights-and-sounds.ino`. Then study the I2C register helpers in
`40-sensors.ino`, followed by battery and sleep behavior in
`50-power-management.ino`. Finally compare the pure logic helpers and tests in
`test/native/`. Change one constant at a time, compile for
`arduino:avr:nano`, and use the debug flags to observe only the subsystem you
are studying.
