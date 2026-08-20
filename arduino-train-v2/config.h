#pragma once
// "#pragma once" tells the compiler to only process this header file one time, even if it gets
// included from multiple .ino tabs. Without it, re-including the same file could redefine the
// same constants twice and fail to compile.

// ================================================================================================
// File description
// ================================================================================================
// Shared compile-time settings used by more than one sketch module.
// Module-specific settings live at the top of 20-motor.ino, 30-lights-and-sounds.ino,
// 50-power-management.ino, and 40-sensors.ino so related knobs stay near related logic.

// ############################################################################
// # USER SETTINGS SHARED BY MULTIPLE MODULES - SAFE TO CHANGE BEFORE BUILD   #
// ############################################################################

// IRremote uses Timer1 so Timer2 remains free for tone() on the passive buzzer.
// Unit: compile-time feature switch.
// Safe change: leave enabled unless you also redesign the IR + buzzer timing.
// Wrong value effect: the remote can stop working while tones or melodies are playing.
// These "#define NAME" lines (with no value) are feature-switch macros: they must be defined
// *before* <IRremote.hpp> is included (see the #include order in arduino-train-v2.ino) because the
// IRremote library reads them at compile time to decide which hardware timer to use and which
// remote-control protocols to build support for. Arduino's Timer1 and Timer2 are internal hardware
// counters shared by several features (PWM output, tone(), IRremote's timing); telling IRremote to
// use Timer1 keeps it out of Timer2's way. EXCLUDE_* macros remove unused remote-control protocol
// decoders to save flash memory, since this train only needs the NEC protocol used by the remote.
#define IR_USE_AVR_TIMER1
#define DECODE_NEC
#define EXCLUDE_UNIVERSAL_PROTOCOLS
#define EXCLUDE_EXOTIC_PROTOCOLS
#define NO_LED_FEEDBACK_CODE
#define RAW_BUFFER_LENGTH 100

// EEPROM logging keeps a small boot/event history.
// Unit: 1 = enabled, 0 = disabled.
// Safe change: set to 0 only if you need extra flash and accept losing diagnostics.
// Wrong value effect: turning it off removes boot/fault history used for troubleshooting.
// The "#ifndef ... #define ... #endif" pattern below means "only define this if it isn't already
// defined". This lets an advanced user override the value from outside this file (for example,
// via a build tool's extra compiler flags) without needing to edit config.h at all. If nothing
// else defines it first, this file provides the default value shown here.
#ifndef ENABLE_EEPROM_LOGGING
#define ENABLE_EEPROM_LOGGING 1
#endif

// Per-module debug flags. Turn on one or multiple while troubleshooting. Turn off for normal train operation.
// DEBUG_* = 1 = print debug messages, 0 = silent.
// Too many enabled flags can make serial logs noisy. Debug modules also use flash and SRAM. If multiple are enabled, in some cases more memory than available on the Nano may be needed and the code won't compile. If that happens, disable some debug flags and recompile.
// Each flag below uses the same "#ifndef / #define / #endif" override pattern explained above for
// ENABLE_EEPROM_LOGGING: it only takes the default value shown here if nothing else already
// defined it first.
#ifndef DEBUG_IR_REMOTE
#define DEBUG_IR_REMOTE 1
#endif
#ifndef DEBUG_MOTOR
#define DEBUG_MOTOR 1
#endif
#ifndef DEBUG_COLOR_SENSOR
#define DEBUG_COLOR_SENSOR 0
#endif
#ifndef DEBUG_DISTANCE_SENSOR
#define DEBUG_DISTANCE_SENSOR 0
#endif
#ifndef DEBUG_TILT_SENSOR
#define DEBUG_TILT_SENSOR 0
#endif
#ifndef DEBUG_VOLTAGE_METER
#define DEBUG_VOLTAGE_METER 0
#endif
#ifndef DEBUG_LEDS
#define DEBUG_LEDS 0
#endif
#ifndef DEBUG_SOUND
#define DEBUG_SOUND 0
#endif
#ifndef DEBUG_EEPROM
#define DEBUG_EEPROM 0
#endif

// --- Pins a builder may want to rewire ---
// These are the main hardware connections a builder may change before compiling.
// Keep each pin unique, and never move a function to D9 or D10 if it needs PWM.
// "constexpr" declares a typed constant that the compiler must be able to compute at compile time
// (unlike a plain "#define", which is just untyped text substitution done before compiling). Using
// constexpr here gives the compiler type-checking (for example, catching an accidental string
// where a number is expected) while still producing code just as small and fast as a #define.
constexpr int pinBatterySense = A0;
constexpr int pinTiltSensor = 9;
constexpr uint8_t vl53l0xAddress = 0x2A;
constexpr int pinVL53L0X_XSHUT = 3;
constexpr int pinIRReceiver = 2;
constexpr int pinMotor_IN1 = 5;
constexpr int pinMotor_IN2 = 6;
constexpr int pinMotorFault = 8;
constexpr int pinMotorSleep = 7;
constexpr int pinColorSensorLED = 4;
constexpr int pinBuzzer = 12;

// --- Shared motor and auto-drive settings ---
constexpr unsigned long BOOST_DURATION_MS = 10000UL;
constexpr unsigned long BOOST_COOLDOWN_MS = 50000UL;
constexpr uint16_t MAX_SAFE_MOTOR_MV = 7000;
constexpr uint16_t NORMAL_MAX_MOTOR_MV = 6000;
constexpr uint8_t NORMAL_MAX_SPEED_STEP = 3;
constexpr uint8_t BOOST_SPEED_STEP = 4;
constexpr uint16_t voltageSteps[] = { 0, 3500, 4500, 6000, 7000 };
constexpr int AUTO_SAMPLES_FOR_MEDIAN = 5;

// --- Shared power-management settings ---
constexpr unsigned long BATTERY_CHECK_INTERVAL_MS = 5000UL;
constexpr unsigned long BATTERY_WARNING_SIGNAL_MS = 3000UL;
constexpr unsigned long BATTERY_WARNING_REPEAT_MS = 60000UL;
constexpr unsigned long BATTERY_SHUTDOWN_SIGNAL_MS = 10000UL;
constexpr unsigned long IDLE_SLEEP_HEARTBEAT_MS = 32000UL;
constexpr unsigned long IDLE_SLEEP_HEARTBEAT_ON_MS = 100UL;
constexpr uint16_t BATTERY_LOW_WARNING_MV = 7250;
constexpr uint16_t BATTERY_LOW_SHUTDOWN_MV = 7150;
constexpr uint16_t BATTERY_WARNING_RECOVERY_MV = 7350;
constexpr uint16_t BATTERY_MAX_VALID_MV = 8500;
constexpr uint16_t BATTERY_MILLIVOLT_SCALE_NUMERATOR = 12221;
constexpr int BATTERY_ADC_MAX = 1023;
constexpr uint8_t BATTERY_ADC_SAMPLES = 8;

// --- Shared light and sound settings ---
constexpr int FrontLightOnOff = 1;
constexpr uint8_t colorSensorLEDOnLevel = HIGH;
constexpr uint8_t colorSensorLEDOffLevel = LOW;
constexpr uint16_t BATTERY_ALERT_TONE_HZ = 1500;
constexpr uint16_t buzzerPatternToneHz = 2200;

// --- Shared safety checks ---
// static_assert(condition, "message") is a *compile-time* check: the compiler evaluates the
// condition while building the sketch, and if it is false, the build fails immediately with the
// given message instead of producing a train that could misbehave. Unlike a runtime "if" check,
// this costs zero flash/RAM and catches a bad configuration (for example, mixed-up threshold
// constants) before the code is ever uploaded to the Arduino.
static_assert(BATTERY_LOW_SHUTDOWN_MV < BATTERY_LOW_WARNING_MV, "Shutdown threshold must be below warning threshold.");
static_assert(BATTERY_LOW_WARNING_MV < BATTERY_WARNING_RECOVERY_MV, "Warning recovery must sit above the warning threshold.");
static_assert(NORMAL_MAX_SPEED_STEP < BOOST_SPEED_STEP, "Boost step must come after the normal top step.");
