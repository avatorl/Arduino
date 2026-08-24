#pragma once
// "#pragma once" tells the compiler to only process this header file one time, even if it gets
// included from multiple .ino tabs. Without it, re-including the same file could redefine the
// same constants twice and fail to compile.

// ================================================================================================
// File description
// ================================================================================================
// Central home for user-changeable train settings.
// Group settings by module so a train developer can tune wiring, thresholds, timings, calibration,
// and remote mappings here without hunting through multiple .ino files.

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
#define DEBUG_MOTOR 0
#endif
#ifndef DEBUG_COLOR_SENSOR
#define DEBUG_COLOR_SENSOR 0
#endif
#ifndef DEBUG_DISTANCE_SENSOR
#define DEBUG_DISTANCE_SENSOR 0
#endif
#ifndef DEBUG_TILT_SENSOR
#define DEBUG_TILT_SENSOR 1
#endif
#ifndef DEBUG_ACCELEROMETER
#define DEBUG_ACCELEROMETER 0
#endif
// Keep the retired VL53L0X driver (90-legacy-vl53l0x.ino) out of the build.
// Unit: compile-time feature switch.
// Wrong value effect: at 1 the old driver compiles again and wastes flash; it is reference code
// only and is not wired to anything.
#ifndef ENABLE_VL53L0X_LEGACY_DRIVER
#define ENABLE_VL53L0X_LEGACY_DRIVER 0
#endif
#ifndef DEBUG_POWER_MANAGEMENT
#define DEBUG_POWER_MANAGEMENT 1
#endif
#ifndef DEBUG_LEDS
#define DEBUG_LEDS 0
#endif
#ifndef DEBUG_SOUND
#define DEBUG_SOUND 1
#endif
#ifndef DEBUG_EEPROM
#define DEBUG_EEPROM 1
#endif
#ifndef DISABLE_VOLTAGE_METERING
#define DISABLE_VOLTAGE_METERING 1  // Set to 1 to disable voltage metering and assume 5.0V constant (for testing with 5V power source). Set to 0 to enable voltage metering (normal operation).
#endif

// --- I2C Addresses ---
// The I2C bus is shared by multiple devices. Each device has a unique 7-bit address.
constexpr uint8_t mcp23008Address = 0x24; // MCP23008 LED expander = 0x24 (A0 LOW, A1 LOW, A2 HIGH)
constexpr uint8_t tcs34725Address = 0x29; // TCS34725 RGB color sensor = 0x29 (default)
constexpr uint8_t vl53l1xAddress = 0x2A; // VL53L1X distance sensor = 0x2A (changed from default 0x29 using XSHUT pin)
constexpr uint8_t mpu6050Address = 0x68; // MPU6050 accelerometer = 0x68 (default, AD0 pin LOW)

// --- Pins a builder may want to rewire ---
// These are the main hardware connections a builder may change before compiling.
// Keep each pin unique, and never move a function to D9 or D10 if it needs PWM.
// "constexpr" declares a typed constant that the compiler must be able to compute at compile time
// (unlike a plain "#define", which is just untyped text substitution done before compiling). Using
// constexpr here gives the compiler type-checking (for example, catching an accidental string
// where a number is expected) while still producing code just as small and fast as a #define.
constexpr int pinBatterySense = A0;
// A1 - unused
constexpr int pinColorSensorLED = A2;
constexpr int pinVL53L1X_XSHUT = A3;
// A4 - SDA (I2C data) shared by all I2C devices
// A5 - SCL (I2C clock) shared by all I2C devices
// A6 - unused
// A7 - unused
// D0 - RX (serial input) shared by USB serial and Bluetooth
// D1 - TX (serial output) shared by USB serial and Bluetooth
constexpr int pinIRReceiver = 2;
// D3 - unused
// D4 - unused
constexpr int pinMotor_IN1 = 5; // with PWM
constexpr int pinMotor_IN2 = 6; // with PWM
constexpr int pinMotorSleep = 7;
constexpr int pinMotorFault = 8;
constexpr int pinTiltSensor = 9;
// D10 - unused
// D11 - unused
constexpr int pinBuzzer = 12;
// D13 - unused (built-in LED)

// --- MCP23017 expander pin mapping ---
constexpr uint8_t led1RedExpanderPin = 0;
constexpr uint8_t led1GreenExpanderPin = 1;
constexpr uint8_t led1BlueExpanderPin = 2;
constexpr uint8_t led2RedExpanderPin = 3;
constexpr uint8_t led2GreenExpanderPin = 4;
constexpr uint8_t led2BlueExpanderPin = 5;
constexpr uint8_t ledGreenExpanderPin = 6;
constexpr uint8_t ledRearRedExpanderPin = 7;

// --- IR remote mapping ---
// Button codes for the NEC "Car MP3" handheld remote bundled with this build.
// Change these only if you swap to a different remote or remap train functions.
constexpr uint8_t buttonCHminus = 69;   // Speed -
constexpr uint8_t buttonCH = 70;        // Stop
constexpr uint8_t buttonCHplus = 71;    // Speed +
constexpr uint8_t buttonBackward = 68;  // Momentary backward jog
constexpr uint8_t buttonForward = 64;   // Momentary forward jog
constexpr uint8_t buttonPlayPause = 67; // Auto-speed toggle
constexpr uint8_t buttonEQ = 9;         // Mute / unmute
constexpr uint8_t button0 = 22;         // Color sensor ON/OFF
constexpr uint8_t button100plus = 25;   // Horn
constexpr uint8_t button200plus = 13;   // Siren
constexpr uint8_t button1 = 12;         // Play music 1
constexpr uint8_t button2 = 24;         // Play music 2
constexpr uint8_t button3 = 94;         // Play music 3
constexpr uint8_t button4 = 8;          // Play music 4
constexpr uint8_t button5 = 28;         // Play music 5
constexpr uint8_t button6 = 90;         // Play music 6
constexpr uint8_t button7 = 66;         // Play music 7
constexpr uint8_t button8 = 82;         // Play music 8
constexpr uint8_t button9 = 74;         // Battery test
constexpr unsigned long momentaryTimeout = 200UL; // Stop a held jog this long after repeats stop.

// --- Motor and drive settings ---
// Manual speed steps are expressed as requested motor voltage, then converted to PWM at runtime
// using the current battery voltage. The first entry must stay 0 for "stopped".
constexpr unsigned long DIR_DELAY = 1000UL;        // Coast time before reversing direction.
constexpr unsigned long BOOST_DURATION_MS = 10000UL; // How long level 4 boost may stay active.
constexpr unsigned long BOOST_COOLDOWN_MS = 50000UL; // Wait time before boost may be used again.
constexpr uint16_t MAX_SAFE_MOTOR_MV = 7000;         // Hard top voltage request the motor may ever see.
constexpr uint16_t NORMAL_MAX_MOTOR_MV = 6000;       // Normal top voltage outside boost mode.
constexpr uint8_t NORMAL_MAX_SPEED_STEP = 3;         // Highest regular manual step.
constexpr uint8_t BOOST_SPEED_STEP = 4;              // Extra manual step reserved for boost.
constexpr uint16_t voltageSteps[] = { 0, 3500, 4500, 6000, 7000 }; // Requested motor mV for steps 0..4.
constexpr int rampStep = 5;                          // PWM change per auto-speed ramp update.
constexpr unsigned long rampDelay = 80UL;            // Delay between ramp steps in auto mode.
constexpr int AUTO_SAMPLES_FOR_MEDIAN = 5;           // Distance samples kept for median filtering.
constexpr int AUTO_DISTANCE_STOP = 8;                // Stop auto drive when obstacle is closer than this (cm).
constexpr int AUTO_DISTANCE_RESTART = 11;            // Start moving again once obstacle clears this distance (cm).
constexpr int AUTO_DISTANCE_MAX_SPEED = 50;          // Distance at which auto mode may request full normal speed (cm).

// --- Sensor settings ---
// Track-marker classification labels and color-cluster calibration table.
// Tune markerClusters when testing shows track colors are being misclassified.
enum TrackMarkerClass : uint8_t {
  MarkerUnknown = 0,
  MarkerWhite,
  MarkerBrown,
  MarkerCyan,
  MarkerGreen,
  MarkerGrey,
  MarkerMagenta,
  MarkerOrange,
  MarkerYellow,
  MarkerRed
};

struct PrototypeRgb {
  uint16_t r;
  uint16_t g;
  uint16_t b;
};

struct MarkerClusterDefinition {
  TrackMarkerClass markerClass;
  PrototypeRgb center;
  uint16_t maxDistance;
};

// Color-sensor sampling and calibration.
constexpr unsigned long colorSensorReadEveryMs = 60UL; // Time between color-sensor reads.
constexpr uint16_t colorClearMinThreshold = 160;       // Minimum clear-channel brightness before trusting a read.
constexpr uint16_t colorMatchClearThreshold = 304;     // Extra brightness gate for track-marker matching.
constexpr float whiteBalanceRedGain = 1.000f;          // Red-channel calibration multiplier.
constexpr float whiteBalanceGreenGain = 1.212f;        // Green-channel calibration multiplier.
constexpr float whiteBalanceBlueGain = 2.318f;         // Blue-channel calibration multiplier.
// Each entry is a known track-marker color center plus its allowed matching radius.
const MarkerClusterDefinition markerClusters[] PROGMEM = {
  { MarkerWhite, { 334, 333, 333 }, 32 },
  { MarkerBrown, { 392, 322, 287 }, 31 },
  { MarkerBrown, { 415, 305, 280 }, 22 },
  { MarkerBrown, { 428, 289, 284 }, 25 },
  { MarkerCyan, { 158, 313, 529 }, 60 },
  { MarkerGreen, { 160, 518, 322 }, 62 },
  { MarkerGreen, { 203, 482, 315 }, 52 },
  { MarkerGrey, { 303, 345, 352 }, 38 },
  { MarkerGrey, { 327, 331, 342 }, 22 },
  { MarkerGrey, { 347, 317, 336 }, 27 },
  { MarkerMagenta, { 455, 201, 344 }, 14 },
  { MarkerMagenta, { 476, 193, 331 }, 18 },
  { MarkerMagenta, { 488, 186, 326 }, 10 },
  { MarkerOrange, { 542, 254, 204 }, 22 },
  { MarkerRed, { 514, 180, 305 }, 27 },
  { MarkerRed, { 550, 191, 259 }, 36 },
  { MarkerRed, { 572, 176, 252 }, 22 },
  { MarkerYellow, { 420, 368, 212 }, 14 },
  { MarkerYellow, { 432, 353, 215 }, 18 },
};
constexpr uint8_t markerClusterCount = sizeof(markerClusters) / sizeof(markerClusters[0]);

// Distance-sensor timing and fault handling.
constexpr uint16_t distanceTofTimeoutMs = 50;          // VL53L1X timeout for one measurement attempt.
constexpr uint32_t distanceTofTimingBudgetUs = 33000UL; // VL53L1X measurement timing budget.
constexpr uint32_t distanceTofContinuousPeriodMs = 50UL; // VL53L1X continuous-mode period.

// Distance-sensor detection cone (VL53L1X region of interest).
// Smaller ROI = narrower cone = better floor and side-wall rejection, but shorter range.
// Unit: SPADs on the sensor's 16x16 array. The driver clamps width/height to 4..16.
// Safe change: 4x4 is roughly a 15 degree cone, 16x16 is the full ~27 degrees.
// Wrong value effect: a centre far from 199 combined with a small ROI can push the window off the
// SPAD array, which yields range status 13 and no usable readings at all.
constexpr uint8_t distanceTofRoiWidth = 4;
constexpr uint8_t distanceTofRoiHeight = 4;
constexpr uint8_t distanceTofRoiCenterSpad = 199;      // 199 is the array's optical centre.
constexpr unsigned long tofReadEveryMs = 50UL;         // How often the sketch consumes a ToF reading.
constexpr uint8_t maxConsecutiveTofFailures = 3;       // Latch a distance fault after this many misses.
constexpr unsigned long tofFailureGraceMs = 250UL;     // Brief grace period before treating misses as invalid.

// Tilt-sensor debounce. Increase if the sensor chatters, decrease if stop detection feels slow.
constexpr unsigned long TILT_STABLE_MS = 1000UL;
constexpr unsigned long TILT_QUIET_MS = 500UL;

// MPU-6050 accelerometer sampling and safety thresholds.
constexpr unsigned long mpu6050ReadEveryMs = 20UL;
constexpr uint8_t mpu6050AccelConfig = 0x00;       // +/-2 g
constexpr int16_t mpu6050AccelLsbPerG = 16384;     // +/-2 g scale
// Keep the degree labels and tangent-squared ratios together: ratios avoid runtime floating-point trigonometry.
constexpr uint16_t mpu6050TiltTriggerDegrees = 45;
constexpr uint16_t mpu6050TiltTriggerTanSquaredPermille = 1000;  // tan(45)^2
constexpr uint16_t mpu6050TiltRecoveryDegrees = 30;
constexpr uint16_t mpu6050TiltRecoveryTanSquaredPermille = 333;  // tan(30)^2
constexpr int16_t mpu6050CrashDeltaMg = 600;
// Confirm these signs from DEBUG_ACCELEROMETER output after physical installation.
constexpr int8_t mpu6050ForwardAxis = 0;      // 0 = X, 1 = Y
constexpr int8_t mpu6050ForwardAxisSign = 1;  // +1 or -1
constexpr int8_t mpu6050UprightZSign = 1;     // +1 or -1

// --- Power-management settings ---
constexpr unsigned long BATTERY_CHECK_INTERVAL_MS = 5000UL;     // Time between parked battery-health checks.
constexpr unsigned long BATTERY_WARNING_SIGNAL_MS = 3000UL;     // Length of the warning sound/light signal.
constexpr unsigned long BATTERY_WARNING_REPEAT_MS = 60000UL;    // How often warning mode reminds the user.
constexpr unsigned long BATTERY_SHUTDOWN_SIGNAL_MS = 10000UL;   // Length of the final shutdown signal.
constexpr unsigned long IDLE_SLEEP_HEARTBEAT_MS = 32000UL;      // Sleep heartbeat cycle while idling.
constexpr unsigned long IDLE_SLEEP_HEARTBEAT_ON_MS = 100UL;     // Heartbeat pulse ON time.
constexpr unsigned long idleTimeout = 5UL * 60UL * 1000UL;      // Inactivity time before entering idle sleep.
constexpr unsigned long IDLE_SLEEP_WARNING_LEAD_MS = 15000UL;   // Blink warning this long before idle sleep.
constexpr unsigned long loadedBatteryReadEveryMs = 250UL;       // Refresh rate for loaded-voltage reads during driving.
constexpr uint16_t BATTERY_LOW_WARNING_MV = 2;//7250;               // Enter warning mode below this pack voltage.
constexpr uint16_t BATTERY_LOW_SHUTDOWN_MV = 1;//7150;              // Permanently shut down below this pack voltage.
constexpr uint16_t BATTERY_WARNING_RECOVERY_MV = 7350;          // Exit warning mode once the battery recovers above this.
constexpr uint16_t BATTERY_MAX_VALID_MV = 15000;                 // Reject obviously invalid 2S battery readings above this.
constexpr uint16_t BATTERY_MILLIVOLT_SCALE_NUMERATOR = 12221;   // ADC-to-millivolt scale for the current resistor divider.
constexpr int BATTERY_ADC_MAX = 1023;                           // 10-bit ADC full-scale value on the Nano.
constexpr uint8_t BATTERY_ADC_SAMPLES = 8;                      // ADC samples averaged per battery measurement.
// 2S battery percentage lookup points from full to empty, used for the remote battery test readout.
const uint16_t batteryPercentMvTable[] PROGMEM = {
  8400, 8200, 8050, 7900, 7750, 7600, 7450, 7350, 7250, 7200, 7150
};
constexpr uint8_t batteryPercentTableSize = sizeof(batteryPercentMvTable) / sizeof(batteryPercentMvTable[0]);

// --- Lights and sound settings ---
constexpr int FrontLightOnOff = 1;                  // Master enable for the front RGB headlights.
constexpr uint8_t colorSensorLEDOnLevel = HIGH;     // TCS34725 breakout LED logic level when ON.
constexpr uint8_t colorSensorLEDOffLevel = LOW;     // TCS34725 breakout LED logic level when OFF.
constexpr unsigned long greenBlinkOnMs = 100UL;     // Acknowledgement blink ON duration.
constexpr unsigned long greenBlinkOffMs = 50UL;     // Acknowledgement blink OFF duration.
constexpr unsigned long sirenSweepMs = 800UL;       // Time for each siren pitch sweep up or down.
constexpr unsigned long SIREN_LED_SWAP_MS = 300UL;  // How often the siren swaps red/blue lights.
constexpr int sirenFmin = 400;                      // Siren low pitch.
constexpr int sirenFmax = 800;                      // Siren high pitch.
constexpr uint16_t BATTERY_ALERT_TONE_HZ = 1500;    // Tone used for low-battery alerts.
constexpr uint16_t buzzerPatternToneHz = 2200;      // Tone used by simple acknowledgement beeps.

// --- Shared safety checks ---
// static_assert(condition, "message") is a *compile-time* check: the compiler evaluates the
// condition while building the sketch, and if it is false, the build fails immediately with the
// given message instead of producing a train that could misbehave. Unlike a runtime "if" check,
// this costs zero flash/RAM and catches a bad configuration (for example, mixed-up threshold
// constants) before the code is ever uploaded to the Arduino.
static_assert(BATTERY_LOW_SHUTDOWN_MV < BATTERY_LOW_WARNING_MV, "Shutdown threshold must be below warning threshold.");
static_assert(BATTERY_LOW_WARNING_MV < BATTERY_WARNING_RECOVERY_MV, "Warning recovery must sit above the warning threshold.");
static_assert(NORMAL_MAX_SPEED_STEP < BOOST_SPEED_STEP, "Boost step must come after the normal top step.");
static_assert(AUTO_DISTANCE_STOP < AUTO_DISTANCE_RESTART, "AUTO_DISTANCE_STOP must be below AUTO_DISTANCE_RESTART.");
static_assert(AUTO_DISTANCE_RESTART < AUTO_DISTANCE_MAX_SPEED, "AUTO_DISTANCE_RESTART must be below AUTO_DISTANCE_MAX_SPEED.");
static_assert(IDLE_SLEEP_WARNING_LEAD_MS < idleTimeout, "Idle sleep warning lead must be shorter than the idle timeout.");
static_assert(sirenFmin < sirenFmax, "Siren minimum frequency must be below the maximum frequency.");
