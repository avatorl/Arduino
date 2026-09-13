#pragma once
// "#pragma once" tells the compiler to only process this header file one time, even if it gets
// included from multiple .ino tabs. Without it, re-including the same file could redefine the
// same constants twice and fail to compile.

// ############################################################################
// # USER SETTINGS SHARED BY MULTIPLE MODULES                                 #
// ############################################################################

// === IR RECEIVER CONFIGURATION ==================================================================

  // These "#define NAME" lines (with no value) are feature-switch macros: they must be defined
  // *before* <IRremote.hpp> is included (see the #include order in arduino-locomotive.ino) because the
  // IRremote library reads them at compile time to decide which hardware timer to use and which
  // remote-control protocols to build support for. Arduino's Timer1 and Timer2 are internal hardware
  // counters shared by several features (PWM output, tone(), IRremote's timing); telling IRremote to
  // use Timer1 keeps it out of Timer2's way. EXCLUDE_* macros remove unused remote-control protocol
  // decoders to save flash memory, since this train only needs the NEC protocol used by the remote.
  #define IR_USE_AVR_TIMER1 // IRremote uses Timer1 so Timer2 remains free for tone() on the passive buzzer.
  #define DECODE_NEC
  #define EXCLUDE_UNIVERSAL_PROTOCOLS
  #define EXCLUDE_EXOTIC_PROTOCOLS
  #define NO_LED_FEEDBACK_CODE
  #define RAW_BUFFER_LENGTH 100

// === EEPROM LOGGING AND SERIAL MONITOR OUTPUT CONFIGURATION =====================================

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
#define DEBUG_IR_REMOTE 0
#endif
#ifndef DEBUG_MOTOR
#define DEBUG_MOTOR 0
#endif
#ifndef DEBUG_COLOR_SENSOR
#define DEBUG_COLOR_SENSOR 0
#endif
#ifndef DEBUG_DISTANCE_SENSOR
#define DEBUG_DISTANCE_SENSOR 1
#endif
#ifndef DEBUG_TILT_SENSOR
#define DEBUG_TILT_SENSOR 0
#endif
#ifndef DEBUG_POWER_MANAGEMENT
#define DEBUG_POWER_MANAGEMENT 0
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

// === VOLTAGE MONITORING =========================================================================

// Permanent software shutdown policies. VIN shutdown protects a connected, discharged pack.
// VCC and meter-fault shutdowns stay disabled so USB-powered debugging cannot lock out the train.
#ifndef ENABLE_VIN_BATTERY_SHUTDOWN
#define ENABLE_VIN_BATTERY_SHUTDOWN 1
#endif
#ifndef ENABLE_VCC_POWER_SHUTDOWN
#define ENABLE_VCC_POWER_SHUTDOWN 1
#endif
#ifndef ENABLE_OVERVOLTAGE_POWER_SHUTDOWN
#define ENABLE_OVERVOLTAGE_POWER_SHUTDOWN 1
#endif

// === I2C ADDRESSES ==============================================================================

// The I2C bus is shared by multiple devices. Each device has a unique 7-bit address.
constexpr uint8_t mcp23008Address = 0x20; // (A0, A1, A2 pulled LOW by default)
// constexpr uint8_t tcs34725Address = 0x29; // Reference only: Adafruit_TCS34725 uses the fixed default 0x29 address.
constexpr uint8_t distanceSensorAddress = 0x2A; // Distance sensor = 0x2A (changed from default 0x29 using XSHUT pin)
// VL53L1X_ULD follows ST's convention and accepts the 8-bit I2C address byte.
constexpr uint8_t distanceSensorDefaultAddress8Bit = 0x52;
constexpr uint8_t distanceSensorAddress8Bit = distanceSensorAddress << 1;

// === ARDUINO PIN MAPPING ========================================================================

// These are the main hardware connections a builder may change before compiling.
// Keep each pin unique, and never move a function to D9 or D10 if it needs PWM.
// "constexpr" declares a typed constant that the compiler must be able to compute at compile time
// (unlike a plain "#define", which is just untyped text substitution done before compiling). Using
// constexpr here gives the compiler type-checking (for example, catching an accidental string
// where a number is expected) while still producing code just as small and fast as a #define.
constexpr int pinBatterySense = A0;
// A1 - unused
constexpr int pinColorSensorLED = A2;
constexpr int pinDistanceSensorXSHUT = A3;
// A4 - SDA (I2C data) shared by all I2C devices
// A5 - SCL (I2C clock) shared by all I2C devices
// A6 - unused
// A7 - unused
// D0 - RX (serial input) shared by USB serial and Bluetooth
// D1 - TX (serial output) shared by USB serial and Bluetooth
constexpr int pinIRReceiver = 2;
// D3 - unused - reserved for accelerometer interrupt (INT)
// D4 - unused
constexpr int pinMotor_IN1 = 5; // with PWM
constexpr int pinMotor_IN2 = 6; // with PWM
constexpr int pinMotorSleep = 7;
constexpr int pinMotorFault = 8;
constexpr int pinBuzzer = 9;
constexpr int pinTiltSensor = 12;
// D11 - unused
// D12 - unused
// D13 - unused (built-in LED)

// === MCP23008 EXPANDER PIN MAPPING ==============================================================

// Green LED
constexpr uint8_t ledGreenExpanderPin = 0;
// RGB LED 1
constexpr uint8_t led1RedExpanderPin = 1;
constexpr uint8_t led1GreenExpanderPin = 2;
constexpr uint8_t led1BlueExpanderPin = 3;
// RGB LED 2
constexpr uint8_t led2RedExpanderPin = 4;
constexpr uint8_t led2GreenExpanderPin = 5;
constexpr uint8_t led2BlueExpanderPin = 6;
// 2 x Red LEDs (on the same pin)
constexpr uint8_t ledRearRedExpanderPin = 7;

// === IR REMOTE MAPPING ==========================================================================

// Button codes for the NEC "Car MP3" handheld remote bundled with this build.
// Change these only if you swap to a different remote or remap train functions.
constexpr uint8_t buttonCHminus = 69;   // Speed down
constexpr uint8_t buttonCH = 70;        // Stop
constexpr uint8_t buttonCHplus = 71;    // Speed up
constexpr uint8_t buttonBackward = 68;  // Momentary backward jog
constexpr uint8_t buttonForward = 64;   // Momentary forward jog
constexpr uint8_t buttonPlayPause = 67; // Auto-speed toggle
constexpr uint8_t buttonMinus = 7;      // Ramped momentary backwa/rd jog
constexpr uint8_t buttonPlus = 21;      // Ramped momentary forward jog
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
// + and - start at level 1, then reach the normal 6 V maximum while held; only CH+ enables boost.
constexpr unsigned long MOMENTARY_RAMP_DURATION_MS = 2000UL;

// === MOTOR AND DRIVE SETTINGS ===================================================================

// Manual speed steps are expressed as requested motor voltage, then converted to PWM at runtime
// using the current battery voltage. The first entry must stay 0 for "stopped".
constexpr unsigned long DIR_DELAY = 500UL;        // Coast time before reversing direction.
constexpr unsigned long BOOST_DURATION_MS = 15000UL; // How long level 4 boost may stay active.
constexpr unsigned long BOOST_COOLDOWN_MS = 45000UL; // Wait time before boost may be used again.
constexpr uint16_t MAX_SAFE_MOTOR_MV = 7500;         // Hard top voltage request the motor may ever see.
constexpr uint16_t NORMAL_MAX_MOTOR_MV = 6000;       // Normal top voltage outside boost mode.
constexpr uint8_t NORMAL_MAX_SPEED_STEP = 3;         // Highest regular manual step.
constexpr uint8_t BOOST_SPEED_STEP = 4;              // Extra manual step reserved for boost.
constexpr uint16_t voltageSteps[] = { 0, 3500, 4500, 6000, 7500 }; // Requested motor mV for steps 0..4.
constexpr int rampStep = 5;                          // PWM change per auto-speed ramp update.
constexpr unsigned long rampDelay = 10UL;            // Delay between ramp steps in auto mode.
// Median of 3 keeps single-sample glitches out while reacting one full sample sooner than a
// median of 5 (about 60 ms faster at the 30 ms read period) - important for a short DUPLO train
// approaching an obstacle at speed.
constexpr int AUTO_SAMPLES_FOR_MEDIAN = 5;           // Distance samples kept for median filtering.
// STOP and RESTART form a hysteresis band (see motorVoltageFromDistance() in 20-motor.ino):
// the train stops when an obstacle comes closer than STOP and will not move again until the
// obstacle has cleared past RESTART. The gap prevents rapid stop/start oscillation when an
// obstacle sits right at the boundary.
constexpr int AUTO_DISTANCE_STOP = 10;                // Stop auto drive when obstacle is closer than this (cm).
constexpr int AUTO_DISTANCE_RESTART = 12;            // Start moving again once obstacle clears this distance (cm).
constexpr int AUTO_DISTANCE_MAX_SPEED = 80;          // Distance at which auto mode may request full normal speed (cm).
constexpr int AUTO_DISTANCE_MIN_SPEED = 30;          // Distance at which auto mode slows down to the minimal speed (cm).

// === COLOR SENSOR SETTINGS ======================================================================

// Defining required data structures --------------------------------------------------------------

// Headlight and status color palette.
// "enum class" defines a small, named set of allowed values (Off, Red, Green, ...) instead of
// using plain numbers. Compared to a plain "enum", "enum class" values must always be written as
// RgbColor::Red (not just Red), which avoids accidentally mixing up unrelated enums that happen to
// share a value name. ": uint8_t" tells the compiler to store each value in a single byte instead
// of the default int size, saving a little RAM/flash since there are only 8 colors here.
enum class RgbColor : uint8_t {
  Off = 0,
  Red,
  Green,
  Blue,
  Yellow,
  Cyan,
  Magenta,
  White
};

enum TrackMarkerClass : uint8_t {
  MarkerUnknown = 0,
  MarkerWhite,
  MarkerBlue,
  MarkerGreen,
  MarkerMagenta,
  MarkerYellow,
  MarkerRed,
  MarkerClassCount
};

// Each marker class selects the temporary headlight color shown when that marker is confirmed.
// MarkerUnknown maps to Off so unknown readings never produce visual feedback.
const RgbColor markerFeedbackColors[] PROGMEM = {
  RgbColor::Off,
  RgbColor::White,
  RgbColor::Blue,
  RgbColor::Green,
  RgbColor::Magenta,
  RgbColor::Yellow,
  RgbColor::Red
};
constexpr uint8_t markerFeedbackColorCount =
  sizeof(markerFeedbackColors) / sizeof(markerFeedbackColors[0]);

struct PrototypeRgb {
  uint16_t r;
  uint16_t g;
  uint16_t b;
};

struct MarkerClusterDefinition {
  TrackMarkerClass markerClass;
  PrototypeRgb center;
  uint16_t maxDistance;
  uint16_t minClearThreshold;
};

// Color sensor calibration -----------------------------------------------------------------------

// The sensor uses 24 ms integration (exposure) and 4x gain. This polling interval is slightly
// longer so each Adafruit getRawData() call can collect a fresh reading.
constexpr unsigned long colorSensorReadEveryMs = 30UL; // Time between color-sensor reads.
// A marker action runs after this many consecutive readings of the same known color.
constexpr uint8_t colorMarkerConfirmationSamples = 2;
// The confirmed marker is cleared after this many consecutive unknown readings.
constexpr uint8_t colorMarkerLeaveSamples = 2;
// Ignore green readings briefly after reversing so the same physical marker
// cannot immediately reverse the train again as it travels back over it.
constexpr unsigned long colorGreenMarkerCooldownMs = 1500UL;
// Controls how long confirmed-marker visual feedback stays visible; sensing continues meanwhile.
constexpr unsigned long colorMarkerFeedbackDurationMs = 1000UL;

// Readings above this unchanged clear-channel limit are rejected as sensor saturation.
constexpr uint16_t colorSaturationClearThreshold = 60000;

// White balance ----------------------------------------------------------------------------------

// Put the locomotive on a sheet of white paper (high quality white paper, e.g. incjet photo paper)
// Scan white paper with DEBUG_COLOR_SENSOR = 1 and adjust multipliers to achieve { 333, 333, 333 } as close as possible

constexpr float whiteBalanceRedGain = 1.00f;;          // Red-channel calibration multiplier.
constexpr float whiteBalanceGreenGain = 1.46f;        // Green-channel calibration multiplier.
constexpr float whiteBalanceBlueGain = 2.45f;         // Blue-channel calibration multiplier.

// Known colors -----------------------------------------------------------------------------------

// Each entry is a known track-marker color center plus its allowed matching radius.
// Format: { MarkerColor, { R, G, B }, Radius, minClearThreshold }. RGB is normalized to about
// 1000 total; integer rounding can make the total differ slightly.
// A cluster is considered only when the clear channel reaches its own minClearThreshold below.
// Put the locomorive on a track with an action brick (original or 3D-printed), then
// Scan color the brick with DEBUG_COLOR_SENSOR = 1 and copy { R, G, B } values from serial monitor
// Then adjust Radius and minClearThreshold
const MarkerClusterDefinition markerClusters[] PROGMEM = {
  { MarkerWhite, { 315, 364, 321 }, 30, 3000 }, // printed
  { MarkerBlue, { 180, 273, 547 } , 60, 750 }, // printed
  { MarkerGreen, { 177, 501, 321 } , 60, 500 }, // original
  { MarkerGreen, { 197, 515, 288 } , 60, 750 }, // printed
  { MarkerMagenta, { 455, 201, 344 }, 60, 750 }, // 
  { MarkerRed,  { 612, 174, 214 }, 60, 750 }, // printed
  { MarkerYellow, { 424, 373, 203 }, 60, 2000 }, // 
};
constexpr uint8_t markerClusterCount = sizeof(markerClusters) / sizeof(markerClusters[0]);

// === DISTANCE SENSOR SETTINGS ===================================================================

// Values calibrated for this VL53L1X in time-of-flight-1a. The reference
// sketch stores the offset as raw uint16 value 63491; that is -2045 as the
// signed value the ULD API expects. Recalibrate before changing either value.
constexpr int16_t distanceTofOffsetMm = -2045;
constexpr uint16_t distanceTofXtalkCps = 108;
// Long mode and a 50 ms timing budget maintain a valid signal through the
// narrowed 4 x 4 cone better than the earlier short/20 ms profile.
constexpr uint16_t distanceTofTimingBudgetMs = 50;
constexpr uint32_t distanceTofInterMeasurementMs = 50UL;
constexpr unsigned long tofReadEveryMs = distanceTofInterMeasurementMs; // How often the sketch consumes a ToF reading.
// The verified standalone setup measured this sensor about 30 mm short below
// 500 mm. Apply that correction only in its calibrated near-range region.
constexpr uint16_t distanceTofNearRangeCorrectionLimitMm = 500;
constexpr uint16_t distanceTofNearRangeOffsetMm = 30;
// The VL53L1X normally sees a ~27-degree cone through its complete 16 x 16
// SPAD array. Restricting the measurement to this centred 4 x 4 region narrows
// the cone to about 15 degrees, rejecting nearby track and body reflections.
constexpr uint8_t distanceTofRoiWidthSpads = 4;
constexpr uint8_t distanceTofRoiHeightSpads = 4;
// The 4 x 4 measurement window is selected from the VL53L1X's full 16 x 16
// SPAD matrix. SPAD IDs are not arranged numerically left-to-right or
// top-to-bottom, so do not infer a physical offset from an ID increment:
//
//        col:   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
// row  0:    128 136 144 152 160 168 176 184 192 200 208 216 224 232 240 248
// row  1:    129 137 145 153 161 169 177 185 193 201 209 217 225 233 241 249
// row  2:    130 138 146 154 162 170 178 186 194 202 210 218 226 234 242 250
// row  3:    131 139 147 155 163 171 179 187 195 203 211 219 227 235 243 251
// row  4:    132 140 148 156 164 172 180 188 196 204 212 220 228 236 244 252
// row  5:    133 141 149 157 165 173 181 189 197 205 213 221 229 237 245 253
// row  6:    134 142 150 158 166 174 182 190 198 206 214 222 230 238 246 254
// row  7:    135 143 151 159 167 175 183 191 199 207 215 223 231 239 247 255
// row  8:    127 119 111 103  95  87  79  71  63  55  47  39  31  23  15   7
// row  9:    126 118 110 102  94  86  78  70  62  54  46  38  30  22  14   6
// row 10:    125 117 109 101  93  85  77  69  61  53  45  37  29  21  13   5
// row 11:    124 116 108 100  92  84  76  68  60  52  44  36  28  20  12   4
// row 12:    123 115 107  99  91  83  75  67  59  51  43  35  27  19  11   3
// row 13:    122 114 106  98  90  82  74  66  58  50  42  34  26  18  10   2
// row 14:    121 113 105  97  89  81  73  65  57  49  41  33  25  17   9   1
// row 15:    120 112 104  96  88  80  72  64  56  48  40  32  24  16   8   0
//
// Factory-calibrated optical center read from this specific VL53L1X module.
// ST specifies a possible -2 to +2 SPAD variation on each axis. Update this
// value if the sensor is replaced, using the reported debug value from a
// factory-center read.
constexpr uint8_t distanceTofFactoryRoiCenterSpad = 199;
// Choose the desired 4 x 4 ROI centre from the matrix above. For reference,
// 197 is two matrix rows above factory center 199; the physical up/down
// direction depends on breakout orientation, so verify it with a
// target-position test. For an even 4 x 4 ROI, use the upper-right SPAD at
// the desired centre point. This setting changes where the sensor looks, not
// any drive threshold.
constexpr uint8_t distanceTofRoiCenterSpad = 199;
constexpr unsigned long tofFailureGraceMs = 250UL;     // Keep using the last good reading for this long before declaring a fault.
// If the sensor never delivers a single valid reading within this time after ranging starts,
// something is wrong (loose wire, dead sensor) and a distance fault is latched instead of the
// train waiting forever with no obstacle protection.
constexpr unsigned long tofStartupGraceMs = 1000UL;

// Tilt-sensor debounce. Increase if the sensor chatters, decrease if stop detection feels slow.
constexpr unsigned long TILT_STABLE_MS = 1000UL;
constexpr unsigned long TILT_QUIET_MS = 500UL;

// === POWER MANAGEMENT SETTINGS ==================================================================

constexpr unsigned long BATTERY_CHECK_INTERVAL_MS = 5000UL;    // Time between parked battery-health checks.
constexpr unsigned long VCC_CHECK_INTERVAL_MS = 1000UL;          // Check the Nano 5V rail this often during normal operation.
// After the train idles and enters sleep, it gives a short visual “I’m sleeping but powered” indication every IDLE_SLEEP_HEARTBEAT_MS seconds. The signal is a short IDLE_SLEEP_HEARTBEAT_ON_MS ms red flash at the rear of the locomotive. The front RGB lights and green LED remain off.
constexpr unsigned long IDLE_SLEEP_HEARTBEAT_MS = 8000UL;      // Sleep heartbeat cycle while idling (use 8-second intervals)
constexpr unsigned long IDLE_SLEEP_HEARTBEAT_ON_MS = 100UL;     // Heartbeat pulse ON time.
// If there were no IR remote activity, the train will enter idle sleep after this timeout.
constexpr unsigned long idleTimeout = 5UL * 60UL * 1000UL;      // Inactivity time before entering idle sleep.
constexpr unsigned long IDLE_SLEEP_WARNING_LEAD_MS = 15000UL;   // Blink warning this long before idle sleep.

// --- Battery voltage thresholds ---
// VCC (5V rail) monitoring and shutdown thresholds.
constexpr uint16_t VCC_LOW_SHUTDOWN_MV = 4600;                  // Protect the 16MHz Nano and 5V peripherals before VCC reaches 4.5V.
// KAmod I2C Mini Out8 module leaks small current from it's V+ line (Arduino +5V rail) to it's VCC line (Arduino VIN) when battery pack is disconnected (power off) but the Arduino is still powered via USB or another source. This can cause false readings on the battery voltage measurement during debugging.
constexpr uint16_t BATTERY_IMPLAUSIBLE_MV = 5000;               // Below this on a 2S pack = implausible reading (e.g. due to KAmod I2C Mini Out8 leakage); reject, do not count as low.
constexpr uint16_t VIN_BATTERY_SHUTDOWN_MAX_MV = 6600;          // Discharged 2S pack upper bound for permanent shutdown.
constexpr uint16_t BATTERY_LOW_WARNING_MV = 6630;               // Enter warning mode below this pack voltage.
constexpr uint16_t BATTERY_WARNING_RECOVERY_MV = 6700;          // Exit warning mode once the battery recovers above this.
// A healthy 2S 18650 pack never exceeds 8.4 V (two cells x 4.2 V full charge). Anything measured
// above 8.5 V therefore means a genuine overvoltage or a broken/disconnected voltage divider, and
// the sketch latches a critical-overvoltage fault (see enterCriticalOvervoltage() in
// 50-power-management.ino). A saturated ADC (raw 1023 = full scale, about 12.2 V with the current
// divider) cannot be told apart from a broken meter, so the fault log reports both possibilities.
constexpr uint16_t BATTERY_MAX_VALID_MV = 8500;                 // Above this = overvoltage or broken meter (2S max is 8.4 V).

constexpr unsigned long BATTERY_WARNING_SIGNAL_MS = 3000UL;     // Length of the warning sound/light signal.
constexpr unsigned long BATTERY_WARNING_REPEAT_MS = 60000UL;    // How often warning mode reminds the user.
constexpr unsigned long BATTERY_SHUTDOWN_SIGNAL_MS = 10000UL;   // Length of the final shutdown signal.

constexpr uint8_t BATTERY_LOW_CONFIRMATION_COUNT = 3;           // Consecutive low battery samples required before warning/shutdown.
constexpr uint8_t VCC_LOW_CONFIRMATION_COUNT = 3;               // Consecutive low VCC samples required before shutdown.

// Arduino Nano internal 1.1V bandgap calibration in microvolts.
constexpr uint32_t VCC_BANDGAP_CALIBRATION_UV = 1100000UL;
// 100K and 10K resistor divider for battery voltage measurement (99K and 9.9k actual values), scale factor = (100K + 10K) / 10K = 11.
// 11 * 1.1V max reference voltage * 1.01 calibration factor = 12.221
constexpr uint16_t BATTERY_MILLIVOLT_SCALE_NUMERATOR = 12221;   // ADC-to-millivolt scale for the current resistor divider.
constexpr int BATTERY_ADC_MAX = 1023;                           // 10-bit ADC full-scale value on the Nano.
constexpr uint8_t BATTERY_ADC_SAMPLES = 8;                      // ADC samples averaged per battery measurement.

// === LIGHTS AND SOUNDS SETTINGS =================================================================

constexpr int FrontLightOnOff = 1;                  // Master enable for the front RGB headlights.
constexpr unsigned long greenBlinkOnMs = 100UL;     // Acknowledgement blink ON duration.
constexpr unsigned long greenBlinkOffMs = 50UL;     // Acknowledgement blink OFF duration.
constexpr unsigned long sirenSweepMs = 800UL;       // Time for each siren pitch sweep up or down.
constexpr unsigned long SIREN_LED_SWAP_MS = 300UL;  // How often the siren swaps red/blue lights.
constexpr int sirenFmin = 400;                      // Siren low pitch.
constexpr int sirenFmax = 800;                      // Siren high pitch.
constexpr uint16_t BATTERY_ALERT_TONE_HZ = 1500;    // Tone used for low-battery alerts.
constexpr uint16_t buzzerPatternToneHz = 2200;      // Tone used by simple acknowledgement beeps.

// === COMPILE TIME ASSERTION CHECKING ============================================================

// static_assert(condition, "message") is a *compile-time* check: the compiler evaluates the
// condition while building the sketch, and if it is false, the build fails immediately with the
// given message instead of producing a train that could misbehave. Unlike a runtime "if" check,
// this costs zero flash/RAM and catches a bad configuration (for example, mixed-up threshold
// constants) before the code is ever uploaded to the Arduino.

static_assert(VIN_BATTERY_SHUTDOWN_MAX_MV < BATTERY_LOW_WARNING_MV, "VIN shutdown range must sit below the warning threshold.");
// Guards against debug leftovers: a 2S lithium pack must never be discharged below ~6.0 V, so a
// warning threshold under 6000 mV can only be an accidental test value (this exact bug shipped
// once as "BATTERY_LOW_WARNING_MV = 2").
static_assert(BATTERY_LOW_WARNING_MV >= 6000, "Warning threshold below 6.0 V is unsafe for a 2S pack.");
static_assert(BATTERY_WARNING_RECOVERY_MV < BATTERY_MAX_VALID_MV, "Recovery threshold must be below the overvoltage limit.");
static_assert(BATTERY_LOW_WARNING_MV < BATTERY_WARNING_RECOVERY_MV, "Warning recovery must sit above the warning threshold.");
static_assert(VCC_CHECK_INTERVAL_MS > 0, "VCC check interval must be nonzero.");
static_assert(VCC_LOW_CONFIRMATION_COUNT > 0, "VCC low confirmation count must be nonzero.");
static_assert(BATTERY_LOW_CONFIRMATION_COUNT > 0, "Battery low confirmation count must be nonzero.");
static_assert(IDLE_SLEEP_HEARTBEAT_MS % 8000UL == 0, "Idle heartbeat period must be a whole number of 8-second sleep intervals.");
static_assert(BATTERY_IMPLAUSIBLE_MV < VIN_BATTERY_SHUTDOWN_MAX_MV, "Implausible-glitch floor must sit below the VIN shutdown range.");
static_assert(VCC_LOW_SHUTDOWN_MV > 4500, "VCC shutdown threshold must stay above the 16MHz ATmega328P minimum.");
static_assert(NORMAL_MAX_SPEED_STEP < BOOST_SPEED_STEP, "Boost step must come after the normal top step.");
static_assert(MOMENTARY_RAMP_DURATION_MS > 0, "Momentary ramp duration must be nonzero.");
static_assert(distanceTofTimingBudgetMs <= distanceTofInterMeasurementMs, "VL53L1X inter-measurement period must cover its timing budget.");
static_assert(distanceTofRoiWidthSpads >= 4 && distanceTofRoiWidthSpads <= 16, "VL53L1X ROI width must be 4 through 16 SPADs.");
static_assert(distanceTofRoiHeightSpads >= 4 && distanceTofRoiHeightSpads <= 16, "VL53L1X ROI height must be 4 through 16 SPADs.");
static_assert(AUTO_DISTANCE_STOP < AUTO_DISTANCE_RESTART, "AUTO_DISTANCE_STOP must be below AUTO_DISTANCE_RESTART.");
static_assert(AUTO_DISTANCE_RESTART < AUTO_DISTANCE_MAX_SPEED, "AUTO_DISTANCE_RESTART must be below AUTO_DISTANCE_MAX_SPEED.");
static_assert(AUTO_DISTANCE_RESTART <= AUTO_DISTANCE_MIN_SPEED, "AUTO_DISTANCE_MIN_SPEED must not be below AUTO_DISTANCE_RESTART.");
static_assert(AUTO_DISTANCE_MIN_SPEED < AUTO_DISTANCE_MAX_SPEED, "AUTO_DISTANCE_MIN_SPEED must be below AUTO_DISTANCE_MAX_SPEED.");
static_assert(IDLE_SLEEP_WARNING_LEAD_MS < idleTimeout, "Idle sleep warning lead must be shorter than the idle timeout.");
static_assert(sirenFmin < sirenFmax, "Siren minimum frequency must be below the maximum frequency.");
static_assert(markerFeedbackColorCount == MarkerClassCount, "Marker feedback colors must match the TrackMarkerClass entries.");