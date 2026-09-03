  #include "config.h"
  // Warning: config.h forces IRremote onto Timer1, so do not use analogWrite() on D9 or D10.
  
  // Library includes. Do not include full libraries for all sensors to avoid unnecessary flash and SRAM usage. Use Wire.h for I2C and implement only the register-level access needed for each sensor.
  // "#include" pastes the contents of another file in at this point before compiling. Wire.h, IRremote.hpp,
  // and LowPower.h are external libraries (code someone else wrote) that add ready-made functions for talking
  // over the I2C bus, decoding infrared remote signals, and putting the chip to sleep to save battery power.
  #include <Wire.h>         // I2C bus
  #include <IRremote.hpp>  // IR remote library
  #include <LowPower.h>    // Low power/sleep mode library
  #if ENABLE_EEPROM_LOGGING
  #include <EEPROM.h>       // Persistent boot-error log
  #endif


  #include "melodies.h"     // Load music patterns

  // ===============================================================================================
  // File description
  // ===============================================================================================
  // Main sketch file for Arduino DUPLO Train v2.
  // This file keeps the shared types, shared state, low-level helper classes, the mechanical tilt
  // switch handler, setup(), and loop().
  // Feature-specific function bodies are grouped into:
  //   - 10-ir-remote.ino
  //   - 20-motor.ino
  //   - 30-lights-and-sounds.ino
  //   - 41-color-sensor.ino
  //   - 42-distance-sensor-vl53l0x.ino
  //   - 43-distance-sensor-vl53l1x.ino
  //   - 43-accelerometer.ino
  //   - 50-power-management.ino

  // ===============================================================================================
  // Sketch Contents / Structure Guide
  // ===============================================================================================
  //  1. Shared enums, structs, forward declarations, and helper classes.
  //  2. Shared calibration data and runtime state used by multiple modules.
  //  3. setup() and loop().
  //
  // Named Type Index
  //   RgbColor                          - Headlight and status color palette.
  //   TrackMarkerClass                  - Track-marker classification labels.
  //   LedRoute                          - LED expander pin mapping helper.
  //   TrainLedMCP23008                  - MCP23008 LED expander helper.
  //   Dir                               - Motor direction state.

  // ===============================================================================================
  #define DEBUG_ANY (DEBUG_COLOR_SENSOR || DEBUG_DISTANCE_SENSOR || DEBUG_IR_REMOTE || DEBUG_TILT_SENSOR || DEBUG_ACCELEROMETER || DEBUG_POWER_MANAGEMENT || DEBUG_EEPROM || DEBUG_MOTOR || DEBUG_LEDS || DEBUG_SOUND)
  // DEBUG_ANY is true (non-zero) if at least one of the per-module DEBUG_* flags from config.h is
  // turned on. It is used just below to decide whether Serial (the USB debug connection) needs to
  // be started at all; if every debug flag is off, the code that would set up Serial is skipped
  // entirely, saving a little flash and startup time.

  #if ENABLE_EEPROM_LOGGING
  // ===============================================================================================
  // EEPROM event layout. The original boot/fault ring remains intact, and a second ring stores
  // low-battery warning/shutdown events with battery voltage and uptime.
  // ===============================================================================================
  // 0x00..0x01  boot counter      uint16_t (2 bytes), wraps 0..65535; serves as the per-boot sequence
  // 0x02        boot log head     uint8_t, next-write slot index (0..EEPROM_BOOT_LOG_SIZE-1)
  // 0x03..0x52  boot ring buffer  16 entries × 5 bytes each:
  //               bytes 0..1  boot_seq      — uint16_t (2 bytes) copy of counter at time of this boot
  //               byte 2      flags         — sensor error bitmask (0x00 = all OK)
  //               byte 3      battery       — pack voltage × 10 as uint8_t (e.g. 74 = 7.4 V); 0xFF = not measured or invalid (>8.5 V for 2S)
  //               byte 4      fault_count   — DRV8833 runtime fault count for this boot session (0..16)
  // 0x53        event log head    uint8_t, next-write slot index (0..EEPROM_EVENT_LOG_SIZE-1)
  // 0x54..0xD3  event ring buffer 16 entries × 8 bytes each:
  //               bytes 0..1  boot_seq      — uint16_t (2 bytes)
  //               byte 2      event_type    — 0x01 warning, 0x02 shutdown, 0x03 ToF fault, 0x04 critical overvoltage
  //               byte 3      battery       — pack voltage × 10 as uint8_t; 0xFF = not measured or invalid (>8.5 V for 2S)
  //               bytes 4..7  uptime_ms     — uint32_t milliseconds since setup()
  //
  // Error flag bit map:
  //   Bit 0  0x01  MCP23008 LED expander not detected
  //   Bit 1  0x02  TCS34725 color sensor not detected
  //   Bit 2  0x04  Distance sensor not detected
  //   Bit 3  0x08  MPU6050 accelerometer not detected
  const uint16_t EEPROM_ADDR_BOOT_COUNT = 0x00;
  const uint8_t  EEPROM_ADDR_LOG_HEAD   = 0x02;
  const uint8_t  EEPROM_ADDR_LOG_BASE   = 0x03;
  const uint8_t  EEPROM_BOOT_LOG_SIZE   = 16;   // 16 entries × 5 bytes = 80 bytes for the ring buffer
  const uint8_t  EEPROM_ENTRY_SIZE      = 5;    // 5 bytes per entry
  const uint8_t  EEPROM_ADDR_EVENT_HEAD = 0x53;
  const uint8_t  EEPROM_ADDR_EVENT_BASE = 0x54;
  const uint8_t  EEPROM_EVENT_LOG_SIZE  = 16;
  const uint8_t  EEPROM_EVENT_ENTRY_SIZE = 8;
  const uint8_t  MAX_FAULTS_PER_BOOT    = 16;   // Cap fault increments / EEPROM rewrites per power-on cycle
  const uint8_t  MAX_RUNTIME_EEPROM_WRITES_PER_BOOT = 16;
  const uint8_t  ERR_LED_EXPANDER = 0x01;
  const uint8_t  ERR_COLOR_SENSOR = 0x02;
  const uint8_t  ERR_DISTANCE_TOF = 0x04;
  const uint8_t  ERR_ACCELEROMETER = 0x08;
  const uint8_t  EEPROM_EVENT_WARNING  = 0x01;
  const uint8_t  EEPROM_EVENT_SHUTDOWN = 0x02;
  const uint8_t  EEPROM_EVENT_TOF_FAULT = 0x03;
  const uint8_t  EEPROM_EVENT_OVERVOLTAGE = 0x04;
  #endif

  #if DEBUG_ANY
  #define DBGBEGIN(...) \
    do { Serial.begin(__VA_ARGS__); } while (0)  // Initialize Serial
  #else
  #define DBGBEGIN(...)  // No operation
  #endif
  // DBGBEGIN(...) and all the DBG_xxx/DBGLN_xxx macros below are "conditional macros": depending on
  // whether the matching DEBUG_xxx flag in config.h is 1 or 0, the preprocessor swaps each call for
  // either a real Serial.print()/Serial.println() call, or for literally nothing at all. Because
  // this substitution happens before compiling, a disabled debug macro leaves no trace in the
  // compiled program - it doesn't just skip printing at runtime, it takes zero flash and zero time.
  // This is why the sketch calls DBG_MOTOR(...) etc. everywhere instead of Serial.print(...)
  // directly: it lets each module's logging be turned on/off independently without editing the code.
  // "..." and "__VA_ARGS__" make these "variadic macros", meaning they can accept any number of
  // arguments (just like Serial.print can take a string, a number, or a number with a base like HEX)
  // and forward all of them straight through to the real function.
  // The "do { ... } while (0)" wrapper is a common C/C++ trick that makes a multi-statement macro
  // behave exactly like a single statement, so it stays safe to use inside an "if" without braces.

  #if DEBUG_COLOR_SENSOR
  #define DBG_COLOR_SENSOR(...) Serial.print(__VA_ARGS__)
  #define DBGLN_COLOR_SENSOR(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_COLOR_SENSOR(...)
  #define DBGLN_COLOR_SENSOR(...)
  #endif

  #if DEBUG_DISTANCE_SENSOR
  #define DBG_DISTANCE_SENSOR(...) Serial.print(__VA_ARGS__)
  #define DBGLN_DISTANCE_SENSOR(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_DISTANCE_SENSOR(...)
  #define DBGLN_DISTANCE_SENSOR(...)
  #endif

  #if DEBUG_IR_REMOTE
  #define DBG_IR_REMOTE(...) Serial.print(__VA_ARGS__)
  #define DBGLN_IR_REMOTE(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_IR_REMOTE(...)
  #define DBGLN_IR_REMOTE(...)
  #endif

  #if DEBUG_TILT_SENSOR
  #define DBG_TILT_SENSOR(...) Serial.print(__VA_ARGS__)
  #define DBGLN_TILT_SENSOR(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_TILT_SENSOR(...)
  #define DBGLN_TILT_SENSOR(...)
  #endif

  #if DEBUG_ACCELEROMETER
  #define DBG_ACCELEROMETER(...) Serial.print(__VA_ARGS__)
  #define DBGLN_ACCELEROMETER(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_ACCELEROMETER(...)
  #define DBGLN_ACCELEROMETER(...)
  #endif

  #if DEBUG_POWER_MANAGEMENT
  #define DBG_POWER_MANAGEMENT(...) Serial.print(__VA_ARGS__)
  #define DBGLN_POWER_MANAGEMENT(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_POWER_MANAGEMENT(...)
  #define DBGLN_POWER_MANAGEMENT(...)
  #endif

  #if DEBUG_EEPROM
  #define DBG_EEPROM(...) Serial.print(__VA_ARGS__)
  #define DBGLN_EEPROM(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_EEPROM(...)
  #define DBGLN_EEPROM(...)
  #endif

  #if DEBUG_MOTOR
  #define DBG_MOTOR(...) Serial.print(__VA_ARGS__)
  #define DBGLN_MOTOR(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_MOTOR(...)
  #define DBGLN_MOTOR(...)
  #endif

  #if DEBUG_LEDS
  #define DBG_LEDS(...) Serial.print(__VA_ARGS__)
  #define DBGLN_LEDS(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_LEDS(...)
  #define DBGLN_LEDS(...)
  #endif

  #if DEBUG_SOUND
  #define DBG_SOUND(...) Serial.print(__VA_ARGS__)
  #define DBGLN_SOUND(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_SOUND(...)
  #define DBGLN_SOUND(...)
  #endif

  // ===============================================================================================

  // AVR-specific include for program memory storage
  // "#if defined(__AVR__)" only compiles the code inside when building for an AVR-family chip (like
  // the ATmega328P on the Arduino Nano). This keeps the sketch portable: <avr/pgmspace.h> gives
  // access to PROGMEM (explained near the melody/pattern data below) and <avr/wdt.h> gives access
  // to the hardware watchdog timer (explained near wdt_enable() in setup()); both are AVR-specific
  // features that would not exist on a different microcontroller family.
  #if defined(__AVR__)
  #include <avr/pgmspace.h>
  #include <avr/wdt.h>
  #endif

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

  // White-balanced RGBC sample.
  struct BalancedRgbs {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
  };

  // Battery warning/shutdown policy state. Inactivity sleep is handled separately.
  enum class BatteryState : uint8_t {
    Normal = 0,
    Warning,
    Shutdown
  };

  enum class ShutdownCause : uint8_t {
    None = 0,
    LowVin,
    LowVcc
  };

  // Visible/audible state reference:
  //   Normal stopped/forward/reverse: motor stopped/forward/reverse; front red/white/blue; no fault sound; command changes state.
  //   Normal auto: motor follows distance; green on and normal drive color; no fault sound; command disables auto.
  //   Low-battery warning: motor and restricted features off; front/green off, rear red during alert; 1500 Hz alert repeats; recovers above BATTERY_WARNING_RECOVERY_MV.
  //   Low-battery shutdown: motor and sensors disabled; rear red and 1500 Hz alert for BATTERY_SHUTDOWN_SIGNAL_MS, then all outputs off; power cycle only.
  //   Critical overvoltage: motor disabled; front RGB, green, and sensor LEDs off; rear red steady; one pattern_criticalOvervoltage alert; power cycle only.
  //   DRV8833 fault: motor disabled; front red, green off; pattern_batteryWarn; a motor command re-arms after the hardware fault clears.
  //   Distance-sensor fault: motor stopped and auto disabled; front red; pattern_batteryWarn; restart/rearm required.
  //   Tilt alarm: motor stopped and auto disabled; front red; pattern_tiltBeep; clears when the tilt input returns idle.
  //   Idle-sleep warning: normal outputs remain active with a green blink; no distinct sound; activity cancels the timeout.
  //   Idle sleep: motor and normal indicators off; pattern_descend before sleep and rear-red heartbeat while sleeping; IR wake restores normal operation.
  //   Boot sensor errors: no dedicated visible/audible indication; recorded in the EEPROM boot error flags when logging is enabled.

  // LED expander pin mapping helper.
  struct LedRoute;

  // Forward declarations are grouped here so the high-level flow can stay readable below.
  // Most functions are implemented in the same order as the contents guide above.
  // A "forward declaration" tells the compiler a function's name/parameters/return type exist
  // before showing the actual body, so code earlier in the file (or in other .ino tabs compiled
  // earlier) can already call it. Arduino normally generates these automatically, but this sketch
  // is large enough, and uses enough default-argument overloads, that some are written by hand here
  // to avoid the auto-generator getting confused (see the note on playPattern below).
  void SetRGBLightColor(RgbColor color, int led = 0);
  void SetRGBColor(RgbColor color, int led = 0);

  void playToneSequenceRaw(const int* seqFD, int pairCount, bool loopPlayback, bool isProgmem = false);

  // PROGMEM wrapper (for PROGMEM melodies)
  // Template wrapper for PROGMEM melodies.
  // "template<size_t N>" makes this function work with an array of any length N: the compiler
  // generates a separate version of the function for each different array size it's actually called
  // with, figuring out N automatically from the array passed in. "static_assert" then runs a
  // compile-time check (not a runtime one) that N is even, since melody data is stored as
  // [frequency, duration, frequency, duration, ...] pairs; if someone passes a melody array with an
  // odd length, the build fails immediately with a clear error instead of behaving oddly at runtime.
  template<size_t N>
  void playToneSequence_P(const int16_t (&seqFD)[N], bool loopPlayback = false) {
    static_assert(N % 2 == 0, "Melody array must have even length [freq,dur,...]");
    playToneSequenceRaw((const int*)seqFD, (int)(N / 2), loopPlayback, true);
  }

  void updateMelody();
  void stopMelody();
  // playPattern's forward declaration carries the "= false" default argument. It is declared here
  // (rather than relying on Arduino's auto-generated prototype) because the auto-generator does not
  // reliably support default arguments; without this manual declaration, files compiled before
  // 30-lights-and-sounds.ino (like 10-ir-remote.ino) would fail to call playPattern(pattern) with
  // just one argument. C++ also only allows the default value to be written once across the whole
  // program, so the actual function body in 30-lights-and-sounds.ino does not repeat "= false".
  void playPattern(const uint16_t* pattern, bool bypassBatteryGate = false);
  void initTrainLedHardware();
  void initSensorHardware();
  void initColorSensorHardware();
  void initBatteryVoltageMeterHardware();
  uint16_t getVccVoltageMv();
  bool confirmLowVccAtStartup();
  void updateVccGuard();
  void powerDownColorSensorCore();
  void setColorSensorEnabled(bool enabled);
  void updateColorSensor();
  void refreshDriveLights();
  uint16_t getBatteryVoltageSettledForStatus();
  void updateBatteryGuard();
  bool isCriticalOvervoltage(uint16_t voltageMv);
  void enterCriticalOvervoltage(uint16_t voltageMv, uint16_t averageRaw);
  void applyCriticalOvervoltageOutputs();
  void updateGreenBlink();
  void updateMotorReverseCooldown();
  void initDistanceSensorHardware();
  void initAccelerometerHardware();
  void updateAccelerometerSafety();
  // sleepAccelerometer()/wakeAccelerometer() live in 43-accelerometer.ino and are declared here so
  // earlier-compiled tabs (30-lights-and-sounds.ino, 50-power-management.ino) can park the MPU6050
  // in its low-power sleep mode during idle sleep and permanent shutdown.
  void sleepAccelerometer();
  void wakeAccelerometer();
  bool startDistanceSensorRanging();
  // Starts or stops distance-sensor continuous ranging without a full re-init. Ranging is only needed while
  // auto-distance mode is active, so stopping it saves power and I2C traffic the rest of the time.
  void setDistanceSensorRangingActive(bool active);
  int getDistanceReading();
  uint8_t irReceive();
  bool tryPlayMelodyForButton(uint8_t code);
  bool isMotorControlCommand(uint8_t code);
  void translateIR();
  void exitAutoDistanceMode(bool clearIndicator = true);
  // Clears the auto-mode obstacle stop latch (hysteresis state in 20-motor.ino) so a freshly
  // enabled auto session never starts from a stale "stopped by obstacle" decision.
  void resetAutoDistanceState();
  void stopAndResetStepSelection(bool resetDirection = false);
  void cancelJog();
  int get2SBatteryPercent(uint16_t voltageMv);
  uint8_t classifyTrackMarkerColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
  void handleTrackMarkerAction(uint8_t markerClass);
  void SetRearRedLight(bool enabled);
  void enterBatteryWarning();
  void exitBatteryWarning();
  void enterBatteryShutdown(bool startupLockout = false, ShutdownCause cause = ShutdownCause::LowVin);
  void updateBatterySignal();
  void clearBuzzerPattern();
  bool isBuzzerPatternPlaying();
  void waitForPatternPlayback(unsigned long timeoutMs);
  void performPermanentShutdown();
  bool captureWakeIrCommand(unsigned long timeoutMs);
  bool areUserSoundsAllowed();
  bool areRgbLightsAllowed();
  bool isGreenIndicatorAllowed();
  bool isColorSensorAllowed();
  bool isAutoDistanceAllowed();
  bool isBoostAllowed();
  bool canEnterIdleSleep();
  bool isWarningModeCommandAllowed(uint8_t code);
  #if ENABLE_EEPROM_LOGGING && DEBUG_EEPROM
  void dumpEepromDebugSummary();
  #endif
  #if DEBUG_COLOR_SENSOR
  const __FlashStringHelper* trackMarkerLabel(uint8_t markerClass);
  #endif

  // Sensor-owned shared state is defined in 41-color-sensor.ino and the selected distance-sensor backend and
  // declared here so other modules can use it.
  extern bool colorSensorDetected;
  extern uint8_t Distance;
  extern bool distanceTofDetected;
  extern bool distanceTofFaultLatched;

  // Motor directions
  // Motor direction state.
  enum class Dir : uint8_t { Stop = 0,
                            Forward = 1,
                            Backward = 2 };

  void setMotor(Dir dir, int speed);
  void JogDrive(Dir dir);


  // ================================================================================================
  // Arduino Pin Mapping
  // ================================================================================================
  
  // Pin usage summary (actual Nano-side roles in this sketch), ordered by board pin:
  //   D0  -> Hardware UART RX shared with USB serial and DEBUG output; avoid other peripherals.
  //   D1  -> Hardware UART TX shared with USB serial and DEBUG output; avoid other peripherals.
  //   D2  -> IR receiver input; also used as the wake interrupt source from sleep.
  //   D3  -> Free digital pin.
  //   D4  -> Free digital pin.
  //   D5  -> DRV8833 IN1 motor drive PWM/direction output.
  //   D6  -> DRV8833 IN2 motor drive PWM/direction output.
  //   D7  -> DRV8833 nSLEEP output, HIGH = enabled, LOW = sleep.
  //   D8  -> DRV8833 nFAULT input with internal pull-up; active LOW.
  //   D9  -> Tilt sensor input with internal pull-up; switch is closed to GND while the train is
  //          upright (reads LOW) and opens when tilted (pull-up takes the pin HIGH = alarm).
  //   D10 -> Free digital-only pin; do not use analogWrite because IRremote owns Timer1.
  //   D11 -> Free general-purpose digital pin if SPI is not needed; also SPI MOSI.
  //   D12 -> Passive buzzer output driven by tone(); also SPI MISO.
  //   D13 -> Free digital pin; also tied to the Nano onboard LED and SPI SCK.
  //   A0  -> Battery voltage sense analog input.
  //   A1  -> Free analog-input-only pin.
  //   A2  -> TCS34725 breakout LED control output (digital-capable analog pin).
  //   A3  -> Distance-sensor XSHUT output for sensor reset / I2C address setup.
  //   A4  -> I2C SDA shared by the MCP23008, TCS34725, distance sensor, and MPU6050.
  //   A5  -> I2C SCL shared by the MCP23008, TCS34725, distance sensor, and MPU6050.
  //   A6  -> Free analog-input-only pin.
  //   A7  -> Free analog-input-only pin.
  // SPI note: SPI is not used in this sketch, but an SPI peripheral would conflict with the buzzer
  // on D12.
  
  // The unused DRV8833 channel's IN3 and IN4 pins must be physically tied to GND on this board
  // revision. They are not driven by the Nano, so never leave them floating.
  // LED channels are driven from the MCP23008 expander outputs through the MOSFET stage.
  // The exact LED polarity depends on the module wiring; the sketch treats each channel as
  // an on/off output and does not use PWM for the RGB LEDs.
  // LED expander pin mapping helper.
  struct LedRoute {
    uint8_t expanderPin;
  };

  // MCP23008 LED expander helper.
  // This struct acts like a tiny hand-written driver "class" for the MCP23008 I2C GPIO-expander
  // chip: instead of pulling in a full third-party library, it directly reads/writes the two
  // registers this sketch actually needs, to save flash and RAM. I2C is a two-wire bus (SDA for
  // data, SCL for clock) that lets the Nano talk to multiple chips using only 2 pins, each chip
  // identified by a 7-bit "address". Wire.beginTransmission(address) starts a message to that chip,
  // Wire.write(...) queues bytes to send, and Wire.endTransmission() actually sends them and returns
  // 0 on success (any other value means the chip did not acknowledge, e.g. it's not wired up).
  struct TrainLedMCP23008 {
    // MCP23008 register addresses used by the expander helper.
    // "static const" inside a struct/class means this value belongs to the type itself (shared by
    // all instances) rather than being duplicated in every TrainLedMCP23008 variable - here there's
    // only one instance anyway, but it also documents that these are fixed chip constants, not
    // per-instance data.
    static const uint8_t RegisterIodir = 0x00;
    static const uint8_t RegisterGpio = 0x09;

    // Cached I2C address and shadow copies of the MCP23008 output state.
    // A "shadow register" is a local copy in RAM that mirrors what we last told the real chip. The
    // MCP23008's registers can only be written, not read back cheaply here, so the code keeps its
    // own copy (iodirShadow, gpioShadow) and updates individual bits in it before writing the whole
    // byte back over I2C. This lets digitalWrite() below change just one LED pin without disturbing
    // the others, since I2C register writes replace the entire byte each time.
    uint8_t address = 0;
    uint8_t iodirShadow = 0xFF;
    uint8_t gpioShadow = 0x00;

    // Initialize the MCP23008 over I2C and seed the shadow registers.
    // Note: the shared I2C bus itself (Wire.begin, clock speed, timeout, bus-clear) is configured
    // once in initI2cBus() from setup(), before any device driver runs - not here.
    bool begin_I2C(uint8_t i2cAddress) {
      address = i2cAddress;
      iodirShadow = 0xFF;
      gpioShadow = 0x00;
      if (!probe()) return false;
      if (!writeRegister(RegisterGpio, gpioShadow)) return false;
      return writeRegister(RegisterIodir, iodirShadow);
    }

    // Configure a GPIO pin as an output in the shadow IODIR register.
    // "iodirShadow &= (uint8_t)~(1 << pin);" is a classic embedded bit-manipulation idiom:
    // "1 << pin" makes a byte with only bit number "pin" set (e.g. pin=2 -> 0b00000100), "~" flips
    // every bit (0b11111011), and "&=" clears just that one bit in iodirShadow while leaving every
    // other bit untouched. On the MCP23008, an IODIR bit of 0 means "this pin is an output", so this
    // clears the bit for the pin being configured.
    void pinMode(uint8_t pin, uint8_t mode) {
      if (mode != OUTPUT || pin > 7) return;
      iodirShadow &= (uint8_t)~(1 << pin);
      writeRegister(RegisterIodir, iodirShadow);
    }

    // Drive one expander GPIO high or low and mirror it in the shadow GPIO register.
    // Here "newShadow |= (1 << pin)" sets one bit to turn the output HIGH, and the "&= ~(1 << pin)"
    // branch clears it for LOW, using the same single-bit bitmask trick as pinMode() above. The
    // "if (newShadow == gpioShadow) return;" check skips the I2C write entirely when nothing would
    // actually change, saving bus traffic/time.
    void digitalWrite(uint8_t pin, uint8_t value) {
      if (pin > 7) return;
      uint8_t newShadow = gpioShadow;
      if (value == HIGH) {
        newShadow |= (1 << pin);
      } else {
        newShadow &= (uint8_t)~(1 << pin);
      }
      if (newShadow == gpioShadow) return;
      gpioShadow = newShadow;
      writeRegister(RegisterGpio, gpioShadow);
    }

   private:
    // Probe whether the MCP23008 responds on the configured I2C address.
    bool probe() {
      Wire.beginTransmission(address);
      return Wire.endTransmission() == 0;
    }

    // Write one MCP23008 register over I2C.
    bool writeRegister(uint8_t reg, uint8_t value) {
      Wire.beginTransmission(address);
      Wire.write(reg);
      Wire.write(value);
      return Wire.endTransmission() == 0;
    }
  };

  // Train LEDs are routed through an MCP23008 on the shared I2C bus.
  // MCP23X08 / color sensor wiring summary:
  //   Nano A4/SDA -> MCP23008 SDA and TCS34725 SDA
  //   Nano A5/SCL -> MCP23008 SCL and TCS34725 SCL
  //   Nano 5V     -> MCP23008 VCC and TCS34725 VIN/VCC
  //   Nano GND    -> MCP23008 GND and TCS34725 GND
  //   MCP23008 address pins A0/A1/A2 -> GND, giving I2C address 0x20
  //   MCP23008 RESET -> pull HIGH (10 kOhm to 5V preferred; direct tie to 5V also works)
  //   MCP23008 VDD/GND -> place a 100 nF ceramic decoupling capacitor close to the chip
  //   SDA/SCL     -> need one effective I2C pull-up pair on the whole bus (typically 4.7 kOhm to 5V)
  //                  add pull-ups only if the connected breakouts/modules do not already provide them
  //   MCP23008 INT -> optional in this revision; leave unconnected if not used
  //   Nano A2     -> TCS34725 LED pin for breakout illumination control
  //   TCS34725 INT stays unconnected in this revision
  //   TCS34725 LED pin is assumed active-HIGH in this sketch; swap the on/off levels below if your
  //   breakout uses active-LOW LED control instead
  // MCP23008 GPIO allocation:
  //   MCP GP0     -> RGB LED #1 Red transistor base resistor
  //   MCP GP1     -> RGB LED #1 Green transistor base resistor
  //   MCP GP2     -> RGB LED #1 Blue transistor base resistor
  //   MCP GP3     -> RGB LED #2 Red transistor base resistor
  //   MCP GP4     -> RGB LED #2 Green transistor base resistor
  //   MCP GP5     -> RGB LED #2 Blue transistor base resistor
  //   MCP GP6     -> Green LED transistor base resistor
  //   MCP GP7     -> currently unused / spare
  TrainLedMCP23008 trainLedExpander;
  bool trainLedExpanderDetected = false;
  const LedRoute led1R = { led1RedExpanderPin };
  const LedRoute led1G = { led1GreenExpanderPin };
  const LedRoute led1B = { led1BlueExpanderPin };
  const LedRoute led2R = { led2RedExpanderPin };
  const LedRoute led2G = { led2GreenExpanderPin };
  const LedRoute led2B = { led2BlueExpanderPin };
  const LedRoute ledGreen = { ledGreenExpanderPin };
  // GP7 drives two rear red LEDs wired in parallel and treated as one warning/shutdown indicator.
  const LedRoute ledRearRed = { ledRearRedExpanderPin };

  // ================================================================================================
  // Buzzer sound patterns
  // ================================================================================================
  const uint16_t pattern_melody[] PROGMEM = { 150, 80, 200, 80, 250, 80, 300, 150, 250, 0 };
  const uint16_t pattern_batteryWarn[] PROGMEM = { 3000, 100, 0 };
  const uint16_t pattern_double[] PROGMEM = { 150, 100, 150, 0 };
  const uint16_t pattern_descend[] PROGMEM = { 120, 80, 120, 80, 120, 0 };
  const uint16_t pattern_horn[] PROGMEM = { 1000, 100, 0 };
  const uint16_t pattern_tiltBeep[] PROGMEM = { 500, 0 };  // single 0.5s beep
  const uint16_t pattern_criticalOvervoltage[] PROGMEM = {
    180, 90, 180, 90, 600, 0
  };
  // PROGMEM keeps this short wake-up jingle in flash. It is REQUIRED here because
  // playToneSequence_P() always reads its notes with pgm_read_word() (a flash-read instruction);
  // without PROGMEM the array would live in SRAM and the flash reads would return garbage notes.
  const int16_t melodyWakeReady[] PROGMEM = { 988, 120, 1319, 180 };

  // ================================================================================================
  // Other constants
  // ================================================================================================
  // ================================================================================================
  // Control variables
  // ================================================================================================
  bool SoundOnOff = true;
  bool AutoDistanceOnOff = false;
  bool ColorSensorOnOff = false;
  BatteryState batteryState = BatteryState::Normal;
  ShutdownCause shutdownCause = ShutdownCause::None;
  bool criticalOvervoltageLatched = false;
  uint8_t MotorDirection = 1;   // always has a direction
  uint8_t Speed = 0;            // stopped at start

  // --- Motor driver fault state ---
  bool motorFaultLatched = false;
  bool motorDriveAttemptedSinceFault = true;  // Track if motor run was attempted after fault
  #if ENABLE_EEPROM_LOGGING
  uint16_t currentBootSequence = 0;
  uint8_t currentBootSlot = 0xFF;  // Ring buffer slot index for current boot session
  uint8_t bootFaultCount = 0;      // Motor fault counter for current power-on session (0..16)
  uint8_t runtimeEepromWrites = 0; // Shared budget for loop-originated EEPROM updates this power-on cycle.
  #endif

  // --- Momentary jog state ---
  bool momentaryActive = false;
  uint8_t momentaryButton = 0;
  unsigned long momentaryLastSeen = 0;
  // --- IR repeat tracking ---
  uint8_t lastIRCommand = 0;
  bool lastWasRepeat = false;
  uint8_t pendingIRCommand = 0;
  bool pendingIRWasRepeat = false;

  // Timers
  unsigned long bootStartedAt = 0;
  unsigned long lastActive = 0;
  unsigned long lastBatteryDebugPrintMs = 0;
  unsigned long lastBatteryCheckMs = 0;
  unsigned long lastVccCheckMs = 0;
  unsigned long lastBatteryWarningSignalMs = 0;
  unsigned long batterySignalEndsAt = 0;
  bool batterySignalActive = false;
  bool batterySignalUsesTone = false;
  bool idleSleepActive = false;
  // "volatile" tells the compiler this variable can change at any moment outside the normal program
  // flow - here, it's set inside an interrupt service routine (ISR), which can run in the middle of
  // loop() whenever the wake pin changes state. Without "volatile", the compiler might assume the
  // variable never changes on its own and wrongly optimize away checks of it in the main code.
  volatile bool idleWakeRequested = false;
  bool shutdownSignalPlayedThisBoot = false;
  bool irReceiverStarted = false;
  bool idleSleepWarningIssued = false;

  // Non-blocking green status-LED acknowledgement blink
  uint8_t greenBlinkRemaining = 0;
  bool greenBlinkOn = false;
  unsigned long greenBlinkStepMs = 0;
  // Non-blocking reverse-direction cooldown
  unsigned long motorReverseReadyAt = 0;
  bool motorDrivePending = false;
  bool boostActive = false;
  unsigned long boostEndsAt = 0;
  unsigned long boostCooldownEndsAt = 0;
  const __FlashStringHelper* pendingMotorStopReason = nullptr;

  // Actions run only on color transitions, so a long marker does not keep retriggering.
  // To disable a color's action, comment out that case in handleTrackMarkerAction().
  // Action bodies are placeholders for now - define each behavior in handleTrackMarkerAction().

  // Speed steps (voltage → PWM)
  uint8_t pwmSteps[5];  // 0..255
  uint16_t batteryVoltage = 0;
  uint16_t loadedBatteryVoltage = 0;
  uint16_t vccVoltage = 0;
  uint8_t consecutiveLowVccSamples = 0;
  unsigned long lastLoadedBatteryReadMs = 0;
  // Buzzer
  #define BUZZER_PATTERN_MAX 20
  uint16_t buzzerPattern[BUZZER_PATTERN_MAX];
  int buzzerIndex = 0;
  unsigned long buzzerTimer = 0;

  // Manual step index
  uint8_t currentStep = 0;  // 0=stop, 1=~3.5V, 2=~4.5V, 3=~6V, 4=~7V boost

  // Siren state
  bool sirenActive = false;
  unsigned long sirenTimer = 0;
  int sirenPhase = 0;  // 0=LED1 red, 1=LED1 blue

  // Siren timing (time-based sweep → stable even with loop jitter)
  unsigned long sirenStartMs = 0;          // set when siren toggles ON

  // ── Tilt sensor debounce config/state ─────────────────────────────────────────
  // Polarity (matches the D9 wiring note above): the switch is CLOSED to GND while the train is
  // upright, so LOW = idle/upright; when the train tips over the switch OPENS and the internal
  // pull-up takes the pin HIGH = tilt alarm. Both states start as HIGH on purpose: if the train
  // boots upright, the first stable LOW reading lands in the harmless "IDLE" branch of
  // updateTiltSensor(), which simply refreshes the drive lights once.
  int tiltStableState = HIGH;
  int tiltLastRead = HIGH;
  unsigned long tiltEdgeAt = 0;
  unsigned long tiltQuietUntil = 0;
  bool tiltStopLatched = false;
  bool accelerometerTiltStopLatched = false;
  bool accelerometerCrashLatched = false;

  // ── Tone melody player state ────────────────────────────────────────────────
  const int MELODY_MAX_PAIRS = 64;        // up to 64 (freq,dur) pairs
  const int16_t* melodySrc = nullptr;     // pointer to the active flat [f,d,f,d,...] sequence
  bool melodySrcIsProgmem = false;        // true if melodySrc points into PROGMEM
  int melodyLenPairs = 0;               // number of (f,d) pairs loaded
  int melodyIdxPair = 0;                // current pair index
  bool melodyLoop = false;              // loop playback
  bool melodyPlaying = false;           // active?
  unsigned long melodyStepStarted = 0;  // ms when current note started

  // One-time hardware initialization and startup behavior.
  // Initialize hardware and startup state.

  // ================================================================================================
  // Shared I2C bus initialization
  // ================================================================================================
  // Called exactly once from setup(), BEFORE any I2C device driver (MCP23008, TCS34725, distance sensor,
  // MPU6050) touches the bus. Doing this in one place (instead of inside every driver's begin())
  // guarantees every device sees the same bus speed and timeout settings.
  void initI2cBus() {
    // --- Step 1: bus recovery ("bus clear") ---
    // If the Nano resets in the middle of an I2C read (brown-out, watchdog reset), a slave chip can
    // be left holding the SDA data line LOW, which deadlocks the whole bus forever. The standard
    // fix (I2C spec section 3.1.16) is to manually pulse SCL up to 9 times so the stuck slave
    // finishes clocking out its byte and releases SDA, then issue a STOP condition.
    // This must run before Wire.begin() because it bit-bangs the pins directly.
    pinMode(A4, INPUT_PULLUP);  // A4 = SDA on the Nano
    pinMode(A5, INPUT_PULLUP);  // A5 = SCL on the Nano
    delayMicroseconds(5);
    if (digitalRead(A4) == LOW) {  // Only intervene when a slave is actually holding SDA low.
      for (uint8_t i = 0; i < 9 && digitalRead(A4) == LOW; i++) {
        pinMode(A5, OUTPUT);       // Drive SCL low...
        digitalWrite(A5, LOW);
        delayMicroseconds(5);
        pinMode(A5, INPUT_PULLUP); // ...then release it high (open-drain style, ~100 kHz pace).
        delayMicroseconds(5);
      }
      // Generate a STOP condition (SDA rising while SCL is high) to reset every slave's bus state.
      pinMode(A4, OUTPUT);
      digitalWrite(A4, LOW);
      delayMicroseconds(5);
      pinMode(A4, INPUT_PULLUP);
      delayMicroseconds(5);
    }
    // --- Step 2: start the Wire library and configure the bus ---
    Wire.begin();
    // 400 kHz "Fast Mode": all four devices on this bus support it, and it cuts every sensor
    // read/write to a quarter of the default 100 kHz time - important for a loop() that must stay
    // fast. If a future device only supports 100 kHz, lower this value.
    Wire.setClock(400000);
    #if defined(WIRE_HAS_TIMEOUT)
    // Without a timeout, a wedged bus would make Wire calls block forever and the watchdog would
    // keep resetting the train. 3000 us timeout + auto-reset of the Wire hardware on timeout.
    Wire.setWireTimeout(3000, true);
    Wire.clearWireTimeoutFlag();
    #endif
  }

  void setup() {
    #if defined(__AVR__)
    MCUSR = 0;
    wdt_disable();
    #endif
    bootStartedAt = millis();

    DBGBEGIN(115200);
    #if DEBUG_ANY
    delay(1000);  // Let the serial monitor attach after the Nano auto-resets on port open.
    #endif
    #if DEBUG_ANY
    Serial.println(F("=== TRAIN STARTUP ==="));
    #endif
    #if defined(__AVR__)
    // The watchdog timer (WDT) is a hardware countdown timer independent of the main program. If
    // the countdown reaches zero without being reset, the chip assumes the code has hung/crashed
    // and forces a full reset - a safety net for a battery-powered toy with no easy "reboot" button.
    // wdt_enable(WDTO_2S) arms it for a 2-second timeout; wdt_reset() (called once here, and again
    // at the very top of every loop() below) restarts the countdown so a healthy program never
    // actually triggers a reset.
    wdt_enable(WDTO_2S);
    wdt_reset();
    #endif

    // Initialize Arduino pins
    initI2cBus();  // Must run before any I2C device init below (bus clear + speed + timeout).
    initTrainLedHardware();
    pinMode(pinBuzzer, OUTPUT);
    pinMode(pinMotor_IN1, OUTPUT);
    pinMode(pinMotor_IN2, OUTPUT);
    pinMode(pinMotorFault, INPUT_PULLUP);
    pinMode(pinMotorSleep, OUTPUT);
    digitalWrite(pinMotorSleep, HIGH);  // Enable the DRV8833 after power-up.
    //pinMode(pinIRReceiver, INPUT);
    pinMode(pinBatterySense, INPUT);
    initBatteryVoltageMeterHardware();
    initSensorHardware();

    // Battery must be measured before writeBootErrorCodes() so the voltage is included in the log.
    batteryVoltage = getBatteryVoltageDirect();
    if (criticalOvervoltageLatched) {
      // The latch was already set inside getBatteryVoltageDirect() - stop setup here.
      return;
    }
    #if !DISABLE_VOLTAGE_METERING
    if (confirmLowVccAtStartup()) {
      enterBatteryShutdown(true, ShutdownCause::LowVcc);
      return;
    }
    #endif
    // F("...") wraps a text string so it stays stored in flash memory (PROGMEM) instead of being
    // copied into precious SRAM at startup. Debug/status text is a great candidate for F() because
    // it's read-only and only needed occasionally, freeing up SRAM for actual program data. This
    // sketch uses F() throughout for Serial.print/println debug strings for the same reason.
    DBG_POWER_MANAGEMENT(F("Battery measured: "));
    if (batteryVoltage > BATTERY_MAX_VALID_MV) {
      DBGLN_POWER_MANAGEMENT(F("invalid (>8.5V on 2S or meter error)"));
    } else {
      DBG_POWER_MANAGEMENT(batteryVoltage / 1000);
      DBG_POWER_MANAGEMENT(F("."));
      DBGLN_POWER_MANAGEMENT((batteryVoltage % 1000) / 10);
    }

    #if ENABLE_EEPROM_LOGGING
    // Log boot sequence, sensor status, and battery voltage to EEPROM (setup only, never in loop).
    writeBootErrorCodes();
    #if DEBUG_EEPROM
    dumpEepromDebugSummary();
    #endif
    #endif

    #if !DISABLE_VOLTAGE_METERING
    // PRODUCTION-only low-battery protection: these checks are compiled out in a
    // DISABLE_VOLTAGE_METERING=1 testing build, where a bench 5 V supply would otherwise look like
    // a deeply discharged pack and lock the train in shutdown at every boot.
    if (batteryVoltage <= BATTERY_LOW_SHUTDOWN_MV) {
      enterBatteryShutdown(true);
      return;
    } else if (batteryVoltage <= BATTERY_LOW_WARNING_MV) {
      enterBatteryWarning();
    }
    #endif

    //playPattern(pattern_melody);  // Play melody
    SetGreenLightValue(0);
    SetRGBColor(FrontLightOnOff ? RgbColor::Red : RgbColor::Off);
    if (batteryState == BatteryState::Normal) playToneSequence_P(melodyDemo, false);

    // Configure dynamic speed steps
    configureSpeedSteps();

    lastActive = millis();            // seed idle timer
    IrReceiver.begin(pinIRReceiver, DISABLE_LED_FEEDBACK);  // No D13 feedback blink; keeps the pin free and saves flash.
    irReceiverStarted = true;
  }

  // ================================================================================================
  // Main Loop
  // ================================================================================================
  // Cooperative scheduler: the loop runs many times per second, so every helper called here must
  // return quickly. Long delays would make the train miss remote presses, ignore a tilt event, or
  // react too slowly to a low battery or obstacle. Each numbered block below checks one subsystem
  // and then gives control back immediately so the next subsystem also gets time to run.
  void loop() {
    #if defined(__AVR__)
    wdt_reset();
    #endif

    if (criticalOvervoltageLatched) {
      // Critical overvoltage: keep every output in its safe state and let the one-time alert
      // pattern finish, then drop into permanent deep sleep instead of spinning in loop() forever
      // (spinning would keep the MCU, sensors, and IR receiver drawing current from a supply that
      // is already suspected to be faulty). The rear red indicator stays lit while sleeping - see
      // performPermanentShutdown(). Recovery requires a physical power cycle.
      applyCriticalOvervoltageOutputs();
      updateBuzzer();
      if (!isBuzzerPatternPlaying()) {
        performPermanentShutdown();
      }
      return;
    }

    #if !DISABLE_VOLTAGE_METERING
    updateVccGuard();
    #endif

    // === 1. DRV8833 fault watchdog ===
    // Reads the motor-driver fault pin and latches a safe stop if the driver reports an error such
    // as overcurrent or thermal protection. This must run early so later drive commands cannot
    // restart the motor before the fault state has been handled.
    updateMotorFault();

    // === 2. Idle timeout watchdog ===
    // Checks how long the train has been inactive. If the user has not pressed the remote for a
    // long time, this block starts the sleep sequence that turns off extras and waits for a wake
    // command. The short warning blink a little before sleeping is handled here too.
    if (canEnterIdleSleep() && millis() - lastActive > idleTimeout) {
      goToIdle();
    }
    if (!idleSleepWarningIssued
        && canEnterIdleSleep()
        && !idleSleepActive
        && (millis() - lastActive) >= (idleTimeout - IDLE_SLEEP_WARNING_LEAD_MS)) {
      idleSleepWarningIssued = true;
      DBGLN_POWER_MANAGEMENT(F("Idle sleep warning issued"));
      GreenLEDBlink();
    }

    // === 2b. Low-battery guard (only judged while stopped) ===
    // Measures battery health only when the train is safely stopped, so motor-voltage sag does not
    // trigger a false warning or shutdown. It also advances the warning/shutdown sound-light signal
    // without blocking the rest of the loop.
    updateBatteryGuard();
    updateBatterySignal();
    if (batteryState == BatteryState::Shutdown && !batterySignalActive) {
      performPermanentShutdown();
    }

    // === 3. Jog watchdog (safety if button released) ===
    // Momentary jog buttons should move the train only while the button is being received. If IR
    // repeats stop arriving, this block treats that as a released button and stops the motor.
    if (!motorFaultLatched && momentaryActive && (millis() - momentaryLastSeen > momentaryTimeout)) {
      DBGLN_MOTOR(F("Jog watchdog timeout -> STOP"));
      Stop();
      SetRGBColor(RgbColor::Red);
      momentaryActive = false;
    }

    // === 4. Auto-speed mode ===
    // When obstacle-following mode is enabled, this block reads the distance sensor, converts the
    // measured distance into a safe target voltage, and lets the motor helper ramp toward that
    // target. It also handles the timed end of boost mode and starts the cooldown window.
    if (!motorFaultLatched && AutoDistanceOnOff == 1) {
      updateAutoDistanceSpeed();
    }
    if (boostActive && (long)(millis() - boostEndsAt) >= 0) {
      boostActive = false;
      boostCooldownEndsAt = millis() + BOOST_COOLDOWN_MS;
      currentStep = NORMAL_MAX_SPEED_STEP;
      DBGLN_MOTOR(F("Boost cooldown period started, speed reduced"));
      playPattern(pattern_descend);
      applySpeedStep();
    }

    // === 5. Tilt sensor ===
    // Debounces the tilt switch so bumps do not cause false alarms. A confirmed tilt stops the
    // train, blocks new drive commands, and provides the user with a clear warning indication.
    updateTiltSensor();
    updateAccelerometerSafety();

    // === 6. IR remote handler ===
    // Reads the newest NEC frame from the remote and maps it to train actions such as speed
    // changes, lights, sounds, mode toggles, and wake-up behavior.
    translateIR();

    // === 7. Color sensor handler ===
    // If color-marker mode is enabled, this block polls the color sensor, classifies the current
    // marker under the train, and triggers the configured action when the marker changes.
    updateColorSensor();

    // === 8. Non-blocking feedback engines ===
    // These helpers advance sounds, blinking LEDs, melodies, and cooldown timers one tiny step at
    // a time. Because they never sit in long delays, the loop stays responsive while feedback is
    // still playing in the background.
    updateBuzzer();
    updateSiren();
    updateMelody();
    updateGreenBlink();
    updateMotorReverseCooldown();

  }

  // ================================================================================================
  // Tilt sensor
  // ================================================================================================
  // Sensor-wide hardware setup groups tilt pin setup, accelerometer probing, distance-sensor reset,
  // and color-sensor startup. The individual sensor drivers live in 41-color-sensor.ino,
  // the selected distance-sensor backend, and 43-accelerometer.ino.
  void initSensorHardware() {
    pinMode(pinTiltSensor, INPUT); // external pull resistor expected
    initAccelerometerHardware();
    initDistanceSensorHardware();
    initColorSensorHardware();
  }

  // Debounce tilt input and latch emergency stop.
  // "Debouncing" means waiting for a signal to settle before trusting it: a mechanical tilt switch
  // can flicker rapidly between HIGH/LOW for a few milliseconds while it's physically moving, so
  // reacting to every raw reading would cause false triggers. This function tracks the last raw
  // reading (tiltLastRead) and when it changes (tiltEdgeAt), then only accepts it as a real,
  // "stable" state change once it has held steady for TILT_STABLE_MS - and then enforces a further
  // "quiet period" (tiltQuietUntil) afterward before it will consider yet another change, to avoid
  // rapid re-triggering right at the edge of stability.
  // This alarm bypasses the battery-restriction sound gate (bypassBatteryGate = true), the same way
  // the denial beeps elsewhere in this project do: a physical tip-over is a safety event and must
  // stay audible even during battery Warning/Shutdown restrictions.
  void updateTiltSensor() {
    unsigned long now = millis();
    int reading = digitalRead(pinTiltSensor);

    if (reading != tiltLastRead) {
      tiltLastRead = reading;
      tiltEdgeAt = now;
    }

    if ((long)(now - tiltQuietUntil) >= 0 && reading != tiltStableState && (now - tiltEdgeAt) >= TILT_STABLE_MS) {

      tiltStableState = reading;
      tiltQuietUntil = now + TILT_QUIET_MS;

      if (tiltStableState == HIGH) {
        DBGLN_TILT_SENSOR(F("TILT: ACTIVE -> emergency stop"));
        if (!tiltStopLatched) {
          stopAndResetStepSelection();
          exitAutoDistanceMode(false);  // optional: exit AUTO
          tiltStopLatched = true;
        }
        sirenActive = false;   // silence siren first so the tilt beep is audible
        stopMelody();
        noTone(pinBuzzer);
        SetRGBColor(RgbColor::Red);
        // Bypass the battery-restriction sound gate: this is a safety alarm, not a user sound.
        playPattern(pattern_tiltBeep, true);
      } else {
        DBGLN_TILT_SENSOR(F("TILT: IDLE -> clear latch, restore LEDs"));
        tiltStopLatched = false;
        refreshDriveLights();
      }
    }
  }
