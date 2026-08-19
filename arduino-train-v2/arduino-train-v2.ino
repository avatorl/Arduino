  // Warning: when IRremote is configured to use Timer1, don’t use PWM (analogWrite) on Timer1 pins (D9, D10)
  
  // Library includes
  #include <Wire.h>         // I2C bus
  #include <IRremote.hpp>  // IR remote library
  #include <LowPower.h>    // Low power/sleep mode library
  #if ENABLE_EEPROM_LOGGING
  #include <EEPROM.h>       // Persistent boot-error log
  #endif
  #include <math.h>        // Math functions


  #include "src/melodies.h" // Load music patterns

  // ===============================================================================================
  // Description
  // ===============================================================================================
  // Arduino DUPLO Train v2 is a remote-controlled DUPLO-compatible train with:
  // - IR remote driving with forward, backward, stop, and three manual speed levels
  //   plus a higher speed boost / turbo-style drive mode where supported by the current state
  // - headlights and colored status lights
  // - horn, siren, and melody playback
  // - battery status reporting and low-battery protection
  // - automatic sleep after inactivity and wake-up from the IR remote
  // - tilt protection that stops the train when it is on its side
  // - obstacle detection with automatic stop and restart when the distance sensor is used
  // - optional color recognition for track markers, with color-based actions triggered on changes

  // ===============================================================================================
  // Sketch Contents / Structure Guide
  // ===============================================================================================
  //  1. PROGMEM melody-sequence playback wrapper
  //       playToneSequence_P()              - Template wrapper for PROGMEM melodies.
  //
  //  2. LED expander and color-sensor initialization, marker-color classification, and drive-light refresh
  //       initTrainLedHardware()            - Detect and initialize the MCP23008 LED expander.
  //       initColorSensorHardware()         - Configure the TCS34725 color sensor and its onboard lamp control.
  //       setColorSensorEnabled()           - Toggle color-sensor mode and related status lighting.
  //       applyWhiteBalance()               - Apply per-channel gain correction to raw RGBC data.
  //       normalizePrototype()              - Convert RGB counts to a 0-1000 normalized triple.
  //       prototypeDistance()               - Manhattan distance between two normalized RGB prototypes.
  //       classifyTrackMarkerColor()        - Convert raw RGBC values into track marker classes.
  //       trackMarkerLabel()                - Debug-only marker name lookup.
  //       handleTrackMarkerAction()         - Run the action associated with a detected marker color.
  //       updateColorSensor()               - Poll and process the TCS34725 color sensor without blocking.
  //       refreshDriveLights()              - Restore status lights from the current drive state.
  //
  //  3. Setup, main loop, idle/sleep, and boot-error logging
  //       writeBootErrorCodes()             - Log boot sequence, sensor status, and battery to EEPROM ring buffer.
  //       setup()                           - Initialize hardware and startup state.
  //       loop()                            - Main scheduler for safety, input, sensing, and playback.
  //       goToIdle()                        - Shut down outputs and enter low-power sleep.
  //       wakeUp()                          - Interrupt callback used only to wake the MCU.
  //
  //  4. Power management and battery measurement helpers
  //       initBatteryVoltageMeterHardware() - Select the internal 1.1V ADC reference.
  //       getBatteryVoltageDirect()         - Read pack voltage through the resistor divider.
  //       getBatteryVoltageSettledForStatus() - Stop loads, let the pack settle, then measure.
  //       updateBatteryGuard()              - Periodically check voltage and enforce low-battery limits.
  //       get2SBatteryPercent()             - Map pack voltage to a 0-100% charge estimate.
  //       safePWMFromVoltage()              - Convert a target motor voltage to a safe PWM value.
  //
  //  5. Manual speed-step control
  //       configureSpeedSteps()             - Build manual speed steps from live battery voltage.
  //       playStepBeep()                    - Audible confirmation for the selected manual step.
  //       applySpeedStep()                  - Apply the current manual step to the motor state.
  //       increaseStep() / decreaseStep()   - Move between manual drive steps.
  //       stopAndResetStepSelection()       - Stop motor and reset manual speed step to 0.
  //
  //  6. Auto-distance speed control and distance filtering
  //       motorVoltageFromDistance()        - Map obstacle distance to a target motor voltage.
  //       initDistanceSensorHardware()      - Initialize the VL53L0X distance sensor backend.
  //       startDistanceSensorRanging()      - (Re-)configure and start continuous VL53L0X ranging.
  //       pushDistanceSampleAndGetMedian()  - Insert a sample into the median filter and return result.
  //       getDistanceReading()              - Return the latest filtered distance in cm.
  //       updateAutoDistanceSpeed()         - Run the automatic speed controller.
  //       updateMotorSpeed()                - Ramp the motor toward a requested speed.
  //       exitAutoDistanceMode()            - Disable auto-distance mode and clear its indicator.
  //
  //  7. IR receiver decoding and command translation
  //       irReceive()                       - Decode NEC commands, including repeat frames.
  //       tryPlayMelodyForButton()          - Map a remote button code to a melody and start playback.
  //       translateIR()                     - Map remote buttons to train behavior.
  //
  //  8. DRV8833 safety and re-arm logic
  //       isMotorControlCommand()           - Identify commands that may re-arm the motor driver.
  //       updateMotorFault()                - Latch, report, and EEPROM-log DRV8833 fault conditions.
  //       rearmMotorDriver()                - Re-enable the driver after a cleared fault.
  //
  //  9. Motor drive helpers and jog control
  //       setMotor()                        - Low-level DRV8833 direction and PWM output.
  //       GoForward() / GoBackward()        - Direction-safe drive entry points.
  //       Stop()                            - Coast the motor to an idle state.
  //       JogDrive()                        - Temporary hold-to-run movement helper.
  //       updateMotorReverseCooldown()      - Apply deferred motor drive after direction-change delay.
  //       cancelJog()                       - Clear the momentary jog active flag.
  //
  // 10. Buzzer patterns and spoken-voltage playback
  //       clearBuzzerPattern()              - Reset queued buzzer timing data.
  //       updateBuzzer()                    - Advance non-blocking buzzer patterns.
  //       playPattern()                     - Start a predefined buzzer pattern.
  //       playVoltagePattern()              - Speak battery voltage with long/short beeps.
  //
  // 11. RGB / status LED output helpers
  //       writeTrainOutput()                - Write one expander-backed LED channel.
  //       writeRGBPins()                    - Drive one RGB LED as discrete color channels.
  //       SetRGBLight()                     - Apply raw RGB values to one or both headlights.
  //       SetRGBLightColor()                - Apply a named RGB color.
  //       SetRGBColor()                     - Apply status color unless siren/sensor mode overrides it.
  //       SetGreenLightValue()              - Control the green status LED.
  //       GreenLEDBlink() / updateGreenBlink() - Non-blocking status-LED acknowledgement blink.
  //
  // 12. Siren effect, tilt protection, and non-blocking melody player
  //       updateSiren()                     - Animate siren lights and pitch sweep.
  //       updateTiltSensor()                - Debounce tilt input and latch emergency stop.
  //       stopMelody()                      - Stop melody playback and clear melody state.
  //       updateMelody()                    - Advance non-blocking melody playback.
  //       playToneSequenceRaw()             - Load and start a melody from RAM or PROGMEM.
  //
  // Named Type Index
  //   RgbColor                          - Headlight and status color palette.
  //   TrackMarkerClass                  - Track-marker classification labels.
  //   PrototypeRgb                      - Normalized RGB prototype triple.
  //   BalancedRgbs                      - White-balanced RGBC sample.
  //   MarkerClusterDefinition           - Marker-cluster entry used by color matching.
  //   LedRoute                          - LED expander pin mapping helper.
  //   TrainLedMCP23008                  - MCP23008 LED expander helper.
  //   TrainColorSensorTCS34725          - TCS34725 color-sensor helper.
  //   TrainDistanceSensorVL53L0X        - VL53L0X distance sensor helper.
  //   Dir                               - Motor direction state.

  #if ENABLE_EEPROM_LOGGING
  // ===============================================================================================
  // EEPROM boot log layout  (83 bytes total, Nano has 1024)
  // ===============================================================================================
  // 0x00..0x01  boot counter    uint16_t (2 bytes), wraps 0..65535; serves as the per-boot sequence/timestamp
  // 0x02        log head        uint8_t, next-write slot index (0..EEPROM_BOOT_LOG_SIZE-1)
  // 0x03..0x52  ring buffer     16 entries × 5 bytes each:
  //               bytes 0..1  boot_seq      — uint16_t (2 bytes) copy of counter at time of this boot
  //               byte 2      flags         — sensor error bitmask (0x00 = all OK)
  //               byte 3      battery       — pack voltage × 10 as uint8_t (e.g. 74 = 7.4 V); 0xFF = not measured
  //               byte 4      fault_count   — DRV8833 runtime fault count for this boot session (0..16)
  //
  // Error flag bit map:
  //   Bit 0  0x01  MCP23008 LED expander not detected
  //   Bit 1  0x02  TCS34725 color sensor not detected
  //   Bit 2  0x04  VL53L0X distance sensor not detected
  const uint16_t EEPROM_ADDR_BOOT_COUNT = 0x00;
  const uint8_t  EEPROM_ADDR_LOG_HEAD   = 0x02;
  const uint8_t  EEPROM_ADDR_LOG_BASE   = 0x03;
  const uint8_t  EEPROM_BOOT_LOG_SIZE   = 16;   // 16 entries × 5 bytes = 80 bytes for the ring buffer
  const uint8_t  EEPROM_ENTRY_SIZE      = 5;    // 5 bytes per entry
  const uint8_t  MAX_FAULTS_PER_BOOT    = 16;   // Cap fault increments / EEPROM rewrites per power-on cycle
  const uint8_t  ERR_LED_EXPANDER = 0x01;
  const uint8_t  ERR_COLOR_SENSOR = 0x02;
  const uint8_t  ERR_DISTANCE_TOF = 0x04;
  #endif

  // ===============================================================================================
  // Per-module debug flags. Set a flag to 1 to compile only that subsystem's
  // logs. The legacy DEBUG flag is still supported as a fallback that enables
  // every module when set to 1.
  #define DEBUG 0
  #define DEBUG_COLOR_SENSOR DEBUG
  #define DEBUG_DISTANCE_SENSOR DEBUG
  #define DEBUG_IR_REMOTE DEBUG
  #define DEBUG_TILT_SENSOR DEBUG
  #define DEBUG_VOLTAGE_METER DEBUG
  #define DEBUG_MOTOR DEBUG
  #define DEBUG_LEDS DEBUG
  #define DEBUG_SOUND DEBUG
  #define DEBUG_REMOTE DEBUG

  #define ENABLE_EEPROM_LOGGING 0

  #define DEBUG_ANY (DEBUG || DEBUG_COLOR_SENSOR || DEBUG_DISTANCE_SENSOR || DEBUG_IR_REMOTE || DEBUG_TILT_SENSOR || DEBUG_VOLTAGE_METER || DEBUG_MOTOR || DEBUG_LEDS || DEBUG_SOUND || DEBUG_REMOTE)

  #if DEBUG_ANY
  #define DBGBEGIN(...) \
    do { Serial.begin(__VA_ARGS__); } while (0)  // Initialize Serial
  #else
  #define DBGBEGIN(...)  // No operation
  #endif

  #if DEBUG || DEBUG_COLOR_SENSOR
  #define DBG_COLOR_SENSOR(...) Serial.print(__VA_ARGS__)
  #define DBGLN_COLOR_SENSOR(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_COLOR_SENSOR(...)
  #define DBGLN_COLOR_SENSOR(...)
  #endif

  #if DEBUG || DEBUG_DISTANCE_SENSOR
  #define DBG_DISTANCE_SENSOR(...) Serial.print(__VA_ARGS__)
  #define DBGLN_DISTANCE_SENSOR(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_DISTANCE_SENSOR(...)
  #define DBGLN_DISTANCE_SENSOR(...)
  #endif

  #if DEBUG || DEBUG_IR_REMOTE
  #define DBG_IR_REMOTE(...) Serial.print(__VA_ARGS__)
  #define DBGLN_IR_REMOTE(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_IR_REMOTE(...)
  #define DBGLN_IR_REMOTE(...)
  #endif

  #if DEBUG || DEBUG_TILT_SENSOR
  #define DBG_TILT_SENSOR(...) Serial.print(__VA_ARGS__)
  #define DBGLN_TILT_SENSOR(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_TILT_SENSOR(...)
  #define DBGLN_TILT_SENSOR(...)
  #endif

  #if DEBUG || DEBUG_VOLTAGE_METER
  #define DBG_VOLTAGE_METER(...) Serial.print(__VA_ARGS__)
  #define DBGLN_VOLTAGE_METER(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_VOLTAGE_METER(...)
  #define DBGLN_VOLTAGE_METER(...)
  #endif

  #if DEBUG || DEBUG_MOTOR
  #define DBG_MOTOR(...) Serial.print(__VA_ARGS__)
  #define DBGLN_MOTOR(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_MOTOR(...)
  #define DBGLN_MOTOR(...)
  #endif

  #if DEBUG || DEBUG_LEDS
  #define DBG_LEDS(...) Serial.print(__VA_ARGS__)
  #define DBGLN_LEDS(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_LEDS(...)
  #define DBGLN_LEDS(...)
  #endif

  #if DEBUG || DEBUG_SOUND
  #define DBG_SOUND(...) Serial.print(__VA_ARGS__)
  #define DBGLN_SOUND(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_SOUND(...)
  #define DBGLN_SOUND(...)
  #endif

  #if DEBUG || DEBUG_REMOTE
  #define DBG_REMOTE(...) Serial.print(__VA_ARGS__)
  #define DBGLN_REMOTE(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_REMOTE(...)
  #define DBGLN_REMOTE(...)
  #endif
  // ===============================================================================================

  // AVR-specific include for program memory storage
  #if defined(__AVR__)
  #include <avr/pgmspace.h>
  #endif

  // Headlight and status color palette.
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

  // Track-marker classification labels.
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

  // Normalized RGB prototype triple.
  struct PrototypeRgb {
    uint16_t r;
    uint16_t g;
    uint16_t b;
  };

  // White-balanced RGBC sample.
  struct BalancedRgbs {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
  };

  // Marker-cluster entry used by color matching.
  struct MarkerClusterDefinition {
    TrackMarkerClass markerClass;
    PrototypeRgb center;
    uint16_t maxDistance;
  };

  // LED expander pin mapping helper.
  struct LedRoute;

  // Forward declarations are grouped here so the high-level flow can stay readable below.
  // Most functions are implemented in the same order as the contents guide above.
  void SetRGBLightColor(RgbColor color, int led = 0);
  void SetRGBColor(RgbColor color, int led = 0);

  void playToneSequenceRaw(const int* seqFD, int pairCount, bool loopPlayback, bool isProgmem = false);

  // PROGMEM wrapper (for PROGMEM melodies)
  // Template wrapper for PROGMEM melodies.
  template<size_t N>
  void playToneSequence_P(const int16_t (&seqFD)[N], bool loopPlayback = false) {
    static_assert(N % 2 == 0, "Melody array must have even length [freq,dur,...]");
    playToneSequenceRaw((const int*)seqFD, (int)(N / 2), loopPlayback, true);
  }

  void updateMelody();
  void stopMelody();
  void initTrainLedHardware();
  void initColorSensorHardware();
  void initBatteryVoltageMeterHardware();
  void setColorSensorEnabled(bool enabled);
  void updateColorSensor();
  void refreshDriveLights();
  float getBatteryVoltageSettledForStatus();
  void updateBatteryGuard();
  void updateGreenBlink();
  void updateMotorReverseCooldown();
  void initDistanceSensorHardware();
  bool startDistanceSensorRanging(bool reinitializeSensor = true);
  int getDistanceReading();
  bool tryPlayMelodyForButton(uint8_t code);
  void exitAutoDistanceMode(bool clearIndicator = true);
  void stopAndResetStepSelection(bool resetDirection = false);
  void cancelJog();
  int get2SBatteryPercent(float voltage);
  uint8_t classifyTrackMarkerColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
  void handleTrackMarkerAction(uint8_t markerClass);
  #if DEBUG || DEBUG_COLOR_SENSOR
  const __FlashStringHelper* trackMarkerLabel(uint8_t markerClass);
  #endif

  // Motor directions
  // Motor direction state.
  enum class Dir : uint8_t { Stop = 0,
                            Forward = 1,
                            Backward = 2 };

  void setMotor(Dir dir, int speed);
  void JogDrive(Dir dir);


  // ================================================================================================
  // Remote Button Codes (NEC protocol, "Car MP3" remote control)
  // ================================================================================================
  // Format: Decimal code | Label
  //  69 | CH-     70 | CH      71 | CH+
  //  68 | <<      64 | >>      67 | Play/Pause
  //   7 | -       21 | +        9 | EQ
  //  22 | 0       25 | 100+    13 | 200+
  //  12 | 1       24 | 2       94 | 3
  //   8 | 4       28 | 5       90 | 6
  //  66 | 7       82 | 8       74 | 9

  // IR remote button assignments
  const int buttonCHminus = 69;     // Speed -
  const int buttonCH = 70;          // Stop
  const int buttonCHplus = 71;      // Speed +
  const int buttonBackward = 68;    // Momentary backward
  const int buttonForward = 64;     // Momentary forward
  const int buttonPlayPause = 67;   // Auto-speed toggle, start/stop
  const int buttonEQ = 9;           // Mute / Unmute
  const int button0 = 22;           // Color sensor ON/OFF
  const int button100plus = 25;     // Horn
  const int button200plus = 13;     // Siren
  const int button1 = 12;           // Play music 1
  const int button2 = 24;           // Play music 2
  const int button3 = 94;           // Play music 3
  const int button4 = 8;            // Play music 4
  const int button5 = 28;           // Play music 5
  const int button6 = 90;           // Play music 6
  const int button7 = 66;           // Play music 7
  const int button8 = 82;           // Play music 8
  const int button9 = 74;           // Battery Test

  const __FlashStringHelper* irButtonLabel(uint8_t code) {
    switch (code) {
      case buttonCHminus: return F("CH-");
      case buttonCH: return F("CH");
      case buttonCHplus: return F("CH+");
      case buttonBackward: return F("<<");
      case buttonForward: return F(">>");
      case buttonPlayPause: return F("Play/Pause");
      case buttonEQ: return F("EQ");
      case button0: return F("0");
      case button100plus: return F("100+");
      case button200plus: return F("200+");
      case button1: return F("1");
      case button2: return F("2");
      case button3: return F("3");
      case button4: return F("4");
      case button5: return F("5");
      case button6: return F("6");
      case button7: return F("7");
      case button8: return F("8");
      case button9: return F("9");
      default: return F("Unknown");
    }
  }

  const __FlashStringHelper* irButtonActionDescription(uint8_t code) {
    switch (code) {
      case buttonCHminus: return F("Speed -");
      case buttonCH: return F("Stop");
      case buttonCHplus: return F("Speed +");
      case buttonBackward: return F("Momentary backward");
      case buttonForward: return F("Momentary forward");
      case buttonPlayPause: return F("Auto-speed toggle, start/stop");
      case buttonEQ: return F("Mute / Unmute");
      case button0: return F("Color sensor ON/OFF");
      case button100plus: return F("Horn");
      case button200plus: return F("Siren");
      case button1: return F("Play music 1");
      case button2: return F("Play music 2");
      case button3: return F("Play music 3");
      case button4: return F("Play music 4");
      case button5: return F("Play music 5");
      case button6: return F("Play music 6");
      case button7: return F("Play music 7");
      case button8: return F("Play music 8");
      case button9: return F("Battery Test");
      default: return F("Unassigned button");
    }
  }

  // ================================================================================================
  // Arduino Pin Mapping
  // ================================================================================================
  
  // Pin usage summary (actual Nano-side roles in this sketch), ordered by board pin:
  //   D0  -> Hardware UART RX shared with USB serial and DEBUG output; avoid other peripherals.
  //   D1  -> Hardware UART TX shared with USB serial and DEBUG output; avoid other peripherals.
  //   D2  -> IR receiver input; also used as the wake interrupt source from sleep.
  //   D3  -> VL53L0X XSHUT output for sensor reset / I2C address setup.
  //   D4  -> TCS34725 breakout LED control output.
  //   D5  -> DRV8833 IN1 motor drive PWM/direction output.
  //   D6  -> DRV8833 IN2 motor drive PWM/direction output.
  //   D7  -> DRV8833 nSLEEP output, HIGH = enabled, LOW = sleep.
  //   D8  -> DRV8833 nFAULT input with internal pull-up; active LOW.
  //   D9  -> Tilt sensor input with internal pull-up; switch closes to GND.
  //   D10 -> Free digital-only pin; do not use analogWrite because IRremote owns Timer1.
  //   D11 -> Free general-purpose digital pin if SPI is not needed; also SPI MOSI.
  //   D12 -> Passive buzzer output driven by tone(); also SPI MISO.
  //   D13 -> Free digital pin; also tied to the Nano onboard LED and SPI SCK.
  //   A0  -> Battery voltage sense analog input.
  //   A1  -> Free analog-input-only pin.
  //   A2  -> Free analog-input-only pin.
  //   A3  -> Free analog-input-only pin used here as general-purpose digital spare.
  //   A4  -> I2C SDA shared by the MCP23008, TCS34725, and VL53L0X.
  //   A5  -> I2C SCL shared by the MCP23008, TCS34725, and VL53L0X.
  //   A6  -> Free analog-input-only pin.
  //   A7  -> Free analog-input-only pin.
  // SPI note: SPI is not used in this sketch, but an SPI peripheral would conflict with the buzzer
  // on D12.
  
  const int pinBatterySense = A0;   // Battery voltage monitoring
  const int pinTiltSensor = 9;      // Tilt sensor, digital input with internal pull-up; switch closes to GND
  const uint8_t vl53l0xAddress = 0x2A; // Changed from default 0x29 to avoid conflict with TCS34725
  const int pinVL53L0X_XSHUT = 3;   // VL53L0X XSHUT pin for I2C address reset
  const int pinIRReceiver = 2;      // IR receiver input: D2 or D3 required to wake from sleep
  const int pinMotor_IN1 = 5;      // DRV8833 IN1: motor direction and speed (PWM)
  const int pinMotor_IN2 = 6;      // DRV8833 IN2: motor direction and speed (PWM)
  const int pinMotorFault = 8;      // DRV8833 ULT/nFAULT, active-LOW diagnostic output
  const int pinMotorSleep = 7;      // DRV8833 EEP/nSLEEP, HIGH = enabled, LOW = sleep
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
  struct TrainLedMCP23008 {
    // MCP23008 register addresses used by the expander helper.
    static const uint8_t RegisterIodir = 0x00;
    static const uint8_t RegisterGpio = 0x09;

    // Cached I2C address and shadow copies of the MCP23008 output state.
    uint8_t address = 0;
    uint8_t iodirShadow = 0xFF;
    uint8_t gpioShadow = 0x00;

    // Initialize the MCP23008 over I2C and seed the shadow registers.
    bool begin_I2C(uint8_t i2cAddress) {
      Wire.begin();
      address = i2cAddress;
      iodirShadow = 0xFF;
      gpioShadow = 0x00;
      if (!probe()) return false;
      if (!writeRegister(RegisterGpio, gpioShadow)) return false;
      return writeRegister(RegisterIodir, iodirShadow);
    }

    // Configure a GPIO pin as an output in the shadow IODIR register.
    void pinMode(uint8_t pin, uint8_t mode) {
      if (mode != OUTPUT || pin > 7) return;
      iodirShadow &= (uint8_t)~(1 << pin);
      writeRegister(RegisterIodir, iodirShadow);
    }

    // Drive one expander GPIO high or low and mirror it in the shadow GPIO register.
    void digitalWrite(uint8_t pin, uint8_t value) {
      if (pin > 7) return;
      if (value == HIGH) {
        gpioShadow |= (1 << pin);
      } else {
        gpioShadow &= (uint8_t)~(1 << pin);
      }
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

  // TCS34725 color-sensor helper.
  struct TrainColorSensorTCS34725 {
    // TCS34725 register addresses and command bits used by the helper.
    static const uint8_t DefaultAddress = 0x29;
    static const uint8_t CommandBit = 0x80;
    static const uint8_t CommandAutoIncrement = 0x20;
    static const uint8_t RegisterEnable = 0x00;
    static const uint8_t RegisterAtime = 0x01;
    static const uint8_t RegisterId = 0x12;
    static const uint8_t RegisterControl = 0x0F;
    static const uint8_t RegisterClearDataLow = 0x14;
    static const uint8_t EnablePowerOn = 0x01;
    static const uint8_t EnableAdc = 0x02;
    static const uint8_t IntegrationTime50ms = 0xEB;
    static const uint8_t Gain4x = 0x01;

    // Current sensor I2C address.
    uint8_t address = DefaultAddress;

    // Initialize the TCS34725, confirm the chip ID, and program the basic sensor settings.
    bool begin_I2C(uint8_t i2cAddress = DefaultAddress) {
      Wire.begin();
      address = i2cAddress;

      uint8_t deviceId = 0;
      if (!probe()) return false;
      if (!readRegister(RegisterId, &deviceId)) return false;
      if (!isSupportedDeviceId(deviceId)) return false;
      if (!writeRegister(RegisterAtime, IntegrationTime50ms)) return false;
      return writeRegister(RegisterControl, Gain4x);
    }

    // Power the sensor on and enable RGBC ADC conversion.
    bool enable() {
      if (!writeRegister(RegisterEnable, EnablePowerOn)) return false;
      delay(3);
      return writeRegister(RegisterEnable, (uint8_t)(EnablePowerOn | EnableAdc));
    }

    // Put the sensor core back into sleep/power-down.
    bool disable() {
      return writeRegister(RegisterEnable, 0x00);
    }

    // Read the raw clear/red/green/blue channels from the sensor.
    bool readRawData(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
      uint8_t raw[8] = { 0 };
      if (!readRegisters(RegisterClearDataLow, raw, sizeof(raw))) {
        *r = 0;
        *g = 0;
        *b = 0;
        *c = 0;
        return false;
      }

      *c = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
      *r = (uint16_t)raw[2] | ((uint16_t)raw[3] << 8);
      *g = (uint16_t)raw[4] | ((uint16_t)raw[5] << 8);
      *b = (uint16_t)raw[6] | ((uint16_t)raw[7] << 8);
      return true;
    }

   private:
    // Return true only for known supported TCS34725 family IDs.
    bool isSupportedDeviceId(uint8_t deviceId) {
      return deviceId == 0x44 || deviceId == 0x4D || deviceId == 0x10;
    }

    // Probe whether the TCS34725 responds on the configured I2C address.
    bool probe() {
      Wire.beginTransmission(address);
      return Wire.endTransmission() == 0;
    }

    // Write one TCS34725 register.
    bool writeRegister(uint8_t reg, uint8_t value) {
      Wire.beginTransmission(address);
      Wire.write((uint8_t)(CommandBit | reg));
      Wire.write(value);
      return Wire.endTransmission() == 0;
    }

    // Read one TCS34725 register.
    bool readRegister(uint8_t reg, uint8_t* value) {
      return readRegisters(reg, value, 1);
    }

    // Read a contiguous register range from the sensor.
    bool readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length) {
      Wire.beginTransmission(address);
      Wire.write((uint8_t)(CommandBit | CommandAutoIncrement | startReg));
      if (Wire.endTransmission(false) != 0) return false;

      uint8_t bytesRead = Wire.requestFrom((int)address, (int)length);
      if (bytesRead != length) return false;

      for (uint8_t i = 0; i < length; ++i) {
        buffer[i] = (uint8_t)Wire.read();
      }
      return true;
    }
  };

  // VL53L0X distance-sensor helper.
  struct TrainDistanceSensorVL53L0X {
    // VL53L0X register addresses used by the helper.
    static const uint8_t DefaultAddress = 0x29;
    static const uint8_t RegisterSysrangeStart = 0x00;
    static const uint8_t RegisterSystemSequenceConfig = 0x01;
    static const uint8_t RegisterSystemIntermeasurementPeriod = 0x04;
    static const uint8_t RegisterSystemInterruptConfigGpio = 0x0A;
    static const uint8_t RegisterSystemInterruptClear = 0x0B;
    static const uint8_t RegisterResultInterruptStatus = 0x13;
    static const uint8_t RegisterResultRangeStatus = 0x14;
    static const uint8_t RegisterFinalRangeConfigMinCountRateRtnLimit = 0x44;
    static const uint8_t RegisterMsrcConfigTimeoutMacrop = 0x46;
    static const uint8_t RegisterMsrcConfigControl = 0x60;
    static const uint8_t RegisterSystemHistogramBin = 0x81;
    static const uint8_t RegisterGpioHvMuxActiveHigh = 0x84;
    static const uint8_t RegisterVhvConfigPadSclSdaExtsupHv = 0x89;
    static const uint8_t RegisterI2cSlaveDeviceAddress = 0x8A;
    static const uint8_t RegisterGlobalConfigSpadEnablesRef0 = 0xB0;
    static const uint8_t RegisterGlobalConfigRefEnStartSelect = 0xB6;
    static const uint8_t RegisterIdentificationModelId = 0xC0;
    static const uint8_t RegisterOscCalibrateVal = 0xF8;

    // Current sensor state and timing bookkeeping.
    uint8_t address = DefaultAddress;
    uint16_t ioTimeoutMs = 0;
    uint8_t stopVariable = 0;
    uint32_t measurementTimingBudgetUs = 33000UL;
    bool didTimeout = false;
    bool lastReadValid = false;
    unsigned long timeoutStartMs = 0;

    // Set the I/O timeout used by the helper's polling logic.
    void setTimeout(uint16_t timeoutMs) {
      ioTimeoutMs = timeoutMs;
    }

    // Change the sensor's I2C address, falling back to the default address if needed.
    bool setAddress(uint8_t newAddress) {
      if (!writeRegAt(address, RegisterI2cSlaveDeviceAddress, newAddress & 0x7F)) {
        if (address == DefaultAddress) return false;
        if (!writeRegAt(DefaultAddress, RegisterI2cSlaveDeviceAddress, newAddress & 0x7F)) return false;
      }
      address = newAddress;
      return true;
    }

    // Initialize the VL53L0X and program the timing, calibration, and SPAD settings.
    bool init(bool io2v8 = true) {
      Wire.begin();
      didTimeout = false;
      lastReadValid = false;

      if (readReg(RegisterIdentificationModelId) != 0xEE) return false;

      if (io2v8) {
        if (!writeReg(RegisterVhvConfigPadSclSdaExtsupHv,
                      (uint8_t)(readReg(RegisterVhvConfigPadSclSdaExtsupHv) | 0x01))) return false;
      }

      if (!writeReg(0x88, 0x00)) return false;
      if (!writeReg(0x80, 0x01)) return false;
      if (!writeReg(0xFF, 0x01)) return false;
      if (!writeReg(0x00, 0x00)) return false;
      stopVariable = readReg(0x91);
      if (!writeReg(0x00, 0x01)) return false;
      if (!writeReg(0xFF, 0x00)) return false;
      if (!writeReg(0x80, 0x00)) return false;

      if (!writeReg(RegisterMsrcConfigControl, (uint8_t)(readReg(RegisterMsrcConfigControl) | 0x12))) return false;
      if (!setSignalRateLimit(0.25f)) return false;
      if (!writeReg(RegisterSystemSequenceConfig, 0xFF)) return false;

      uint8_t spadCount = 0;
      bool spadTypeIsAperture = false;
      if (!getSpadInfo(&spadCount, &spadTypeIsAperture)) return false;

      uint8_t refSpadMap[6] = { 0 };
      if (!readMulti(RegisterGlobalConfigSpadEnablesRef0, refSpadMap, sizeof(refSpadMap))) return false;

      if (!writeReg(0xFF, 0x01)) return false;
      if (!writeReg(0x4F, 0x00)) return false;
      if (!writeReg(0x4E, 0x2C)) return false;
      if (!writeReg(0xFF, 0x00)) return false;
      if (!writeReg(RegisterGlobalConfigRefEnStartSelect, 0xB4)) return false;

      uint8_t firstSpadToEnable = spadTypeIsAperture ? 12 : 0;
      uint8_t spadsEnabled = 0;
      for (uint8_t i = 0; i < 48; ++i) {
        if (i < firstSpadToEnable || spadsEnabled == spadCount) {
          refSpadMap[i / 8] &= (uint8_t)~(1 << (i % 8));
        } else if ((refSpadMap[i / 8] >> (i % 8)) & 0x01) {
          ++spadsEnabled;
        }
      }
      if (!writeMulti(RegisterGlobalConfigSpadEnablesRef0, refSpadMap, sizeof(refSpadMap))) return false;

      const uint8_t initRegisters[][2] = {
        { 0xFF, 0x01 }, { 0x00, 0x00 }, { 0xFF, 0x00 }, { 0x09, 0x00 }, { 0x10, 0x00 },
        { 0x11, 0x00 }, { 0x24, 0x01 }, { 0x25, 0xFF }, { 0x75, 0x00 }, { 0xFF, 0x01 },
        { 0x4E, 0x2C }, { 0x48, 0x00 }, { 0x30, 0x20 }, { 0xFF, 0x00 }, { 0x30, 0x09 },
        { 0x54, 0x00 }, { 0x31, 0x04 }, { 0x32, 0x03 }, { 0x40, 0x83 }, { 0x46, 0x25 },
        { 0x60, 0x00 }, { 0x27, 0x00 }, { 0x50, 0x06 }, { 0x51, 0x00 }, { 0x52, 0x96 },
        { 0x56, 0x08 }, { 0x57, 0x30 }, { 0x61, 0x00 }, { 0x62, 0x00 }, { 0x64, 0x00 },
        { 0x65, 0x00 }, { 0x66, 0xA0 }, { 0xFF, 0x01 }, { 0x22, 0x32 }, { 0x47, 0x14 },
        { 0x49, 0xFF }, { 0x4A, 0x00 }, { 0xFF, 0x00 }, { 0x7A, 0x0A }, { 0x7B, 0x00 },
        { 0x78, 0x21 }, { 0xFF, 0x01 }, { 0x23, 0x34 }, { 0x42, 0x00 }, { 0x44, 0xFF },
        { 0x45, 0x26 }, { 0x46, 0x05 }, { 0x40, 0x40 }, { 0x0E, 0x06 }, { 0x20, 0x1A },
        { 0x43, 0x40 }, { 0xFF, 0x00 }, { 0x34, 0x03 }, { 0x35, 0x44 }, { 0xFF, 0x01 },
        { 0x31, 0x04 }, { 0x4B, 0x09 }, { 0x4C, 0x05 }, { 0x4D, 0x04 }, { 0xFF, 0x00 },
        { 0x44, 0x00 }, { 0x45, 0x20 }, { 0x47, 0x08 }, { 0x48, 0x28 }, { 0x67, 0x00 },
        { 0x70, 0x04 }, { 0x71, 0x01 }, { 0x72, 0xFE }, { 0x76, 0x00 }, { 0x77, 0x00 },
        { 0xFF, 0x01 }, { 0x0D, 0x01 }, { 0xFF, 0x00 }, { 0x80, 0x01 }, { 0x01, 0xF8 },
        { 0xFF, 0x01 }, { 0x8E, 0x01 }, { 0x00, 0x01 }, { 0xFF, 0x00 }, { 0x80, 0x00 }
      };
      for (uint8_t i = 0; i < sizeof(initRegisters) / sizeof(initRegisters[0]); ++i) {
        if (!writeReg(initRegisters[i][0], initRegisters[i][1])) return false;
      }

      if (!writeReg(RegisterSystemInterruptConfigGpio, 0x04)) return false;
      if (!writeReg(RegisterGpioHvMuxActiveHigh, (uint8_t)(readReg(RegisterGpioHvMuxActiveHigh) & ~0x10))) return false;
      if (!writeReg(RegisterSystemInterruptClear, 0x01)) return false;
      if (!writeReg(RegisterSystemSequenceConfig, 0xE8)) return false;
      measurementTimingBudgetUs = 33000UL;

      if (!performSingleRefCalibration(0x40)) return false;
      if (!writeReg(RegisterSystemSequenceConfig, 0x02)) return false;
      if (!performSingleRefCalibration(0x00)) return false;
      return writeReg(RegisterSystemSequenceConfig, 0xE8);
    }

    // Store the requested measurement timing budget.
    bool setMeasurementTimingBudget(uint32_t budgetUs) {
      if (budgetUs < 20000UL) return false;
      measurementTimingBudgetUs = budgetUs;
      return true;
    }

    // Start continuous ranging with the requested inter-measurement period.
    void startContinuous(uint32_t periodMs = 0) {
      didTimeout = false;
      lastReadValid = false;

      writeReg(0x80, 0x01);
      writeReg(0xFF, 0x01);
      writeReg(0x00, 0x00);
      writeReg(0x91, stopVariable);
      writeReg(0x00, 0x01);
      writeReg(0xFF, 0x00);
      writeReg(0x80, 0x00);

      if (periodMs != 0) {
        uint16_t oscCalibrateVal = readReg16Bit(RegisterOscCalibrateVal);
        if (oscCalibrateVal != 0) {
          periodMs *= oscCalibrateVal;
        }
        writeReg32Bit(RegisterSystemIntermeasurementPeriod, periodMs);
        writeReg(RegisterSysrangeStart, 0x04);
      } else {
        writeReg(RegisterSysrangeStart, 0x02);
      }
    }

    uint16_t readRangeContinuousMillimeters() {
      didTimeout = false;
      startTimeout();
      while ((readReg(RegisterResultInterruptStatus) & 0x07) == 0) {
        if (hasTimedOut()) {
          didTimeout = true;
          lastReadValid = false;
          return 65535;
        }
      }

      uint16_t range = readReg16Bit(RegisterResultRangeStatus + 10);
      writeReg(RegisterSystemInterruptClear, 0x01);
      lastReadValid = range != 0 && range != 65535;
      return range;
    }

    bool timeoutOccurred() {
      bool timedOut = didTimeout;
      didTimeout = false;
      return timedOut;
    }

    bool lastRangeReadValid() const {
      return lastReadValid;
    }

   private:
    void startTimeout() {
      timeoutStartMs = millis();
    }

    bool hasTimedOut() const {
      return ioTimeoutMs > 0 && (uint16_t)(millis() - timeoutStartMs) > ioTimeoutMs;
    }

    bool writeReg(uint8_t reg, uint8_t value) {
      return writeRegAt(address, reg, value);
    }

    bool writeRegAt(uint8_t targetAddress, uint8_t reg, uint8_t value) {
      Wire.beginTransmission(targetAddress);
      Wire.write(reg);
      Wire.write(value);
      return Wire.endTransmission() == 0;
    }

    bool writeReg16Bit(uint8_t reg, uint16_t value) {
      Wire.beginTransmission(address);
      Wire.write(reg);
      Wire.write((uint8_t)(value >> 8));
      Wire.write((uint8_t)value);
      return Wire.endTransmission() == 0;
    }

    bool writeReg32Bit(uint8_t reg, uint32_t value) {
      Wire.beginTransmission(address);
      Wire.write(reg);
      Wire.write((uint8_t)(value >> 24));
      Wire.write((uint8_t)(value >> 16));
      Wire.write((uint8_t)(value >> 8));
      Wire.write((uint8_t)value);
      return Wire.endTransmission() == 0;
    }

    uint8_t readReg(uint8_t reg) {
      uint8_t value = 0;
      readMulti(reg, &value, 1);
      return value;
    }

    uint16_t readReg16Bit(uint8_t reg) {
      uint8_t data[2] = { 0, 0 };
      readMulti(reg, data, sizeof(data));
      return ((uint16_t)data[0] << 8) | data[1];
    }

    bool writeMulti(uint8_t reg, const uint8_t* data, uint8_t length) {
      Wire.beginTransmission(address);
      Wire.write(reg);
      for (uint8_t i = 0; i < length; ++i) {
        Wire.write(data[i]);
      }
      return Wire.endTransmission() == 0;
    }

    bool readMulti(uint8_t reg, uint8_t* data, uint8_t length) {
      Wire.beginTransmission(address);
      Wire.write(reg);
      if (Wire.endTransmission(false) != 0) return false;

      uint8_t bytesRead = Wire.requestFrom((int)address, (int)length);
      if (bytesRead != length) return false;

      for (uint8_t i = 0; i < length; ++i) {
        data[i] = (uint8_t)Wire.read();
      }
      return true;
    }

    bool setSignalRateLimit(float limitMcps) {
      if (limitMcps < 0.0f || limitMcps > 511.99f) return false;
      return writeReg16Bit(RegisterFinalRangeConfigMinCountRateRtnLimit, (uint16_t)(limitMcps * 128.0f));
    }

    bool getSpadInfo(uint8_t* count, bool* typeIsAperture) {
      if (!writeReg(0x80, 0x01)) return false;
      if (!writeReg(0xFF, 0x01)) return false;
      if (!writeReg(0x00, 0x00)) return false;
      if (!writeReg(0xFF, 0x06)) return false;
      if (!writeReg(0x83, (uint8_t)(readReg(0x83) | 0x04))) return false;
      if (!writeReg(0xFF, 0x07)) return false;
      if (!writeReg(0x81, 0x01)) return false;
      if (!writeReg(0x80, 0x01)) return false;
      if (!writeReg(0x94, 0x6B)) return false;
      if (!writeReg(0x83, 0x00)) return false;

      startTimeout();
      while (readReg(0x83) == 0x00) {
        if (hasTimedOut()) return false;
      }

      if (!writeReg(0x83, 0x01)) return false;
      uint8_t value = readReg(0x92);
      *count = value & 0x7F;
      *typeIsAperture = ((value >> 7) & 0x01) != 0;

      if (!writeReg(0x81, 0x00)) return false;
      if (!writeReg(0xFF, 0x06)) return false;
      if (!writeReg(0x83, (uint8_t)(readReg(0x83) & ~0x04))) return false;
      if (!writeReg(0xFF, 0x01)) return false;
      if (!writeReg(0x00, 0x01)) return false;
      if (!writeReg(0xFF, 0x00)) return false;
      return writeReg(0x80, 0x00);
    }

    bool performSingleRefCalibration(uint8_t vhvInitByte) {
      if (!writeReg(RegisterSysrangeStart, (uint8_t)(0x01 | vhvInitByte))) return false;

      startTimeout();
      while ((readReg(RegisterResultInterruptStatus) & 0x07) == 0) {
        if (hasTimedOut()) return false;
      }

      if (!writeReg(RegisterSystemInterruptClear, 0x01)) return false;
      return writeReg(RegisterSysrangeStart, 0x00);
    }
  };

  // Train LEDs are routed through an MCP23008 on the shared I2C bus.
  // MCP23X08 / color sensor wiring summary:
  //   Nano A4/SDA -> MCP23008 SDA and TCS34725 SDA
  //   Nano A5/SCL -> MCP23008 SCL and TCS34725 SCL
  //   Nano 5V     -> MCP23008 VCC and TCS34725 VIN/VCC
  //   Nano GND    -> MCP23008 GND and TCS34725 GND
  //   MCP23008 address pins A0/A1/A2 -> GND so the expander stays at I2C address 0x20
  //   MCP23008 RESET -> pull HIGH (10 kOhm to 5V preferred; direct tie to 5V also works)
  //   MCP23008 VDD/GND -> place a 100 nF ceramic decoupling capacitor close to the chip
  //   SDA/SCL     -> need one effective I2C pull-up pair on the whole bus (typically 4.7 kOhm to 5V)
  //                  add pull-ups only if the connected breakouts/modules do not already provide them
  //   MCP23008 INT -> optional in this revision; leave unconnected if not used
  //   Nano D4     -> TCS34725 LED pin for breakout illumination control
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
  const uint8_t mcpAddressTrainLeds = 0x20;
  TrainLedMCP23008 trainLedExpander;
  bool trainLedExpanderDetected = false;
  const LedRoute led1R = { 0 };
  const LedRoute led1G = { 1 };
  const LedRoute led1B = { 2 };
  const LedRoute led2R = { 3 };
  const LedRoute led2G = { 4 };
  const LedRoute led2B = { 5 };
  const LedRoute ledGreen = { 6 };

  // D4 is dedicated to the TCS34725 breakout LED control input in this revision.
  const int pinColorSensorLED = 4;
  const uint8_t colorSensorLEDOnLevel = HIGH;
  const uint8_t colorSensorLEDOffLevel = LOW;
  TrainColorSensorTCS34725 colorSensor;
  bool colorSensorDetected = false;

  // ================================================================================================
  // TCS34725 Low-Power Sleep / Power-Down Notes:
  // - The sensor IC has an internal sleep/power-down state (~1-2 uA) controlled via I2C:
  //     colorSensor.disable();  // Clears PON and AEN bits -> puts sensor core to sleep (~2 uA)
  //     colorSensor.enable();   // Re-enables oscillator and RGBC ADC (~235 uA active without LED)
  // - Turning off the onboard LED via D4 saves ~15-20 mA (the dominant current draw).
  // - For maximum power savings when idle / sleeping:
  //     1. In setColorSensorEnabled(false) / goToIdle():
  //        digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);
  //        if (colorSensorDetected) colorSensor.disable();
  //     2. In setColorSensorEnabled(true) / wake up:
  //        if (colorSensorDetected) colorSensor.enable();
  //        digitalWrite(pinColorSensorLED, colorSensorLEDOnLevel);
  // ================================================================================================

  const int pinBuzzer = 12;                                // Passive buzzer (driven by tone() for melodies and siren)

  // ================================================================================================
  // Buzzer sound patterns
  // ================================================================================================
  const int pattern_melody[] = { 150, 80, 200, 80, 250, 80, 300, 150, 250, 0 };
  const int pattern_batteryWarn[] = { 3000, 100, 0 };
  const int pattern_double[] = { 150, 100, 150, 0 };
  const int pattern_descend[] = { 120, 80, 120, 80, 120, 0 };
  const int pattern_horn[] = { 1000, 100, 0 };
  const int pattern_tiltBeep[] = { 500, 0 };  // single 0.5s beep

  // ================================================================================================
  // Other constants
  // ================================================================================================
  const unsigned long DIR_DELAY = 1000;    // delay before motor direction change, ms
  const float R1 = 100000.0;               // Top resistor (to battery +)
  const float R2 = 10000.0;                // Bottom resistor (to GND)
  const float BATTERY_ADC_CORRECTION_FACTOR = 1.01;
  const int BATTERY_ADC_MAX = 1023;
  const float BATTERY_ADC_REF_VOLTAGE = 1.1;
  const uint8_t BATTERY_ADC_SAMPLES = 8;
  const float batteryPercentVoltTable[] = { 8.40, 8.20, 8.05, 7.90, 7.75, 7.60, 7.45, 7.35, 7.25, 7.20, 7.15 };
  const uint8_t batteryPercentTableSize = sizeof(batteryPercentVoltTable) / sizeof(batteryPercentVoltTable[0]);
  const int AUTO_SAMPLES_FOR_MEDIAN = 5;   // number of samples for median filter
  const int AUTO_DISTANCE_STOP = 8;        // distance to obstacle <= cm to stop the train
  const int AUTO_DISTANCE_RESTART = 11;    // distance to obstacle >= cm to re-start the train
  const int AUTO_DISTANCE_MAX_SPEED = 50;  // distance to obstacle >= cm to run at max speed
  const float BATTERY_LOW_WARNING = 7.25;  // warn at this voltage (judged only while stopped)
  const float BATTERY_LOW_SHUTDOWN = 7.15; // force stop at this voltage (judged only while stopped)
  const unsigned long BATTERY_CHECK_INTERVAL_MS = 5000;  // minimum gap between low-battery checks
  const float MAX_SAFE_MOTOR_VOLTAGE = 6.0;

  // ================================================================================================
  // Control variables
  // ================================================================================================
  const int FrontLightOnOff = 1;  // Front lights always on in this revision (no toggle button)
  bool SoundOnOff = true;
  bool AutoDistanceOnOff = false;
  bool ColorSensorOnOff = false;
  uint8_t MotorDirection = 1;   // always has a direction
  uint8_t Speed = 0;            // stopped at start
  uint8_t Distance = 0;         // Latest distance from the VL53L0X

  // --- Distance median filter state ---
  uint8_t distanceBuffer[AUTO_SAMPLES_FOR_MEDIAN];
  uint8_t bufferIndex = 0;
  bool bufferFilled = false;

  // --- Motor driver fault state ---
  bool motorFaultLatched = false;
  bool motorDriveAttemptedSinceFault = true;  // Track if motor run was attempted after fault
  #if ENABLE_EEPROM_LOGGING
  uint8_t currentBootSlot = 0xFF;  // Ring buffer slot index for current boot session
  uint8_t bootFaultCount = 0;      // Motor fault counter for current power-on session (0..16)
  #endif

  // --- Momentary jog state ---
  bool momentaryActive = false;
  uint8_t momentaryButton = 0;
  unsigned long momentaryLastSeen = 0;
  const unsigned long momentaryTimeout = 200;  // ms after last repeat → stop

  // --- IR repeat tracking ---
  uint8_t lastIRCommand = 0;
  bool lastWasRepeat = false;

  // Timers
  unsigned long lastActive = 0;
  const unsigned long idleTimeout = 5UL * 60UL * 1000UL;  // 5 minutes
  unsigned long lastColorSensorRead = 0;
  const unsigned long colorSensorReadEveryMs = 60;
  uint8_t lastTrackMarkerClass = 255;
  unsigned long lastBatteryDebugPrintMs = 0;
  unsigned long lastBatteryCheckMs = 0;

  // Non-blocking green status-LED acknowledgement blink
  uint8_t greenBlinkRemaining = 0;
  bool greenBlinkOn = false;
  unsigned long greenBlinkStepMs = 0;
  const unsigned long greenBlinkOnMs = 100;
  const unsigned long greenBlinkOffMs = 50;

  // Non-blocking reverse-direction cooldown
  unsigned long motorReverseReadyAt = 0;
  bool motorDrivePending = false;

  // ================================================
  // BEGIN: generated from rgb-color-sensor\calibration_tool.py analyze-clusters
  // Source artifact: rgb-color-sensor\data\generated-clusters.txt
  // Mounting: aim for ~3 mm sensor-to-marker gap (usable 2-5 mm), fixed height and angle, LED on.
  // ================================================
  const uint16_t colorClearMinThreshold = 160;
  const uint16_t colorMatchClearThreshold = 304;
  const float whiteBalanceRedGain = 1.00f;
  const float whiteBalanceGreenGain = 1.212f;
  const float whiteBalanceBlueGain = 2.318f;

  const MarkerClusterDefinition markerClusters[] = {
    { MarkerWhite,   { 334, 333, 333 }, 32 },
    { MarkerBrown,   { 392, 322, 287 }, 31 },
    { MarkerBrown,   { 415, 305, 280 }, 22 },
    { MarkerBrown,   { 428, 289, 284 }, 25 },
    { MarkerCyan,    { 158, 313, 529 }, 60 },
    { MarkerGreen,   { 160, 518, 322 }, 62 },
    { MarkerGreen,   { 203, 482, 315 }, 52 },
    { MarkerGrey,    { 303, 345, 352 }, 38 },
    { MarkerGrey,    { 327, 331, 342 }, 22 },
    { MarkerGrey,    { 347, 317, 336 }, 27 },
    { MarkerMagenta, { 455, 201, 344 }, 14 },
    { MarkerMagenta, { 476, 193, 331 }, 18 },
    { MarkerMagenta, { 488, 186, 326 }, 10 },
    { MarkerOrange,  { 542, 254, 204 }, 22 },
    { MarkerRed,     { 514, 180, 305 }, 27 },
    { MarkerRed,     { 550, 191, 259 }, 36 },
    { MarkerRed,     { 572, 176, 252 }, 22 },
    { MarkerYellow,  { 420, 368, 212 }, 14 },
    { MarkerYellow,  { 432, 353, 215 }, 18 }
  };
  const uint8_t markerClusterCount = sizeof(markerClusters) / sizeof(markerClusters[0]);
  // ================================================
  // END: generated from rgb-color-sensor\calibration_tool.py analyze-clusters
  // ================================================

  // Actions run only on color transitions, so a long marker does not keep retriggering.
  // To disable a color's action, comment out that case in handleTrackMarkerAction().
  // Action bodies are placeholders for now - define each behavior in handleTrackMarkerAction().

  // Speed steps (voltage → PWM)
  uint8_t pwmSteps[4];  // 0..255
  float voltageSteps[] = { 0.0, 3.5, 4.5, 6.0 };
  float batteryVoltage = 0.0;

  TrainDistanceSensorVL53L0X distanceTof;
  bool distanceTofDetected = false;
  unsigned long lastTofReadMs = 0;
  const uint16_t distanceTofTimeoutMs = 50;
  const uint32_t distanceTofTimingBudgetUs = 33000UL;
  const uint32_t distanceTofContinuousPeriodMs = 50UL;
  const unsigned long tofReadEveryMs = 50;

  // Buzzer
  #define BUZZER_PATTERN_MAX 20
  int buzzerPattern[BUZZER_PATTERN_MAX];
  int buzzerIndex = 0;
  unsigned long buzzerTimer = 0;

  // Manual step index
  uint8_t currentStep = 0;  // 0=stop, 1=~3.5V, 2=~4.5V, 3=~6V

  // Optional future upgrade: measure motor current with a shunt + amplifier/monitor.
  // Use this for load-aware behavior or soft limits; nFAULT only reports hard driver faults.
  // // Current sense
  // const int pinMotorSense = A2;   // shunt resistor voltage
  // const float shuntResistor = 0.1; // ohms
  // const float maxSafeCurrent = 0.8; // amps (cutoff threshold)

  // // Overcurrent detection
  // unsigned long overCurrentStart = 0;
  // bool overCurrentActive = false;

  // Siren state
  bool sirenActive = false;
  unsigned long sirenTimer = 0;
  int sirenPhase = 0;  // 0=LED1 red, 1=LED1 blue

  // Siren timing (time-based sweep → stable even with loop jitter)
  const int sirenFmin = 400;
  const int sirenFmax = 800;
  const unsigned long sirenSweepMs = 800;  // up in 800 ms, down in 800 ms
  unsigned long sirenStartMs = 0;          // set when siren toggles ON

  // ── Tilt sensor debounce config/state ─────────────────────────────────────────
  const unsigned long TILT_STABLE_MS = 1000;  // must hold this long to confirm state
  const unsigned long TILT_QUIET_MS = 500;   // ignore flips for a short time after change
  int tiltStableState = HIGH;                // external pull-up: OPEN=HIGH (idle)
  int tiltLastRead = HIGH;
  unsigned long tiltEdgeAt = 0;
  unsigned long tiltQuietUntil = 0;
  bool tiltStopLatched = false;

  // ── Tone melody player state ────────────────────────────────────────────────
  const int MELODY_MAX_PAIRS = 64;        // up to 64 (freq,dur) pairs
  const int16_t* melodySrc = nullptr;     // pointer to the active flat [f,d,f,d,...] sequence
  bool melodySrcIsProgmem = false;        // true if melodySrc points into PROGMEM
  int melodyLenPairs = 0;               // number of (f,d) pairs loaded
  int melodyIdxPair = 0;                // current pair index
  bool melodyLoop = false;              // loop playback
  bool melodyPlaying = false;           // active?
  unsigned long melodyStepStarted = 0;  // ms when current note started

  // ================================================================================================
  // Setup
  // ================================================================================================
  // Bring up every hardware block once, then seed the initial runtime state.
  // Detect and initialize the MCP23008 LED expander.
  void initTrainLedHardware() {
    trainLedExpanderDetected = trainLedExpander.begin_I2C(mcpAddressTrainLeds);
    if (!trainLedExpanderDetected) {
      DBGLN_LEDS(F("MCP23008 not found; train LEDs expect the expander wiring"));
      return;
    }

    const LedRoute routes[] = { led1R, led1G, led1B, led2R, led2G, led2B, ledGreen };
    for (uint8_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
      trainLedExpander.pinMode(routes[i].expanderPin, OUTPUT);
      trainLedExpander.digitalWrite(routes[i].expanderPin, LOW);
    }

    DBGLN_LEDS(F("MCP23008 ready for train LEDs"));
  }

  // Configure the TCS34725 and its onboard lamp control.
  void initColorSensorHardware() {
    pinMode(pinColorSensorLED, OUTPUT);
    digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);

    colorSensorDetected = colorSensor.begin_I2C();
    if (colorSensorDetected) {
      colorSensor.disable();  // Keep the sensor IC core in low-power sleep (~2 uA) until enabled
      DBGLN_COLOR_SENSOR(F("TCS34725 ready (in sleep mode)"));
    } else {
      DBGLN_COLOR_SENSOR(F("TCS34725 not detected on I2C"));
    }
  }

  // Select the internal 1.1V ADC reference.
  void initBatteryVoltageMeterHardware() {
    analogReference(INTERNAL);
    delay(5);  // Let the internal 1.1V reference settle before discarding the first conversion.
    analogRead(pinBatterySense);
  }

  // Toggle color-sensor mode and related status lighting.
  void setColorSensorEnabled(bool enabled) {
    if (enabled && !colorSensorDetected) {
      DBGLN_COLOR_SENSOR(F("Ignored: TCS34725 not detected"));
      return;
    }

    ColorSensorOnOff = enabled ? 1 : 0;
    digitalWrite(pinColorSensorLED, enabled ? colorSensorLEDOnLevel : colorSensorLEDOffLevel);

    if (enabled) {
      if (colorSensorDetected) colorSensor.enable();
      DBGLN_COLOR_SENSOR(F("Color sensor ON"));
      if (!sirenActive) SetRGBLightColor(RgbColor::Cyan);
    } else {
      if (colorSensorDetected) colorSensor.disable();
      DBGLN_COLOR_SENSOR(F("Color sensor OFF"));
      refreshDriveLights();
    }
  }

  // Apply per-channel gain correction to raw RGBC data.
  BalancedRgbs applyWhiteBalance(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    BalancedRgbs balanced;
    balanced.r = (uint16_t)(r * whiteBalanceRedGain + 0.5f);
    balanced.g = (uint16_t)(g * whiteBalanceGreenGain + 0.5f);
    balanced.b = (uint16_t)(b * whiteBalanceBlueGain + 0.5f);
    balanced.c = c;
    return balanced;
  }

  // Convert RGB counts to a 0-1000 normalized triple.
  PrototypeRgb normalizePrototype(uint16_t r, uint16_t g, uint16_t b) {
    PrototypeRgb prototype = { 0, 0, 0 };
    uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;
    if (sum == 0) return prototype;

    prototype.r = (uint16_t)(((uint32_t)r * 1000UL + (sum / 2)) / sum);
    prototype.g = (uint16_t)(((uint32_t)g * 1000UL + (sum / 2)) / sum);
    prototype.b = (uint16_t)(((uint32_t)b * 1000UL + (sum / 2)) / sum);
    return prototype;
  }

  // Manhattan distance between two normalized RGB prototypes.
  uint16_t prototypeDistance(const PrototypeRgb& a, const PrototypeRgb& b) {
    uint16_t distance = 0;
    distance += (a.r > b.r) ? (a.r - b.r) : (b.r - a.r);
    distance += (a.g > b.g) ? (a.g - b.g) : (b.g - a.g);
    distance += (a.b > b.b) ? (a.b - b.b) : (b.b - a.b);
    return distance;
  }

  // Convert raw RGBC values into track marker classes.
  uint8_t classifyTrackMarkerColor(uint16_t rawR, uint16_t rawG, uint16_t rawB, uint16_t rawC) {
    if (rawC < colorClearMinThreshold || rawC < colorMatchClearThreshold) return MarkerUnknown;

    BalancedRgbs balanced = applyWhiteBalance(rawR, rawG, rawB, rawC);
    PrototypeRgb measured = normalizePrototype(balanced.r, balanced.g, balanced.b);
    if (measured.r == 0 && measured.g == 0 && measured.b == 0) return MarkerUnknown;

    uint16_t bestDistance = 0xFFFF;
    TrackMarkerClass bestClass = MarkerUnknown;

    for (uint8_t i = 0; i < markerClusterCount; ++i) {
      uint16_t distance = prototypeDistance(measured, markerClusters[i].center);
      if (distance <= markerClusters[i].maxDistance && distance < bestDistance) {
        bestDistance = distance;
        bestClass = markerClusters[i].markerClass;
      }
    }

    return (uint8_t)bestClass;
  }

  #if DEBUG || DEBUG_COLOR_SENSOR
  // Debug-only marker name lookup.
  const __FlashStringHelper* trackMarkerLabel(uint8_t markerClass) {
    switch (markerClass) {
      case MarkerWhite: return F("white");
      case MarkerBrown: return F("brown");
      case MarkerCyan: return F("cyan");
      case MarkerGreen: return F("green");
      case MarkerGrey: return F("grey");
      case MarkerMagenta: return F("magenta");
      case MarkerOrange: return F("orange");
      case MarkerYellow: return F("yellow");
      case MarkerRed: return F("red");
      default: return F("unknown");
    }
  }
  #endif

  // Run the action associated with a detected marker color.
  void handleTrackMarkerAction(uint8_t markerClass) {
    switch (markerClass) {
      case MarkerWhite:
        DBGLN_COLOR_SENSOR(F("Track action: WHITE marker (no action defined yet)"));
        break;

      case MarkerBrown:
        DBGLN_COLOR_SENSOR(F("Track action: BROWN marker (no action defined yet)"));
        break;

      case MarkerCyan:
        DBGLN_COLOR_SENSOR(F("Track action: CYAN marker (no action defined yet)"));
        break;

      case MarkerRed:
        DBGLN_COLOR_SENSOR(F("Track action: RED marker (no action defined yet)"));
        break;

      case MarkerGreen:
        DBGLN_COLOR_SENSOR(F("Track action: GREEN marker (no action defined yet)"));
        break;

      case MarkerGrey:
        DBGLN_COLOR_SENSOR(F("Track action: GREY marker (no action defined yet)"));
        break;

      case MarkerMagenta:
        DBGLN_COLOR_SENSOR(F("Track action: MAGENTA marker (no action defined yet)"));
        break;

      case MarkerOrange:
        DBGLN_COLOR_SENSOR(F("Track action: ORANGE marker (no action defined yet)"));
        break;

      case MarkerYellow:
        DBGLN_COLOR_SENSOR(F("Track action: YELLOW marker (no action defined yet)"));
        break;

      default:
        break;
    }
  }

  // Poll and process the color sensor without blocking.
  void updateColorSensor() {
    if (ColorSensorOnOff == 0 || !colorSensorDetected) return;

    unsigned long now = millis();
    if (now - lastColorSensorRead < colorSensorReadEveryMs) return;
    lastColorSensorRead = now;

    uint16_t r = 0, g = 0, b = 0, c = 0;
    colorSensor.readRawData(&r, &g, &b, &c);
    uint8_t markerClass = classifyTrackMarkerColor(r, g, b, c);
    #if DEBUG || DEBUG_COLOR_SENSOR
    const __FlashStringHelper* markerLabel = trackMarkerLabel(markerClass);
    DBG_COLOR_SENSOR(F("TCS34725 RGBC: "));
    DBG_COLOR_SENSOR(r);
    DBG_COLOR_SENSOR(F(", "));
    DBG_COLOR_SENSOR(g);
    DBG_COLOR_SENSOR(F(", "));
    DBG_COLOR_SENSOR(b);
    DBG_COLOR_SENSOR(F(", clear="));
    DBG_COLOR_SENSOR(c);
    DBG_COLOR_SENSOR(F(" -> "));
    DBGLN_COLOR_SENSOR(markerLabel);
    #endif

    if (markerClass != lastTrackMarkerClass) {
      lastTrackMarkerClass = markerClass;
      #if DEBUG || DEBUG_COLOR_SENSOR
      DBG_COLOR_SENSOR(F("Track marker changed to: "));
      DBGLN_COLOR_SENSOR(markerLabel);
      #endif
      handleTrackMarkerAction(markerClass);
    }
  }

  // Restore status lights from the current drive state.
  void refreshDriveLights() {
    if (sirenActive) return;

    SetGreenLightValue(AutoDistanceOnOff ? 255 : 0);

    if (ColorSensorOnOff == 1) {
      SetRGBLightColor(RgbColor::Cyan);
      return;
    }

    if (Speed == 0) SetRGBColor(RgbColor::Red);
    else if (MotorDirection == 2) SetRGBColor(RgbColor::Blue);
    else SetRGBColor(RgbColor::White);
  }

  // Disable auto-distance mode and clear its indicator.
  void exitAutoDistanceMode(bool clearIndicator) {
    AutoDistanceOnOff = false;
    if (clearIndicator) SetGreenLightValue(0);
  }

  // Stop motor and reset manual speed step to 0.
  void stopAndResetStepSelection(bool resetDirection) {
    Stop();
    currentStep = 0;
    if (resetDirection) MotorDirection = 1;
  }

  // Clear the momentary jog active flag.
  void cancelJog() {
    momentaryActive = false;
  }

  #if ENABLE_EEPROM_LOGGING
  // Log boot sequence number (2-byte timestamp), sensor error flags, battery voltage, and initial
  // fault count (0) to the EEPROM ring buffer. Must be called after initBatteryVoltageMeterHardware()
  // and after batteryVoltage has been set.
  // Log boot sequence, sensor status, and battery to EEPROM ring buffer.
  void writeBootErrorCodes() {
    // Increment the 2-byte global boot counter; treat uninitialized EEPROM (0xFFFF) as pre-boot 0.
    uint16_t bootSeq = 0;
    EEPROM.get(EEPROM_ADDR_BOOT_COUNT, bootSeq);
    bootSeq = (bootSeq == 0xFFFF) ? 1 : (uint16_t)(bootSeq + 1);
    EEPROM.put(EEPROM_ADDR_BOOT_COUNT, bootSeq);

    uint8_t flags = 0;
    if (!trainLedExpanderDetected) flags |= ERR_LED_EXPANDER;
    if (!colorSensorDetected)      flags |= ERR_COLOR_SENSOR;
    if (!distanceTofDetected)      flags |= ERR_DISTANCE_TOF;

    // Store voltage as tenths-of-volt (74 = 7.4 V); 0xFF = not measured.
    uint8_t battByte = (batteryVoltage > 0.0f)
                       ? (uint8_t)constrain((int)(batteryVoltage * 10.0f + 0.5f), 0, 254)
                       : 0xFF;

    // Resolve the current write slot; handle uninitialized EEPROM (0xFF) gracefully.
    uint8_t logHead = EEPROM.read(EEPROM_ADDR_LOG_HEAD);
    if (logHead >= EEPROM_BOOT_LOG_SIZE) logHead = 0;
    currentBootSlot = logHead;
    bootFaultCount = 0;  // Reset fault count for this new power-on session

    uint16_t entryAddr = EEPROM_ADDR_LOG_BASE + (uint16_t)logHead * EEPROM_ENTRY_SIZE;
    EEPROM.put(entryAddr,        bootSeq);
    EEPROM.update(entryAddr + 2, flags);
    EEPROM.update(entryAddr + 3, battByte);
    EEPROM.update(entryAddr + 4, 0);  // Initial fault count is 0 on boot
    EEPROM.update(EEPROM_ADDR_LOG_HEAD, (uint8_t)((logHead + 1) % EEPROM_BOOT_LOG_SIZE));

    #if DEBUG_ANY
    DBG_LEDS(F("Boot #")); DBG_LEDS(bootSeq);
    DBG_LEDS(F("  flags=0x")); DBG_LEDS(flags, HEX);
    DBG_LEDS(F("  batt="));
    if (battByte == 0xFF) { DBGLN_LEDS(F("n/a")); }
    else { DBG_LEDS(battByte / 10); DBG_LEDS(F(".")); DBGLN_LEDS(battByte % 10); }
    if (flags & ERR_LED_EXPANDER)  DBGLN_LEDS(F("  ERR 0x01: MCP23008 LED expander not detected"));
    if (flags & ERR_COLOR_SENSOR)  DBGLN_LEDS(F("  ERR 0x02: TCS34725 color sensor not detected"));
    if (flags & ERR_DISTANCE_TOF)  DBGLN_LEDS(F("  ERR 0x04: VL53L0X distance sensor not detected"));
    if (flags == 0)                DBGLN_LEDS(F("  All sensors OK"));
    #endif

    if (flags != 0) {
      // Three rapid beeps bypass SoundOnOff — this is a safety notification, not user audio.
      for (uint8_t i = 0; i < 3; ++i) { tone(pinBuzzer, 1500, 80); delay(150); }
      noTone(pinBuzzer);
    }
  }
  #endif

  // One-time hardware initialization and startup behavior.
  // Initialize hardware and startup state.
  void setup() {

    // Hardware reset the VL53L0X to allow address change and avoid conflict with TCS34725.
    pinMode(pinVL53L0X_XSHUT, OUTPUT);
    digitalWrite(pinVL53L0X_XSHUT, LOW); // Hold VL53L0X in hardware reset
    delay(10);                           // Give it time to completely power down
    digitalWrite(pinVL53L0X_XSHUT, HIGH); // Wake up the VL53L0X
    delay(10);                           // Wait for it to boot before sending I2C commands


    DBGBEGIN(115200);

    // Initialize Arduino pins
    initTrainLedHardware();
    pinMode(pinBuzzer, OUTPUT);
    pinMode(pinMotor_IN1, OUTPUT);
    pinMode(pinMotor_IN2, OUTPUT);
    pinMode(pinMotorFault, INPUT_PULLUP);
    pinMode(pinMotorSleep, OUTPUT);
    digitalWrite(pinMotorSleep, HIGH);  // Enable the DRV8833 after power-up.
    //pinMode(pinIRReceiver, INPUT);
    pinMode(pinBatterySense, INPUT);
    pinMode(pinTiltSensor, INPUT_PULLUP);
    initBatteryVoltageMeterHardware();
    initDistanceSensorHardware();
    initColorSensorHardware();

    // Battery must be measured before writeBootErrorCodes() so the voltage is included in the log.
    batteryVoltage = getBatteryVoltageDirect();
    DBG_VOLTAGE_METER(F("Battery measured: "));
    DBGLN_VOLTAGE_METER(batteryVoltage, 2);

    #if ENABLE_EEPROM_LOGGING
    // Log boot sequence, sensor status, and battery voltage to EEPROM (setup only, never in loop).
    writeBootErrorCodes();
    #endif

    //playPattern(pattern_melody);  // Play melody
    SetGreenLightValue(0);
    SetRGBColor(FrontLightOnOff ? RgbColor::Red : RgbColor::Off);
    playToneSequence_P(melodyDemo, false);

    // Configure dynamic speed steps
    configureSpeedSteps();

    lastActive = millis();            // seed idle timer
    IrReceiver.begin(pinIRReceiver, DISABLE_LED_FEEDBACK);  // Keep D13 dedicated to DRV8833 nSLEEP.
  }

  // ================================================================================================
  // Main Loop
  // ================================================================================================
  // Cooperative scheduler: each block owns one concern and must stay fast/non-blocking.
  // Main scheduler for safety, input, sensing, and playback.
  void loop() {

    // === 1. DRV8833 fault watchdog ===
    updateMotorFault();

    // === 2. Idle timeout watchdog ===
    if (millis() - lastActive > idleTimeout) {
      goToIdle();
    }

    // === 2b. Low-battery guard (only judged while stopped) ===
    updateBatteryGuard();

    // === 3. Jog watchdog (safety if button released) ===
    if (!motorFaultLatched && momentaryActive && (millis() - momentaryLastSeen > momentaryTimeout)) {
      DBGLN_MOTOR(F("Jog watchdog timeout -> STOP"));
      Stop();
      SetRGBColor(RgbColor::Red);
      momentaryActive = false;
    }

    // === 4. Auto-speed mode ===
    if (!motorFaultLatched && AutoDistanceOnOff == 1) {
      updateAutoDistanceSpeed();
    }

    // === 5. Tilt sensor ===
    updateTiltSensor();

    // === 6. IR remote handler ===
    translateIR();

    // === 7. Color sensor handler ===
    updateColorSensor();

    // Non-blocking playback ===
    updateBuzzer();
    updateSiren();
    updateMelody();
    updateGreenBlink();
    updateMotorReverseCooldown();

  }

  // ================================================================================================
  // Idle / Sleep
  // ================================================================================================
  // Enter the lowest-power idle mode after an extended period without user activity.
  // Shut down outputs and enter low-power sleep.
  void goToIdle() {
    DBGLN_IR_REMOTE(F("Idle: turning everything off..."));

    digitalWrite(pinBuzzer, LOW);
    buzzerPattern[0] = 0;
    buzzerIndex = 0;

    setColorSensorEnabled(false);
    SetRGBColor(RgbColor::Off);
    SetGreenLightValue(0);
    Stop();

    playPattern(pattern_descend);
    delay(2000);  // Blocking: lets the shutdown jingle finish before sleep; safe while powering down.
    digitalWrite(pinBuzzer, LOW);

    DBGLN_IR_REMOTE(F("Entering sleep mode..."));

    digitalWrite(pinMotorSleep, LOW);  // Disable the DRV8833 while the Arduino sleeps.
    digitalWrite(pinVL53L0X_XSHUT, LOW); // Turn off VL53L0X for sleep
    attachInterrupt(digitalPinToInterrupt(pinIRReceiver), wakeUp, CHANGE);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(digitalPinToInterrupt(pinIRReceiver));

    digitalWrite(pinVL53L0X_XSHUT, HIGH); // Wake up VL53L0X
    delay(10); // boot time
    startDistanceSensorRanging();

    digitalWrite(pinMotorSleep, HIGH);
    delay(1);  // DRV8833 wake-up time before checking its diagnostic output.
    initBatteryVoltageMeterHardware();  // ADC was powered down; re-establish the 1.1V reference.
    lastActive = millis();
  }

  // Interrupt callback used only to wake the MCU.
  void wakeUp() {}

  // Measure battery voltage directly under the current load.
  // Read pack voltage through the resistor divider.
  float getBatteryVoltageDirect() {
    float sum = 0.0;
    const float dividerRatio = (R1 + R2) / R2;  // constant for this board
    #if DEBUG || DEBUG_VOLTAGE_METER
    unsigned long rawSum = 0;
    #endif

    // Throw away the first conversion so the ADC sampling capacitor settles on the battery divider.
    analogRead(pinBatterySense);

    for (uint8_t sampleIndex = 0; sampleIndex < BATTERY_ADC_SAMPLES; ++sampleIndex) {
      int rawValue = analogRead(pinBatterySense);
      #if DEBUG || DEBUG_VOLTAGE_METER
      rawSum += rawValue;
      #endif
      float pinVoltage = (rawValue * BATTERY_ADC_REF_VOLTAGE) / BATTERY_ADC_MAX;
      sum += pinVoltage * dividerRatio * BATTERY_ADC_CORRECTION_FACTOR;
    }

    float averagedVoltage = sum / BATTERY_ADC_SAMPLES;

    #if DEBUG || DEBUG_VOLTAGE_METER
    unsigned long now = millis();
    if (now - lastBatteryDebugPrintMs >= 1000UL) {
      lastBatteryDebugPrintMs = now;
      float averageRaw = rawSum / (float)BATTERY_ADC_SAMPLES;
      DBG_VOLTAGE_METER(F("Battery ADC avg raw="));
      DBG_VOLTAGE_METER(averageRaw, 1);
      DBG_VOLTAGE_METER(F(" -> V="));
      DBGLN_VOLTAGE_METER(averagedVoltage, 3);
    }
    #endif

    return averagedVoltage;
  }

  // Stop loads, let the pack settle, then measure.
  float getBatteryVoltageSettledForStatus() {
    exitAutoDistanceMode();
    cancelJog();
    stopAndResetStepSelection(true);
    Speed = 0;
    setColorSensorEnabled(false);
    sirenActive = false;
    stopMelody();
    noTone(pinBuzzer);
    clearBuzzerPattern();

    SetRGBColor(RgbColor::Off);
    SetGreenLightValue(0);
    Stop();

    delay(200);  // Blocking: let the pack settle unloaded before sampling for an accurate reading.
    return getBatteryVoltageDirect();
  }

  // Judge pack health only while stopped (no motor sag) and no more than once per interval.
  // Periodically check voltage and enforce low-battery limits.
  void updateBatteryGuard() {
    if (Speed != 0 || motorFaultLatched) return;
    unsigned long now = millis();
    if (now - lastBatteryCheckMs < BATTERY_CHECK_INTERVAL_MS) return;
    lastBatteryCheckMs = now;

    float v = getBatteryVoltageDirect();  // motor off => reading is sag-free
    if (v <= 0) return;
    batteryVoltage = v;

    if (v <= BATTERY_LOW_SHUTDOWN) {
      DBGLN_VOLTAGE_METER(F("Battery SHUTDOWN level: forcing stop"));
      setColorSensorEnabled(false);
      exitAutoDistanceMode(false);
      cancelJog();
      stopAndResetStepSelection();
      SetRGBColor(RgbColor::Red);
      playPattern(pattern_batteryWarn);
    } else if (v <= BATTERY_LOW_WARNING) {
      DBGLN_VOLTAGE_METER(F("Battery WARNING level"));
      setColorSensorEnabled(false);
      SetRGBColor(RgbColor::Red);
      playPattern(pattern_batteryWarn);
    }
  }

  // Map pack voltage to a 0-100% charge estimate.
  int get2SBatteryPercent(float voltage) {
    float maxVolt = batteryPercentVoltTable[0];
    float minVolt = batteryPercentVoltTable[batteryPercentTableSize - 1];

    if (voltage >= maxVolt) return 100;
    if (voltage <= minVolt) return 0;

    for (uint8_t i = 0; i < batteryPercentTableSize - 1; ++i) {
      float vUpper = batteryPercentVoltTable[i];
      float vLower = batteryPercentVoltTable[i + 1];
      if (voltage > vLower) {
        int pUpper = 100 - (i * 10);
        int pLower = pUpper - 10;
        return map((long)(voltage * 100), (long)(vLower * 100), (long)(vUpper * 100), pLower, pUpper);
      }
    }

    return 0;
  }

  // Convert a requested motor voltage into PWM based on the current pack voltage.
  // Convert a target motor voltage to a safe PWM value.
  int safePWMFromVoltage(float desiredMotorV) {
    batteryVoltage = getBatteryVoltageDirect();
    if (batteryVoltage <= 0) return 0;
    // Clamp to max 6 V effective
    float vm = min(desiredMotorV, MAX_SAFE_MOTOR_VOLTAGE);
    int pwm = (int)(255.0 * vm / batteryVoltage);
    return constrain(pwm, 0, 255);
  }

  // ================================================================================================
  // Manual speed (steps)
  // ================================================================================================
  // Precompute discrete manual speed steps so the remote can step through predictable speeds.
  // Build manual speed steps from live battery voltage.
  void configureSpeedSteps() {
    const int stepCount = sizeof(pwmSteps) / sizeof(pwmSteps[0]);
    for (int i = 0; i < stepCount; i++) {
      pwmSteps[i] = safePWMFromVoltage(voltageSteps[i]);
    }
    DBGLN_MOTOR(F("Configured speed steps (PWM values):"));
    for (int i = 0; i < stepCount; i++) {
      DBG_MOTOR(min(MAX_SAFE_MOTOR_VOLTAGE, voltageSteps[i]));
      DBG_MOTOR(F("V -> PWM "));
      DBGLN_MOTOR(pwmSteps[i]);
    }
  }

  // Audible confirmation for the selected manual step.
  void playStepBeep(int step) {
    if (SoundOnOff != 1) return;  // respect mute
    clearBuzzerPattern();
    int idx = 0;
    if (step == 0) {
      // no beep on stop
    } else {
      for (int i = 0; i < step; i++) {
        buzzerPattern[idx++] = 150;  // ON
        buzzerPattern[idx++] = 150;  // OFF
      }
    }
    buzzerPattern[idx] = 0;
    buzzerIndex = 0;
    buzzerTimer = millis();
    if (buzzerPattern[0] > 0) digitalWrite(pinBuzzer, HIGH);
  }

  // Apply the current manual step to the motor state.
  void applySpeedStep() {
    Speed = pwmSteps[currentStep];

    DBG_MOTOR(F("Step "));
    DBG_MOTOR(currentStep);
    DBG_MOTOR(F(": target "));
    DBG_MOTOR(voltageSteps[currentStep]);
    DBG_MOTOR(F("V -> PWM "));
    DBGLN_MOTOR(Speed);

    playStepBeep(currentStep);

    if (Speed == 0) Stop();
    else if (MotorDirection == 1) GoForward();
    else if (MotorDirection == 2) GoBackward();
  }

  // CH- and CH+ adjust the current drive step through these thin wrappers.
  // Move between manual drive steps.
  void increaseStep() {
    if (currentStep < 3) currentStep++;
    applySpeedStep();
  }
  // Move between manual drive steps.
  void decreaseStep() {
    if (currentStep > 0) currentStep--;
    applySpeedStep();
  }

  // ================================================================================================
  // Auto-distance speed control
  // ================================================================================================
  // Convert obstacle distance into a motor-voltage target with a dead zone near obstacles.
  // Map obstacle distance to a target motor voltage.
  float motorVoltageFromDistance(int distance) {
    if (distance < AUTO_DISTANCE_STOP) return 0.0;
    if (distance <= AUTO_DISTANCE_RESTART) return 0.0;

    int lastIdx = sizeof(voltageSteps) / sizeof(voltageSteps[0]) - 1;
    float minV = min(MAX_SAFE_MOTOR_VOLTAGE, voltageSteps[1]);        // ≈3.5V
    float maxV = min(MAX_SAFE_MOTOR_VOLTAGE, voltageSteps[lastIdx]);  // ≈6.0V

    if (distance < AUTO_DISTANCE_MAX_SPEED) {
      float spanV = (maxV - minV);
      float spanD = (float)(AUTO_DISTANCE_MAX_SPEED - AUTO_DISTANCE_RESTART);
      float rawV = minV + (distance - AUTO_DISTANCE_RESTART) * (spanV / spanD);
      return constrain(rawV, minV, maxV);
    }
    return maxV;  // ≥ 50 cm → full speed
  }

  // Poll distance, compute target speed, and hand off ramping to updateMotorSpeed().
  // Run the automatic speed controller.
  void updateAutoDistanceSpeed() {
    Distance = getDistanceReading();

    DBG_DISTANCE_SENSOR(F("Distance: "));
    DBG_DISTANCE_SENSOR(Distance);
    DBGLN_DISTANCE_SENSOR(F(" cm"));

    float targetV = motorVoltageFromDistance(Distance);
    int targetSpeed = safePWMFromVoltage(targetV);

    if (targetSpeed == 0) {
      SetRGBColor(RgbColor::Red);
      DBGLN_DISTANCE_SENSOR(F("Auto: STOP"));
    } else {
      SetRGBColor(RgbColor::White);
      DBG_DISTANCE_SENSOR(F("Auto target speed = "));
      DBG_DISTANCE_SENSOR(targetSpeed);
      DBG_DISTANCE_SENSOR(F(" (~ "));
      DBG_DISTANCE_SENSOR(targetV, 2);
      DBGLN_DISTANCE_SENSOR(F(" V)"));
    }

    updateMotorSpeed(targetSpeed);
  }

  // Map a remote button code to a melody and start playback.
  bool tryPlayMelodyForButton(uint8_t code) {
    switch (code) {
      case button1: playToneSequence_P(melodyDemo, false); return true;
      case button2: playToneSequence_P(melodyTwinkle, false); return true;
      case button3: playToneSequence_P(melodyOdeToJoy, false); return true;
      case button4: playToneSequence_P(melodyMary, false); return true;
      case button5: playToneSequence_P(melodyWheels, false); return true;
      case button6: playToneSequence_P(melodyHappy, false); return true;
      case button7: playToneSequence_P(melodyBabyShark, false); return true;
      case button8: playToneSequence_P(melodyJingle, false); return true;
      default: return false;
    }
  }

  // Insert a sample into the median filter and return result.
  int pushDistanceSampleAndGetMedian(int raw) {
    distanceBuffer[bufferIndex] = raw;
    bufferIndex = (bufferIndex + 1) % AUTO_SAMPLES_FOR_MEDIAN;
    if (bufferIndex == 0) bufferFilled = true;

    uint8_t size = bufferFilled ? AUTO_SAMPLES_FOR_MEDIAN : bufferIndex;
    uint8_t temp[AUTO_SAMPLES_FOR_MEDIAN];
    for (uint8_t i = 0; i < size; i++) temp[i] = distanceBuffer[i];
    for (uint8_t i = 0; i < size - 1; i++) {
      for (uint8_t j = i + 1; j < size; j++) {
        if (temp[j] < temp[i]) {
          uint8_t swap = temp[i];
          temp[i] = temp[j];
          temp[j] = swap;
        }
      }
    }
    return temp[size / 2];
  }

  // Return the latest filtered distance in cm.
  int getDistanceReading() {
    if (!distanceTofDetected) return AUTO_DISTANCE_MAX_SPEED;

    unsigned long now = millis();
    if (now - lastTofReadMs < tofReadEveryMs) return Distance;
    lastTofReadMs = now;

    uint16_t rawMm = distanceTof.readRangeContinuousMillimeters();
    if (distanceTof.timeoutOccurred() || !distanceTof.lastRangeReadValid()) return AUTO_DISTANCE_MAX_SPEED;

    int raw = (int)(rawMm / 10U);
    if (raw <= 0) raw = AUTO_DISTANCE_MAX_SPEED;
    if (raw > AUTO_DISTANCE_MAX_SPEED) raw = AUTO_DISTANCE_MAX_SPEED;

    int median = pushDistanceSampleAndGetMedian(raw);
    DBG_DISTANCE_SENSOR(F("VL53L0X raw="));
    DBG_DISTANCE_SENSOR(raw);
    DBG_DISTANCE_SENSOR(F(" cm  |  median="));
    DBG_DISTANCE_SENSOR(median);
    DBGLN_DISTANCE_SENSOR(F(" cm"));

    Distance = median;
    return median;
  }

  const int rampStep = 5;
  const unsigned long rampDelay = 80;
  static unsigned long lastRamp = 0;

  // Smooth large speed changes so auto mode does not jerk the drivetrain.
  // Ramp the motor toward a requested speed.
  void updateMotorSpeed(int targetSpeed) {
    unsigned long now = millis();
    if (now - lastRamp < rampDelay) return;
    lastRamp = now;

    if (Speed < targetSpeed) Speed = min(Speed + rampStep, targetSpeed);
    else if (Speed > targetSpeed) Speed = max(Speed - rampStep, targetSpeed);

    if (targetSpeed == 0) {
      Speed = 0;
      Stop();
      return;
    }

    if (Speed == 0) Stop();
    else setMotor(Dir::Forward, Speed);  // auto always forward
  }

  // Initialize the VL53L0X distance sensor backend.
  void initDistanceSensorHardware() {
    startDistanceSensorRanging(true);
  }

  // (Re-)configure and start continuous VL53L0X ranging.
  bool startDistanceSensorRanging(bool reinitializeSensor) {
    distanceTof.setTimeout(distanceTofTimeoutMs);
    if (!distanceTof.setAddress(vl53l0xAddress)) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L0X address set failed"));
      return false;
    }

    if (reinitializeSensor && !distanceTof.init()) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L0X not detected on I2C"));
      return false;
    }

    if (!distanceTof.setMeasurementTimingBudget(distanceTofTimingBudgetUs)) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L0X timing budget rejected"));
      return false;
    }

    distanceTof.startContinuous(distanceTofContinuousPeriodMs);
    distanceTofDetected = true;
    lastTofReadMs = 0;
    DBGLN_DISTANCE_SENSOR(F("VL53L0X ready"));
    return true;
  }

  // ================================================================================================
  // IR Receive (NEC with repeat support)
  // ================================================================================================
  // Wrap IRremote so the rest of the sketch sees a simple 8-bit command stream.
  // Decode NEC commands, including repeat frames.
  uint8_t irReceive() {
    lastWasRepeat = false;
    uint8_t received = 0;

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.protocol == NEC) {
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
          lastWasRepeat = true;
          received = lastIRCommand;
        } else {
          received = IrReceiver.decodedIRData.command;  // 8-bit
          lastIRCommand = received;
          DBG_IR_REMOTE(F("IR pressed: "));
          DBG_IR_REMOTE(received);
          DBG_IR_REMOTE(F(" | "));
          DBGLN_IR_REMOTE(irButtonLabel(received));
        }
      }
      IrReceiver.resume();
    }
    return received;  // 0 = no key received
  }

  // ================================================================================================
  // DRV8833 Safety
  // ================================================================================================
  // Identify the subset of buttons that are allowed to bring the motor driver back online.
  // Identify commands that may re-arm the motor driver.
  bool isMotorControlCommand(uint8_t code) {
    return code == buttonCHminus || code == buttonCH || code == buttonCHplus
      || code == buttonBackward || code == buttonForward || code == buttonPlayPause;
  }

  // nFAULT is a binary driver-protection signal, not an analog current measurement.
  // Once latched, the user must issue a fresh motor command after the fault clears.
  // Overwrites the fault_count byte for the current boot slot (capped to max 16 writes per power-on cycle).
  // Only increments the fault counter if motor drive was actually attempted/started since the previous fault.
  // Latch, report, and EEPROM-log DRV8833 fault conditions.
  void updateMotorFault() {
    if (motorFaultLatched || digitalRead(pinMotorFault) == HIGH) return;

    Stop();  // Coast the motor before disabling the driver.
    digitalWrite(pinMotorSleep, LOW);
    exitAutoDistanceMode();
    cancelJog();
    currentStep = 0;
    motorFaultLatched = true;

    #if ENABLE_EEPROM_LOGGING
    // Log the fault event by updating the single fault counter byte (max 16 times per power-on).
    if (motorDriveAttemptedSinceFault) {
      motorDriveAttemptedSinceFault = false;
      if (bootFaultCount < MAX_FAULTS_PER_BOOT && currentBootSlot < EEPROM_BOOT_LOG_SIZE) {
        bootFaultCount++;
        uint16_t faultAddr = EEPROM_ADDR_LOG_BASE + (uint16_t)currentBootSlot * EEPROM_ENTRY_SIZE + 4;
        EEPROM.update(faultAddr, bootFaultCount);
        DBG_MOTOR(F("DRV8833 fault #"));
        DBG_MOTOR(bootFaultCount);
        DBGLN_MOTOR(F(" logged to EEPROM"));
      }
    }
    #endif

    sirenActive = false;
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    SetGreenLightValue(0);
    SetRGBLightColor(RgbColor::Red);  // Bypass siren suppression for a safety indication.
    playPattern(pattern_batteryWarn);
    DBGLN_MOTOR(F("DRV8833 fault: motor disabled until a motor command re-arms it"));
  }

  // Re-enable the driver after a cleared fault.
  bool rearmMotorDriver() {
    digitalWrite(pinVL53L0X_XSHUT, HIGH); // Wake up VL53L0X
    delay(10); // boot time
    startDistanceSensorRanging();

    digitalWrite(pinMotorSleep, HIGH);
    delay(1);  // Allow nSLEEP to release and nFAULT to report the current condition.
    if (digitalRead(pinMotorFault) == LOW) {
      digitalWrite(pinMotorSleep, LOW);
      DBGLN_MOTOR(F("DRV8833 fault is still active"));
      return false;
    }

    motorFaultLatched = false;
    DBGLN_MOTOR(F("DRV8833 fault cleared; motor command accepted"));
    return true;
  }

  // ================================================================================================
  // Motor Control
  // ================================================================================================
  // Low-level H-bridge writer used by all higher-level movement commands.
  // Low-level DRV8833 direction and PWM output.
  void setMotor(Dir dir, int speed) {
    int safeSpeed = constrain(speed, 0, 255);
    switch (dir) {
      case Dir::Forward:
        analogWrite(pinMotor_IN1, safeSpeed);
        analogWrite(pinMotor_IN2, 0);
        MotorDirection = 1;
        if (safeSpeed > 0) motorDriveAttemptedSinceFault = true;
        DBG_MOTOR(F("Driving Forward >>> Speed="));
        DBGLN_MOTOR(safeSpeed);
        break;
      case Dir::Backward:
        analogWrite(pinMotor_IN1, 0);
        analogWrite(pinMotor_IN2, safeSpeed);
        MotorDirection = 2;
        if (safeSpeed > 0) motorDriveAttemptedSinceFault = true;
        DBG_MOTOR(F("Driving Backward >> Speed="));
        DBGLN_MOTOR(safeSpeed);
        break;
      default:  // Stop
        analogWrite(pinMotor_IN1, 0);
        analogWrite(pinMotor_IN2, 0);
        Speed = 0;
        DBGLN_MOTOR(F("Motor OFF"));
        break;
    }
  }

  // Direction-safe drive entry points.
  void GoForward() {
    if (MotorDirection == 2 && Speed > 0) {
      DBGLN_MOTOR(F("Ignored: cannot switch to FORWARD while moving"));
      return;
    }
    if (MotorDirection == 2 && Speed == 0) {
      Stop();
      motorReverseReadyAt = millis() + DIR_DELAY;  // non-blocking settle before reverse drive
    }
    MotorDirection = 1;
    if (Speed > 0) {
      if ((long)(millis() - motorReverseReadyAt) >= 0) setMotor(Dir::Forward, Speed);
      else motorDrivePending = true;  // drive once the cooldown elapses
    }
  }

  // Direction-safe drive entry points.
  void GoBackward() {
    if (MotorDirection == 1 && Speed > 0) {
      DBGLN_MOTOR(F("Ignored: cannot switch to BACKWARD while moving"));
      return;
    }
    if (MotorDirection == 1 && Speed == 0) {
      Stop();
      motorReverseReadyAt = millis() + DIR_DELAY;  // non-blocking settle before reverse drive
    }
    MotorDirection = 2;
    if (Speed > 0) {
      if ((long)(millis() - motorReverseReadyAt) >= 0) {
        setMotor(Dir::Backward, Speed);
        DBGLN_MOTOR(F("Driving Backward >>"));
      } else {
        motorDrivePending = true;  // drive once the cooldown elapses
      }
    }
  }

  // Coast the motor to an idle state.
  void Stop() {
    setMotor(Dir::Stop, 0);
    motorDrivePending = false;  // cancel any deferred reverse drive
  }

  // Apply a reverse drive that was deferred during the non-blocking direction cooldown.
  // Apply deferred motor drive after direction-change delay.
  void updateMotorReverseCooldown() {
    if (!motorDrivePending) return;
    if ((long)(millis() - motorReverseReadyAt) < 0) return;
    motorDrivePending = false;
    if (Speed <= 0) return;
    if (MotorDirection == 1) setMotor(Dir::Forward, Speed);
    else if (MotorDirection == 2) setMotor(Dir::Backward, Speed);
  }

  // Jog motor briefly without changing the stored direction/speed state machine.
  // Temporary hold-to-run movement helper.
  void JogDrive(Dir dir) {
    int jogPWM = pwmSteps[1];
    if (dir == Dir::Forward) {
      analogWrite(pinMotor_IN1, jogPWM);
      analogWrite(pinMotor_IN2, 0);
      if (jogPWM > 0) motorDriveAttemptedSinceFault = true;
    } else if (dir == Dir::Backward) {
      analogWrite(pinMotor_IN1, 0);
      analogWrite(pinMotor_IN2, jogPWM);
      if (jogPWM > 0) motorDriveAttemptedSinceFault = true;
    } else {
      analogWrite(pinMotor_IN1, 0);
      analogWrite(pinMotor_IN2, 0);
    }
  }

  // ================================================================================================
  // IR Command Handler
  // ================================================================================================
  // Central behavior router for the handheld remote.
  // This is where button meaning, mode changes, and safety interlocks come together.
  // Map remote buttons to train behavior.
  void translateIR() {
    uint8_t code = irReceive();
    if (code != 0 && !lastWasRepeat) {
      DBG_IR_REMOTE(F("Assigned action: "));
      DBGLN_IR_REMOTE(irButtonActionDescription(code));
    }

    // Cancel jog if a different non-repeat key appears
    if (momentaryActive && code != 0 && !lastWasRepeat && code != momentaryButton) {
      DBGLN_REMOTE(F("Cancelling jog due to new key"));
      Stop();
      SetRGBColor(RgbColor::Red);
      cancelJog();
    }

    // Ignore repeats for non-jog use cases (prevents CH± spam)
    if (lastWasRepeat && !momentaryActive) {
      return;
    }

    // No new code; if jogging and repeats stopped → timeout
    if (code == 0) {
      if (momentaryActive && (millis() - momentaryLastSeen > momentaryTimeout)) {
        DBGLN_MOTOR(F("Jog timeout -> STOP"));
        Stop();
        SetRGBColor(RgbColor::Red);
        cancelJog();
      }
      return;
    }

    lastActive = millis();

    // Blink green LED once for any valid button press
    GreenLEDBlink();

    // A motor-control button may re-arm only after the diagnostic output is clear.
    if (motorFaultLatched && isMotorControlCommand(code) && !rearmMotorDriver()) {
      return;
    }

    switch (code) {
      case buttonCHminus:
        {  // Speed -
          if (momentaryActive) {
            DBGLN_REMOTE(F("Ignored: CH- during jog"));
            break;
          }
          if (AutoDistanceOnOff == 0) {
            decreaseStep();
            DBG_MOTOR(F("Manual Speed Down: "));
            DBGLN_MOTOR(Speed);
            if (Speed == 0) {
              Stop();
              SetRGBColor(RgbColor::Red);
            } else {
              if (MotorDirection == 1) GoForward();
              if (MotorDirection == 2) GoBackward();
            }
          } else {
            DBGLN_REMOTE(F("Ignored: Auto-speed active"));
          }
          break;
        }

      case buttonCH:
        {  // Stop
          if (AutoDistanceOnOff) {
            exitAutoDistanceMode();
            DBGLN_REMOTE(F("Switched from AUTO to MANUAL mode"));
          }
          DBGLN_MOTOR(F("STOP pressed -> Motors stopped"));
          SetRGBColor(RgbColor::Red);
          stopAndResetStepSelection(true);  // default to forward when stopped
          DBGLN_REMOTE(F("Manual mode reset: next CH+ will start at step 1 (~3.5V)"));
          break;
        }

      case buttonCHplus:
        {  // Speed +
          if (momentaryActive) {
            DBGLN_REMOTE(F("Ignored: CH+ during jog"));
            break;
          }
          if (AutoDistanceOnOff == 0) {
            increaseStep();
            DBG_MOTOR(F("Manual Speed Up: "));
            DBGLN_MOTOR(Speed);
            if (MotorDirection == 1) {
              GoForward();
              SetRGBColor(RgbColor::White);
            }
            if (MotorDirection == 2) {
              GoBackward();
              SetRGBColor(RgbColor::Blue);
            }
          } else {
            DBGLN_REMOTE(F("Ignored: Auto-speed active"));
          }
          break;
        }

      case buttonBackward:
        {  // << momentary backward (jog)
          if (AutoDistanceOnOff == 0 && Speed == 0) {
            JogDrive(Dir::Backward);
            SetRGBColor(RgbColor::Blue);
            momentaryActive = true;
            momentaryButton = buttonBackward;
            momentaryLastSeen = millis();
            DBGLN_MOTOR(F("Momentary BACKWARD running (hold to move)"));
          } else {
            DBGLN_REMOTE(F("Ignored: << only when stationary & not in AUTO"));
          }
          break;
        }

      case buttonForward:
        {  // >> momentary forward (jog)
          if (AutoDistanceOnOff == 0 && Speed == 0) {
            JogDrive(Dir::Forward);
            SetRGBColor(RgbColor::White);
            momentaryActive = true;
            momentaryButton = buttonForward;
            momentaryLastSeen = millis();
            DBGLN_MOTOR(F("Momentary FORWARD running (hold to move)"));
          } else {
            DBGLN_REMOTE(F("Ignored: >> only when stationary & not in AUTO"));
          }
          break;
        }

      case button0:
        {  // Color sensor toggle
          setColorSensorEnabled(ColorSensorOnOff == 0);
          break;
        }

      case buttonPlayPause:
        {  // Auto-speed toggle
          AutoDistanceOnOff = !AutoDistanceOnOff;
          SetGreenLightValue(AutoDistanceOnOff ? 255 : 0);
          if (AutoDistanceOnOff) {
            DBGLN_MOTOR(F("Driving started (auto-speed)"));
            SetRGBColor(RgbColor::White);
            GoForward();
            playPattern(pattern_double);
          } else {
            DBGLN_MOTOR(F("Driving stopped"));
            SetRGBColor(RgbColor::Red);
            stopAndResetStepSelection();
            Speed = 0;
            playPattern(pattern_descend);
            DBGLN_REMOTE(F("Manual mode rearmed: next step = 1 (~3.5V)"));
          }
          break;
        }

      case buttonEQ:
        {  // Mute / Unmute
          SoundOnOff = !SoundOnOff;
          if (SoundOnOff) DBGLN_SOUND(F("Sound ON"));
          else DBGLN_SOUND(F("Sound OFF"));

          if (!SoundOnOff) {
            // Hard stop any audio that's playing
            noTone(pinBuzzer);
            digitalWrite(pinBuzzer, LOW);
            clearBuzzerPattern();
            // DO NOT touch sirenActive or LEDs -> lights continue flashing if sirenActive==true
          } else {
            playPattern(pattern_double);  // short confirmation chirp
          }
          break;
        }

      case button100plus:
        {  // Horn
          DBGLN_SOUND(F("Horn activated"));
          playPattern(pattern_horn);
          break;
        }

      case button200plus:
        sirenActive = !sirenActive;
        if (!sirenActive) {
          noTone(pinBuzzer);
          SetRGBColor(RgbColor::Off);
        } else {
          sirenTimer = millis();      // for LED swap
          sirenStartMs = sirenTimer;  // for deterministic audio sweep
          DBGLN_SOUND(SoundOnOff ? F("Siren ON (with sound)") : F("Siren ON (lights only, muted)"));
        }
        break;

      case button9:
        {  // Beep battery level in 10% steps
          float vIn = getBatteryVoltageSettledForStatus();
          int batteryPercent = get2SBatteryPercent(vIn);
          DBG_VOLTAGE_METER(F("Battery Voltage: "));
          DBGLN_VOLTAGE_METER(vIn, 1);
          DBG_VOLTAGE_METER(F("Battery level: "));
          DBG_VOLTAGE_METER(batteryPercent);
          DBGLN_VOLTAGE_METER(F("%"));
          playVoltagePattern((float)batteryPercent);
          break;
        }

      default:
        if (tryPlayMelodyForButton(code)) {
          break;
        }
        break;
    }

    // Refresh jog heartbeat if the same jog key is still active
    if (momentaryActive && (code == momentaryButton)) {
      momentaryLastSeen = millis();
    }
  }


  // ================================================================================================
  // Buzzer (non-blocking pattern player)
  // ================================================================================================
  // Reset the queue used by updateBuzzer(). Safe to call before loading a new pattern.
  // Reset queued buzzer timing data.
  inline void clearBuzzerPattern() {
    for (int j = 0; j < BUZZER_PATTERN_MAX; ++j) buzzerPattern[j] = 0;
    buzzerIndex = 0;
  }

  // Advance the current buzzer pattern one timing step at a time.
  // Advance non-blocking buzzer patterns.
  void updateBuzzer() {
    // Siren owns the buzzer while active; queued patterns resume once the siren stops.
    if (sirenActive) return;
    if (buzzerPattern[buzzerIndex] == 0) return;
    unsigned long now = millis();
    if (now - buzzerTimer >= (unsigned long)buzzerPattern[buzzerIndex]) {
      ++buzzerIndex;
      buzzerTimer = now;
      if (buzzerPattern[buzzerIndex] == 0) {
        digitalWrite(pinBuzzer, LOW);
        clearBuzzerPattern();
        return;
      }
      // Even indices are ON durations, odd indices are OFF gaps.
      if ((buzzerIndex & 1) == 0) digitalWrite(pinBuzzer, HIGH);
      else digitalWrite(pinBuzzer, LOW);
    }
  }

  // ------------------------------------------------------------------------------------------------
  // Initiate one of the predefined sound patterns. Then updateBuzzer plays the pattern.
  // ------------------------------------------------------------------------------------------------
  // Start a predefined buzzer pattern.
  void playPattern(const int* pattern) {
    if (SoundOnOff != 1) return;
    digitalWrite(pinBuzzer, LOW);
    clearBuzzerPattern();

    int i = 0;
    for (; i < BUZZER_PATTERN_MAX - 1; ++i) {
      int v = pattern[i];
      buzzerPattern[i] = v;
      if (v == 0) break;
    }
    buzzerPattern[i] = 0;

    buzzerTimer = millis();
    if (buzzerPattern[0] > 0) digitalWrite(pinBuzzer, HIGH);
  }

  // ------------------------------------------------------------------------------------------------
  // Initiate battery percentage sound pattern. Then updateBuzzer plays the pattern.
  //    - 10%..100% -> 1..10 short beeps
  //    - 0% -> one extra-short beep
  // ------------------------------------------------------------------------------------------------
  // Speak battery voltage with long/short beeps.
  void playVoltagePattern(float batteryPercent) {
    if (SoundOnOff != 1) return;
    digitalWrite(pinBuzzer, LOW);
    clearBuzzerPattern();

    int percent = (int)round(batteryPercent);
    int beepCount = constrain(percent / 10, 0, 10);
    int cap = BUZZER_PATTERN_MAX - 1;
    int idx = 0;

    if (beepCount == 0) {
      buzzerPattern[idx++] = 80;
      buzzerPattern[idx] = 0;
      buzzerTimer = millis();
      digitalWrite(pinBuzzer, HIGH);
      return;
    }

    for (int i = 0; i < beepCount && idx < cap; ++i) {
      if (idx < cap) buzzerPattern[idx++] = 150;
      if (i < beepCount - 1 && idx < cap) buzzerPattern[idx++] = 150;
    }

    buzzerPattern[idx] = 0;
    buzzerTimer = millis();
    if (buzzerPattern[0] > 0) digitalWrite(pinBuzzer, HIGH);
  }

  // ================================================================================================
  // LEDs
  // ================================================================================================
  // Lowest-level LED output helper; all train light changes funnel through here.
  // Write one expander-backed LED channel.
  inline void writeTrainOutput(const LedRoute& route, bool Value) {
    if (trainLedExpanderDetected) {
      trainLedExpander.digitalWrite(route.expanderPin, Value ? HIGH : LOW);
    } else {
      DBGLN_LEDS(String(F("Expander pin GP")) + String(route.expanderPin) + F(" to ") + String(Value ? "ON" : "OFF"));
    }
  }

  // Drive one RGB LED as discrete color channels.
  inline void writeRGBPins(const LedRoute& rPin, const LedRoute& gPin, const LedRoute& bPin, bool R, bool G, bool B) {
    // This helper forwards the requested channel state to the expander-backed LED output.
    writeTrainOutput(rPin, R);
    writeTrainOutput(gPin, G);
    writeTrainOutput(bPin, B);
  }

  // Apply raw RGB channel states to one headlight or both.
  // Apply raw RGB values to one or both headlights.
  void SetRGBLight(bool R, bool G, bool B, int led) {
    if (FrontLightOnOff == 0) return;
    if (led == 0 || led == 1) writeRGBPins(led1R, led1G, led1B, R, G, B);
    if (led == 0 || led == 2) writeRGBPins(led2R, led2G, led2B, R, G, B);
  }

  // Apply a named RGB color.
  void SetRGBLightColor(RgbColor color, int led) {
    bool R = false, G = false, B = false;
    switch (color) {
      case RgbColor::Red: R = true; break;
      case RgbColor::Green: G = true; break;
      case RgbColor::Blue: B = true; break;
      case RgbColor::Yellow: R = true; G = true; break;
      case RgbColor::Cyan: G = true; B = true; break;
      case RgbColor::Magenta: R = true; B = true; break;
      case RgbColor::White: R = true; G = true; B = true; break;
      default: break;
    }
    SetRGBLight(R, G, B, led);
  }

  // Apply normal status colors while respecting higher-priority modes like siren and sensor mode.
  // Apply status color unless siren/sensor mode overrides it.
  inline void SetRGBColor(RgbColor color, int led) {
    if (sirenActive) return;  // ignore during siren
    if (ColorSensorOnOff == 1 && color != RgbColor::Off) {
      SetRGBLightColor(RgbColor::Cyan, led);
      return;
    }
    SetRGBLightColor(color, led);
  }

  // Control the green status LED.
  void SetGreenLightValue(int Value) {
    // analogWrite(pinLEDGreen, Value);
    // The green status LED is driven through the expander-backed output stage.
    writeTrainOutput(ledGreen, (Value > 0));  // Any non-zero means ON
  }

  // Start a non-blocking 2-pulse acknowledgement blink; final state restored by updateGreenBlink().
  // Non-blocking status-LED acknowledgement blink.
  void GreenLEDBlink() {
    greenBlinkRemaining = 2;
    greenBlinkOn = true;
    greenBlinkStepMs = millis();
    SetGreenLightValue(255);
  }

  // Advance the acknowledgement blink and restore the auto-mode indicator when finished.
  // Non-blocking status-LED acknowledgement blink.
  void updateGreenBlink() {
    if (greenBlinkRemaining <= 0) return;
    unsigned long now = millis();
    unsigned long dur = greenBlinkOn ? greenBlinkOnMs : greenBlinkOffMs;
    if (now - greenBlinkStepMs < dur) return;
    greenBlinkStepMs = now;
    if (greenBlinkOn) {
      SetGreenLightValue(0);
      greenBlinkOn = false;
    } else {
      greenBlinkRemaining--;
      if (greenBlinkRemaining > 0) {
        SetGreenLightValue(255);
        greenBlinkOn = true;
      } else {
        SetGreenLightValue(AutoDistanceOnOff ? 255 : 0);  // restore auto-speed indicator
      }
    }
  }

  /*
  // Direct Nano fallback block kept for possible rollback from the MCP23008.

  // Direct-pin LED route mapping helper.
  struct LedRoute {
    // Direct Nano GPIO pin used by the fallback mapping.
    uint8_t directPin;
  };

  const LedRoute led1R = { 11 };
  const LedRoute led1G = { 3 };
  const LedRoute led1B = { 4 };
  const LedRoute led2R = { 7 };
  const LedRoute led2G = { 8 };
  const LedRoute led2B = { 9 };
  const LedRoute ledGreen = { 10 };
  const int pinColorSensorLED = -1;

  void initTrainLedHardware() {
    const LedRoute routes[] = { led1R, led1G, led1B, led2R, led2G, led2B, ledGreen };
    for (uint8_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
      pinMode(routes[i].directPin, OUTPUT);
      digitalWrite(routes[i].directPin, LOW);
    }

    DBGLN_LEDS(F("Direct train LED pins ready"));
  }

  inline void writeTrainOutput(const LedRoute& route, bool Value) {
    digitalWrite(route.directPin, Value ? HIGH : LOW);
  }
  */

  // Siren animation is time-based so pitch and light sweep remain stable if loop timing varies.
  // Animate siren lights and pitch sweep.
  void updateSiren() {
    if (!sirenActive) return;

    unsigned long now = millis();

    // LED swap every 300 ms (always runs, even when muted)
    if (now - sirenTimer > 300) {
      sirenPhase = !sirenPhase;
      if (sirenPhase) {
        SetRGBLight(true, false, false, 1);  // LED1 red
        SetRGBLight(false, false, true, 2);  // LED2 blue
      } else {
        SetRGBLight(false, false, true, 1);  // LED1 blue
        SetRGBLight(true, false, false, 2);  // LED2 red
      }
      sirenTimer = now;
    }

    // ---- Deterministic triangle sweep for pitch (immune to loop jitter) ----
    const unsigned long cycleMs = 2UL * sirenSweepMs;  // Total cycle = up + down
    unsigned long t = (now - sirenStartMs) % cycleMs;

    int f;
    if (t < sirenSweepMs) {
      // ramp up: Fmin → Fmax
      f = sirenFmin + (int)((unsigned long)(sirenFmax - sirenFmin) * t / sirenSweepMs);
    } else {
      // ramp down: Fmax → Fmin
      unsigned long td = t - sirenSweepMs;
      f = sirenFmax - (int)((unsigned long)(sirenFmax - sirenFmin) * td / sirenSweepMs);
    }

    // Sound part only if not muted
    if (SoundOnOff == 1) tone(pinBuzzer, f);
    else noTone(pinBuzzer);
  }

  // ================================================================================================
  // Tilt sensor (SW-520D / SW-200D family) – debounced, RGB + 0.5s beep on tilt.
  // Wiring: D9 -> tilt switch -> GND. No external resistor or capacitor is required in this revision.
  // D9 uses the internal pull-up, so a closed tilt switch pulls the input LOW when active.
  // Note: tilt doesn't disable the siren
  // ================================================================================================
  // Debounce tilt input and latch emergency stop.
  void updateTiltSensor() {
    unsigned long now = millis();
    int reading = digitalRead(pinTiltSensor);

    if (reading != tiltLastRead) {
      tiltLastRead = reading;
      tiltEdgeAt = now;
    }

    if (now >= tiltQuietUntil && reading != tiltStableState && (now - tiltEdgeAt) >= TILT_STABLE_MS) {

      tiltStableState = reading;
      tiltQuietUntil = now + TILT_QUIET_MS;

      if (tiltStableState == LOW) {
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
        playPattern(pattern_tiltBeep);
      } else {
        DBGLN_TILT_SENSOR(F("TILT: IDLE -> clear latch, restore LEDs"));
        tiltStopLatched = false;
        SetRGBColor(RgbColor::Yellow);
      }
    }
  }

  // ================================================================================================
  // Tone melody player (non-blocking)
  //   - Sequence format: flat int array [freq, duration_ms, freq, duration_ms, ...]
  //   - freq = 0 -> rest (silence) for 'duration_ms'
  //   - Call updateMelody() in the main loop to handle playback timing.
  //   - Respects SoundOnOff flag (must be 1 to play).
  //   - Automatically stops if sirenActive is true.
  //   - Supports looping playback.
  //   - Use playToneSequenceRaw(...) to start a new melody.
  //   - Use stopMelody() to stop playback manually.
  // ================================================================================================

  // Stops melody playback and resets related state.
  // Stop melody playback and clear melody state.
  void stopMelody() {
    noTone(pinBuzzer);
    melodyPlaying = false;
    melodyLenPairs = 0;
    melodyIdxPair = 0;
    melodyStepStarted = 0;
  }

  // Reads the note value (frequency or duration) at flat index `idx` from the active
  // sequence, transparently handling PROGMEM- vs RAM-backed sources.
  inline int melodyReadAt(int idx) {
    return melodySrcIsProgmem ? (int)(int16_t)pgm_read_word(&melodySrc[idx]) : melodySrc[idx];
  }

  // Advance the current melody using millis()-based timing instead of delay().
  // Advance non-blocking melody playback.
  void updateMelody() {
    if (!melodyPlaying) return;  // Exit if no melody is playing
    if (SoundOnOff != 1 || sirenActive) {
      stopMelody();
      return;
    }  // Stop if sound is muted or siren is active

    unsigned long now = millis();

    // Start the first note
    if (melodyStepStarted == 0) {
      int f = melodyReadAt(melodyIdxPair * 2);      // Frequency
      int d = melodyReadAt(melodyIdxPair * 2 + 1);  // Duration
      if (f > 0) tone(pinBuzzer, f);
      else noTone(pinBuzzer);
      melodyStepStarted = now;
      return;
    }

    // Continue checking if the current note's duration has elapsed
    int dCur = melodyReadAt(melodyIdxPair * 2 + 1);
    if (now - melodyStepStarted >= (unsigned long)dCur) {
      melodyIdxPair++;  // Move to the next note
      if (melodyIdxPair >= melodyLenPairs) {
        if (melodyLoop) melodyIdxPair = 0;  // Loop back if enabled
        else {
          stopMelody();
          return;
        }  // Stop if done
      }
      int f = melodyReadAt(melodyIdxPair * 2);
      int d = melodyReadAt(melodyIdxPair * 2 + 1);
      if (f > 0) tone(pinBuzzer, f);
      else noTone(pinBuzzer);
      melodyStepStarted = now;
    }
  }

  // Point the playback state at a flat [frequency, duration] sequence and arm playback.
  // Supports either RAM-backed arrays or PROGMEM-backed arrays depending on isProgmem;
  // notes are read on-the-fly by updateMelody(), no RAM copy is made.
  // Load and start a melody from RAM or PROGMEM.
  void playToneSequenceRaw(const int* seqFD, int pairCount, bool loopPlayback, bool isProgmem) {
    if (pairCount <= 0) return;
    if (SoundOnOff != 1) return;

    // Stop any ongoing buzzer pattern (non-melody)
    clearBuzzerPattern();

    // Stop the siren if active
    if (sirenActive) {
      sirenActive = false;
      noTone(pinBuzzer);
    }

    // Limit the number of pairs to prevent overflow
    melodyLenPairs = (pairCount > MELODY_MAX_PAIRS) ? MELODY_MAX_PAIRS : pairCount;

    // Reference the sequence directly; no RAM copy needed.
    melodySrc = (const int16_t*)seqFD;
    melodySrcIsProgmem = isProgmem;

    // Initialize playback state
    melodyIdxPair = 0;
    melodyLoop = loopPlayback;
    melodyPlaying = true;
    melodyStepStarted = 0;  // Triggers first note on next update
  }
