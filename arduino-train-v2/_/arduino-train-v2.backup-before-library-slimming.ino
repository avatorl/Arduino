  #define IR_USE_AVR_TIMER1 // Configure IRremote to use Timer1 on AVR
  // Warning: when IRremote is configured to use Timer1, don’t use PWM (analogWrite) on Timer1 pins (D9, D10)
  
  // Library includes
  #include <Wire.h>         // I2C bus
  #include <IRremote.hpp>  // IR remote library
  #include <LowPower.h>    // Low power/sleep mode library
  #include <math.h>        // Math functions
  #include <Adafruit_MCP23X08.h>  // I2C GPIO expander library
  #include <Adafruit_TCS34725.h>  // I2C color sensor library


  #include "src/melodies.h" // Load music patterns

  // Probably use VL53L1X distance sensor with matrix field of view

  // ===============================================================================================
  // Sketch Contents / Structure Guide
  // ===============================================================================================
  //  1. Compile-time config and debug macros
  //  2. Shared enums, forward declarations, and hardware pin mapping
  //  3. LED expander and color-sensor configuration/state
  //  4. Motion, sound, siren, tilt, and melody runtime state
  //  5. Setup / main loop
  //  6. Power management and battery measurement helpers
  //  7. Manual speed-step control
  //  8. Auto-distance speed control and distance filtering
  //  9. IR receiver decoding and command translation
  // 10. DRV8833 safety and re-arm logic
  // 11. Motor drive helpers and jog control
  // 12. Buzzer patterns and spoken-voltage playback
  // 13. RGB / status LED output helpers
  // 14. Siren effect, tilt protection, and non-blocking melody player
  //
  // Function Index
  //   playToneSequence_P()         - Template wrapper for PROGMEM melodies.
  //   initTrainLedHardware()       - Detect and initialize the MCP23008 LED expander.
  //   initColorSensorHardware()    - Configure the TCS34725 and its onboard lamp control.
  //   setColorSensorEnabled()      - Toggle color-sensor mode and related status lighting.
  //   classifyTrackMarkerColor()   - Convert raw RGBC values into track marker classes.
  //   trackMarkerLabel()           - Debug-only marker name lookup.
  //   handleTrackMarkerAction()    - Run the action associated with a detected marker color.
  //   updateColorSensor()          - Poll and process the color sensor without blocking.
  //   refreshDriveLights()         - Restore status lights from the current drive state.
  //   setup()                      - Initialize hardware and startup state.
  //   loop()                       - Main scheduler for safety, input, sensing, and playback.
  //   goToIdle()                   - Shut down outputs and enter low-power sleep.
  //   wakeUp()                     - Interrupt callback used only to wake the MCU.
  //   getBatteryVoltageDirect()    - Read pack voltage through the resistor divider.
  //   getBatteryVoltageSettledForStatus() - Stop loads, let the pack settle, then measure.
  //   safePWMFromVoltage()         - Convert a target motor voltage to a safe PWM value.
  //   configureSpeedSteps()        - Build manual speed steps from live battery voltage.
  //   playStepBeep()               - Audible confirmation for the selected manual step.
  //   applySpeedStep()             - Apply the current manual step to the motor state.
  //   increaseStep() / decreaseStep() - Move between manual drive steps.
  //   motorVoltageFromDistance()   - Map obstacle distance to a target motor voltage.
  //   initDistanceSensorHardware() - Initialize the VL53L0X distance sensor backend.
  //   getDistanceReading()         - Return the latest filtered distance in cm.
  //   updateAutoDistanceSpeed()    - Run the automatic speed controller.
  //   updateMotorSpeed()           - Ramp the motor toward a requested speed.
  //   irReceive()                  - Decode NEC commands, including repeat frames.
  //   isMotorControlCommand()      - Identify commands that may re-arm the motor driver.
  //   updateMotorFault()           - Latch and report DRV8833 fault conditions.
  //   rearmMotorDriver()           - Re-enable the driver after a cleared fault.
  //   setMotor()                   - Low-level DRV8833 direction and PWM output.
  //   GoForward() / GoBackward()   - Direction-safe drive entry points.
  //   Stop()                       - Coast the motor to an idle state.
  //   JogDrive()                   - Temporary hold-to-run movement helper.
  //   translateIR()                - Map remote buttons to train behavior.
  //   clearBuzzerPattern()         - Reset queued buzzer timing data.
  //   updateBuzzer()               - Advance non-blocking buzzer patterns.
  //   playPattern()                - Start a predefined buzzer pattern.
  //   playVoltagePattern()         - Speak battery voltage with long/short beeps.
  //   writeTrainOutput()           - Write one expander-backed LED channel.
  //   writeRGBPins()               - Drive one RGB LED as discrete color channels.
  //   SetRGBLight()                - Apply raw RGB values to one or both headlights.
  //   SetRGBLightColor()           - Apply a named RGB color.
  //   SetRGBColor()                - Apply status color unless siren/sensor mode overrides it.
  //   SetGreenLightValue()         - Control the green status LED.
  //   GreenLEDBlink() / updateGreenBlink() - Non-blocking status-LED acknowledgement blink.
  //   updateSiren()                - Animate siren lights and pitch sweep.
  //   updateTiltSensor()           - Debounce tilt input and latch emergency stop.
  //   stopMelody()                 - Stop melody playback and clear melody state.
  //   updateMelody()               - Advance non-blocking melody playback.
  //   playToneSequenceRaw()        - Load and start a melody from RAM or PROGMEM.

  // ===============================================================================================
  // Per-module debug flags. Set a flag to 1 to compile only that subsystem's
  // logs. The legacy DEBUG flag is still supported as a fallback that enables
  // every module when set to 1.
  #ifndef DEBUG
  #define DEBUG 0
  #endif

  #ifndef DEBUG_COLOR_SENSOR
  #define DEBUG_COLOR_SENSOR DEBUG
  #endif
  #ifndef DEBUG_DISTANCE_SENSOR
  #define DEBUG_DISTANCE_SENSOR DEBUG
  #endif
  #ifndef DEBUG_OTHER_SENSORS
  #define DEBUG_OTHER_SENSORS DEBUG
  #endif
  #ifndef DEBUG_MOTOR
  #define DEBUG_MOTOR DEBUG
  #endif
  #ifndef DEBUG_LEDS
  #define DEBUG_LEDS DEBUG
  #endif
  #ifndef DEBUG_SOUND
  #define DEBUG_SOUND DEBUG
  #endif
  #ifndef DEBUG_REMOTE
  #define DEBUG_REMOTE DEBUG
  #endif

  #define DEBUG_ANY (DEBUG_COLOR_SENSOR || DEBUG_DISTANCE_SENSOR || DEBUG_OTHER_SENSORS || DEBUG_MOTOR || DEBUG_LEDS || DEBUG_SOUND || DEBUG_REMOTE)

  #if DEBUG_ANY
  #define DBGBEGIN(...) \
    do { Serial.begin(__VA_ARGS__); } while (0)  // Initialize Serial
  #else
  #define DBGBEGIN(...)  // No operation
  #endif

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

  #if DEBUG_OTHER_SENSORS
  #define DBG_OTHER_SENSORS(...) Serial.print(__VA_ARGS__)
  #define DBGLN_OTHER_SENSORS(...) Serial.println(__VA_ARGS__)
  #else
  #define DBG_OTHER_SENSORS(...)
  #define DBGLN_OTHER_SENSORS(...)
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

  #if DEBUG_REMOTE
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

  #include <VL53L0X.h>  // Pololu VL53L0X library

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

  struct BalancedRgbs {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
  };

  struct MarkerClusterDefinition {
    TrackMarkerClass markerClass;
    PrototypeRgb center;
    uint16_t maxDistance;
  };

  // Forward declarations are grouped here so the high-level flow can stay readable below.
  // Most functions are implemented in the same order as the contents guide above.
  void SetRGBLightColor(RgbColor color, int led = 0);
  void SetRGBColor(RgbColor color, int led = 0);

  void playToneSequenceRaw(const int* seqFD, int pairCount, bool loopPlayback, bool isProgmem = false);

  // PROGMEM wrapper (for PROGMEM melodies)
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
  int getDistanceReading();
  bool tryPlayMelodyForButton(uint8_t code);
  void exitAutoDistanceMode(bool clearIndicator = true);
  void stopAndResetStepSelection(bool resetDirection = false);
  void cancelJog();
  int get2SBatteryPercent(float voltage);
  uint8_t classifyTrackMarkerColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
  void handleTrackMarkerAction(uint8_t markerClass);
  #if DEBUG_COLOR_SENSOR
  const __FlashStringHelper* trackMarkerLabel(uint8_t markerClass);
  #endif

  // Motor directions
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
  const int button2 = 24;           // Play mucic 2
  const int button3 = 94;           // Play music 3
  const int button4 = 8;            // Play music 4
  const int button5 = 28;           // Play music 5
  const int button6 = 90;           // Play music 6
  const int button7 = 66;           // Play music 7
  const int button8 = 82;           // Play music 8
  const int button9 = 74;           // Battery Test

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
  // LED channels are powered from the 2S pack and low-side switched by BC547s:
  // controller pin -> 1,2 kOhm resistor -> base; collector -> resistor -> LED cathode; emitter -> GND.
  // HIGH turns a channel on. Both RGB LEDs are common-anode and need no PWM.
  struct LedRoute {
    uint8_t expanderPin;
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
  Adafruit_MCP23X08 trainLedExpander;
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
  Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
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

  // TCS34725 cluster-based classification data generated from rgb-color-sensor captures
  // Mounting: aim for ~3 mm sensor-to-marker gap (usable 2-5 mm), fixed height and angle, LED on.
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

  // Per-color marker action toggles; flip to false to disable that color's action.
  // Actions run only on color transitions, so a long marker does not keep retriggering.
  // Action bodies are placeholders for now - define each behavior in handleTrackMarkerAction().
  const bool actionWhiteMarker = true;
  const bool actionBrownMarker = true;
  const bool actionCyanMarker = true;
  const bool actionRedMarker = true;
  const bool actionGreenMarker = true;
  const bool actionGreyMarker = true;
  const bool actionMagentaMarker = true;
  const bool actionOrangeMarker = true;
  const bool actionYellowMarker = true;

  // Speed steps (voltage → PWM)
  uint8_t pwmSteps[4];  // 0..255
  float voltageSteps[] = { 0.0, 3.5, 4.5, 6.0 };
  float batteryVoltage = 0.0;

  VL53L0X distanceTof;
  bool distanceTofDetected = false;
  unsigned long lastTofReadMs = 0;
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
  const int MELODY_MAX_PAIRS = 64;      // up to 64 (freq,dur) pairs
  int melodySeq[MELODY_MAX_PAIRS * 2];  // flat [f,d,f,d,...]
  int melodyLenPairs = 0;               // number of (f,d) pairs loaded
  int melodyIdxPair = 0;                // current pair index
  bool melodyLoop = false;              // loop playback
  bool melodyPlaying = false;           // active?
  unsigned long melodyStepStarted = 0;  // ms when current note started

  // ================================================================================================
  // Setup
  // ================================================================================================
  // Bring up every hardware block once, then seed the initial runtime state.
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

  void initColorSensorHardware() {
    pinMode(pinColorSensorLED, OUTPUT);
    digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);

    colorSensorDetected = colorSensor.begin();
    if (colorSensorDetected) {
      colorSensor.disable();  // Keep the sensor IC core in low-power sleep (~2 uA) until enabled
      DBGLN_COLOR_SENSOR(F("TCS34725 ready (in sleep mode)"));
    } else {
      DBGLN_COLOR_SENSOR(F("TCS34725 not detected on I2C"));
    }
  }

  void initBatteryVoltageMeterHardware() {
    analogReference(INTERNAL);
    delay(5);  // Let the internal 1.1V reference settle before discarding the first conversion.
    analogRead(pinBatterySense);
  }

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

  BalancedRgbs applyWhiteBalance(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    BalancedRgbs balanced;
    balanced.r = (uint16_t)(r * whiteBalanceRedGain + 0.5f);
    balanced.g = (uint16_t)(g * whiteBalanceGreenGain + 0.5f);
    balanced.b = (uint16_t)(b * whiteBalanceBlueGain + 0.5f);
    balanced.c = c;
    return balanced;
  }

  PrototypeRgb normalizePrototype(uint16_t r, uint16_t g, uint16_t b) {
    PrototypeRgb prototype = { 0, 0, 0 };
    uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;
    if (sum == 0) return prototype;

    prototype.r = (uint16_t)(((uint32_t)r * 1000UL + (sum / 2)) / sum);
    prototype.g = (uint16_t)(((uint32_t)g * 1000UL + (sum / 2)) / sum);
    prototype.b = (uint16_t)(((uint32_t)b * 1000UL + (sum / 2)) / sum);
    return prototype;
  }

  uint16_t prototypeDistance(const PrototypeRgb& a, const PrototypeRgb& b) {
    uint16_t distance = 0;
    distance += (a.r > b.r) ? (a.r - b.r) : (b.r - a.r);
    distance += (a.g > b.g) ? (a.g - b.g) : (b.g - a.g);
    distance += (a.b > b.b) ? (a.b - b.b) : (b.b - a.b);
    return distance;
  }

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

  #if DEBUG_COLOR_SENSOR
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

  void handleTrackMarkerAction(uint8_t markerClass) {
    switch (markerClass) {
      case MarkerWhite:
        if (!actionWhiteMarker) return;
        DBGLN_COLOR_SENSOR(F("Track action: WHITE marker (no action defined yet)"));
        break;

      case MarkerBrown:
        if (!actionBrownMarker) return;
        DBGLN_COLOR_SENSOR(F("Track action: BROWN marker (no action defined yet)"));
        break;

      case MarkerCyan:
        if (!actionCyanMarker) return;
        DBGLN_COLOR_SENSOR(F("Track action: CYAN marker (no action defined yet)"));
        break;

      case MarkerRed:
        if (!actionRedMarker) return;
        DBGLN_COLOR_SENSOR(F("Track action: RED marker (no action defined yet)"));
        break;

      case MarkerGreen:
        if (!actionGreenMarker) return;
        DBGLN_COLOR_SENSOR(F("Track action: GREEN marker (no action defined yet)"));
        break;

      case MarkerGrey:
        if (!actionGreyMarker) return;
        DBGLN_COLOR_SENSOR(F("Track action: GREY marker (no action defined yet)"));
        break;

      case MarkerMagenta:
        if (!actionMagentaMarker) return;
        DBGLN_COLOR_SENSOR(F("Track action: MAGENTA marker (no action defined yet)"));
        break;

      case MarkerOrange:
        if (!actionOrangeMarker) return;
        DBGLN_COLOR_SENSOR(F("Track action: ORANGE marker (no action defined yet)"));
        break;

      case MarkerYellow:
        if (!actionYellowMarker) return;
        DBGLN_COLOR_SENSOR(F("Track action: YELLOW marker (no action defined yet)"));
        break;

      default:
        break;
    }
  }

  void updateColorSensor() {
    if (ColorSensorOnOff == 0 || !colorSensorDetected) return;

    unsigned long now = millis();
    if (now - lastColorSensorRead < colorSensorReadEveryMs) return;
    lastColorSensorRead = now;

    uint16_t r = 0, g = 0, b = 0, c = 0;
    colorSensor.getRawData(&r, &g, &b, &c);
    uint8_t markerClass = classifyTrackMarkerColor(r, g, b, c);
    #if DEBUG_COLOR_SENSOR
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
      #if DEBUG_COLOR_SENSOR
      DBG_COLOR_SENSOR(F("Track marker changed to: "));
      DBGLN_COLOR_SENSOR(markerLabel);
      #endif
      handleTrackMarkerAction(markerClass);
    }
  }

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

  void exitAutoDistanceMode(bool clearIndicator) {
    AutoDistanceOnOff = false;
    if (clearIndicator) SetGreenLightValue(0);
  }

  void stopAndResetStepSelection(bool resetDirection) {
    Stop();
    currentStep = 0;
    if (resetDirection) MotorDirection = 1;
  }

  void cancelJog() {
    momentaryActive = false;
  }

  // One-time hardware initialization and startup behavior.
  void setup() {

    // Hardware reset the VL53L0X to allow address change and avoid conflict with TCS34725.
    pinMode(pinVL53L0X_XSHUT, OUTPUT);
    digitalWrite(pinVL53L0X_XSHUT, LOW); // Hold VL53L0X in hardware reset
    delay(10);                           // Give it time to completely power down
    digitalWrite(pinVL53L0X_XSHUT, HIGH); // Wake up the VL53L0X
    delay(10);                           // Wait for it to boot before sending I2C commands


    DBGBEGIN(9600);

    // Initialize Arduino pins
    initTrainLedHardware();
    pinMode(pinBuzzer, OUTPUT);
    pinMode(pinMotor_IN1, OUTPUT);
    pinMode(pinMotor_IN2, OUTPUT);
    pinMode(pinMotorFault, INPUT_PULLUP);
    pinMode(pinMotorSleep, OUTPUT);
    digitalWrite(pinVL53L0X_XSHUT, HIGH); // Wake up VL53L0X
    delay(10); // boot time
    distanceTof.setAddress(vl53l0xAddress);
    distanceTof.startContinuous(50);

    digitalWrite(pinMotorSleep, HIGH);  // Enable the DRV8833 after power-up.
    //pinMode(pinIRReceiver, INPUT);
    pinMode(pinBatterySense, INPUT);
    pinMode(pinTiltSensor, INPUT_PULLUP);
    initBatteryVoltageMeterHardware();
    initColorSensorHardware();

    /* ==========================================================================================
     * PROTOTYPE / FUTURE CODE: Sensor Self-Test & Diagnostic Error Codes on Boot
     * ==========================================================================================
     * Detectable sensor errors on startup:
     *   1. I2C detection failure: colorSensor.begin() returned false (missing pull-ups, wrong ID,
     *      wiring issue, or bus collision with VL53L0X).
     *   2. Optical / ADC test failure: briefly flash the D4 LED (e.g. 60 ms) and sample RGBC:
     *      - If clear channel == 0 or 0xFFFF: ADC conversion or bus stall error.
     *      - If clear channel does not increase when LED is pulsed: LED or photodiode failure.
     *
     * Proposed audio/visual error code patterns:
     *   - 4 rapid error beeps (e.g., 1500 Hz for 80 ms, 50 ms pause x4) instead of long sluggish
     *     beeps so startup stays snappy.
     *   - Dual headlight blink (e.g. flash red 3 times) to indicate color sensor fault visually.
     *
     * Example prototype snippet:
     *
     *   if (!colorSensorDetected) {
     *     DBGLN(F("ERR: TCS34725 not detected on boot"));
     *     // Visual: Flash red headlight error indication
     *     SetRGBColor(RgbColor::Red);
     *     // Audio: 4 short diagnostic warning beeps {duration_ms, pause_ms, ...}
     *     const int pattern_sensorErr[] = { 80, 60, 80, 60, 80, 60, 80, 0 };
     *     playPattern(pattern_sensorErr);
     *   } else {
     *     // Optional optical self-test:
     *     // 1. colorSensor.enable();
     *     // 2. digitalWrite(pinColorSensorLED, colorSensorLEDOnLevel);
     *     // 3. delay(60); // 50ms integration time + margin
     *     // 4. uint16_t r, g, b, c; colorSensor.getRawData(&r, &g, &b, &c);
     *     // 5. digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);
     *     // 6. colorSensor.disable();
     *     // 7. if (c == 0 || c == 0xFFFF) { ... signal optical fault ... }
     *   }
     * ========================================================================================== */

    initDistanceSensorHardware();

    //playPattern(pattern_melody);  // Play melody
    SetGreenLightValue(0);
    SetRGBColor(FrontLightOnOff ? RgbColor::Red : RgbColor::Off);
    playToneSequence_P(melodyDemo, false);

    // Measure battery at startup
    batteryVoltage = getBatteryVoltageDirect();
    DBG_OTHER_SENSORS(F("Battery measured: "));
    DBGLN_OTHER_SENSORS(batteryVoltage, 2);

    // Configure dynamic speed steps
    configureSpeedSteps();

    lastActive = millis();            // seed idle timer
    IrReceiver.begin(pinIRReceiver, DISABLE_LED_FEEDBACK);  // Keep D13 dedicated to DRV8833 nSLEEP.
  }

  // ================================================================================================
  // Main Loop
  // ================================================================================================
  // Cooperative scheduler: each block owns one concern and must stay fast/non-blocking.
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
    updateColorSensor();

    // Non-blocking playback ===
    updateBuzzer();
    updateSiren();
    updateMelody();
    updateGreenBlink();
    updateMotorReverseCooldown();

    delay(10);  // small loop delay
  }

  // ================================================================================================
  // Idle / Sleep
  // ================================================================================================
  // Enter the lowest-power idle mode after an extended period without user activity.
  void goToIdle() {
    DBGLN_OTHER_SENSORS(F("Idle: turning everything off..."));

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

    DBGLN_OTHER_SENSORS(F("Entering sleep mode..."));

    digitalWrite(pinMotorSleep, LOW);  // Disable the DRV8833 while the Arduino sleeps.
    digitalWrite(pinVL53L0X_XSHUT, LOW); // Turn off VL53L0X for sleep
    attachInterrupt(digitalPinToInterrupt(pinIRReceiver), wakeUp, CHANGE);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(digitalPinToInterrupt(pinIRReceiver));

    digitalWrite(pinVL53L0X_XSHUT, HIGH); // Wake up VL53L0X
    delay(10); // boot time
    distanceTof.setAddress(vl53l0xAddress);
    distanceTof.startContinuous(50);

    digitalWrite(pinMotorSleep, HIGH);
    delay(1);  // DRV8833 wake-up time before checking its diagnostic output.
    initBatteryVoltageMeterHardware();  // ADC was powered down; re-establish the 1.1V reference.
    lastActive = millis();
  }

  void wakeUp() {}

  // Measure battery voltage directly under the current load.
  float getBatteryVoltageDirect() {
    float sum = 0.0;
    unsigned long rawSum = 0;

    // Throw away the first conversion so the ADC sampling capacitor settles on the battery divider.
    analogRead(pinBatterySense);

    for (uint8_t sampleIndex = 0; sampleIndex < BATTERY_ADC_SAMPLES; ++sampleIndex) {
      int rawValue = analogRead(pinBatterySense);
      rawSum += rawValue;
      float dividerRatio = (R1 + R2) / R2;
      float pinVoltage = (rawValue * BATTERY_ADC_REF_VOLTAGE) / BATTERY_ADC_MAX;
      sum += pinVoltage * dividerRatio * BATTERY_ADC_CORRECTION_FACTOR;
    }

    float averagedVoltage = sum / BATTERY_ADC_SAMPLES;

    #if DEBUG_OTHER_SENSORS
    unsigned long now = millis();
    if (now - lastBatteryDebugPrintMs >= 1000UL) {
      lastBatteryDebugPrintMs = now;
      float averageRaw = rawSum / (float)BATTERY_ADC_SAMPLES;
        DBG_OTHER_SENSORS(F("Battery ADC avg raw="));
        DBG_OTHER_SENSORS(averageRaw, 1);
        DBG_OTHER_SENSORS(F(" -> V="));
        DBGLN_OTHER_SENSORS(averagedVoltage, 3);
    }
    #endif

    return averagedVoltage;
  }

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
  void updateBatteryGuard() {
    if (Speed != 0 || motorFaultLatched) return;
    unsigned long now = millis();
    if (now - lastBatteryCheckMs < BATTERY_CHECK_INTERVAL_MS) return;
    lastBatteryCheckMs = now;

    float v = getBatteryVoltageDirect();  // motor off => reading is sag-free
    if (v <= 0) return;
    batteryVoltage = v;

    if (v <= BATTERY_LOW_SHUTDOWN) {
      DBGLN_OTHER_SENSORS(F("Battery SHUTDOWN level: forcing stop"));
      setColorSensorEnabled(false);
      exitAutoDistanceMode(false);
      cancelJog();
      stopAndResetStepSelection();
      SetRGBColor(RgbColor::Red);
      playPattern(pattern_batteryWarn);
    } else if (v <= BATTERY_LOW_WARNING) {
      DBGLN_OTHER_SENSORS(F("Battery WARNING level"));
      setColorSensorEnabled(false);
      SetRGBColor(RgbColor::Red);
      playPattern(pattern_batteryWarn);
    }
  }

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
  void increaseStep() {
    if (currentStep < 3) currentStep++;
    applySpeedStep();
  }
  void decreaseStep() {
    if (currentStep > 0) currentStep--;
    applySpeedStep();
  }

  // ================================================================================================
  // Auto-distance speed control
  // ================================================================================================
  // Convert obstacle distance into a motor-voltage target with a dead zone near obstacles.
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

  int getDistanceReading() {
    if (!distanceTofDetected) return AUTO_DISTANCE_MAX_SPEED;

    unsigned long now = millis();
    if (now - lastTofReadMs < tofReadEveryMs) return Distance;
    lastTofReadMs = now;

    uint16_t rawMm = distanceTof.readRangeContinuousMillimeters();
    if (distanceTof.timeoutOccurred()) return AUTO_DISTANCE_MAX_SPEED;

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

  void initDistanceSensorHardware() {
    distanceTof.setTimeout(50);
    distanceTofDetected = distanceTof.init();
    if (!distanceTofDetected) {
      DBGLN_DISTANCE_SENSOR(F("VL53L0X not detected on I2C"));
      return;
    }
    distanceTof.setAddress(vl53l0xAddress);
    distanceTof.setMeasurementTimingBudget(33000UL);
    distanceTof.startContinuous(50);
    DBGLN_DISTANCE_SENSOR(F("VL53L0X ready"));
  }

  // ================================================================================================
  // IR Receive (NEC with repeat support)
  // ================================================================================================
    // Wrap IRremote so the rest of the sketch sees a simple 8-bit command stream.
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
            DBGLN_REMOTE(received, HEX);
          }
      }
        IrReceiver.resume();
    }
      return received; // 0 = no key
  }

  // ================================================================================================
  // DRV8833 Safety
  // ================================================================================================
    // Identify the subset of buttons that are allowed to bring the motor driver back online.
  bool isMotorControlCommand(uint8_t code) {
    return code == buttonCHminus || code == buttonCH || code == buttonCHplus
      || code == buttonBackward || code == buttonForward || code == buttonPlayPause;
  }

  // nFAULT is a binary driver-protection signal, not an analog current measurement.
  // Once latched, the user must issue a fresh motor command after the fault clears.
  void updateMotorFault() {
    if (motorFaultLatched || digitalRead(pinMotorFault) == HIGH) return;

    Stop();  // Coast the motor before disabling the driver.
    digitalWrite(pinMotorSleep, LOW);
    exitAutoDistanceMode();
    cancelJog();
    currentStep = 0;
    motorFaultLatched = true;

    sirenActive = false;
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    SetGreenLightValue(0);
    SetRGBLightColor(RgbColor::Red);  // Bypass siren suppression for a safety indication.
    playPattern(pattern_batteryWarn);
    DBGLN_MOTOR(F("DRV8833 fault: motor disabled until a motor command re-arms it"));
  }

  bool rearmMotorDriver() {
    digitalWrite(pinVL53L0X_XSHUT, HIGH); // Wake up VL53L0X
    delay(10); // boot time
    distanceTof.setAddress(vl53l0xAddress);
    distanceTof.startContinuous(50);

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
  void setMotor(Dir dir, int speed) {
    int safeSpeed = constrain(speed, 0, 255);
    switch (dir) {
      case Dir::Forward:
        analogWrite(pinMotor_IN1, safeSpeed);
        analogWrite(pinMotor_IN2, 0);
        MotorDirection = 1;
        DBG_MOTOR(F("Driving Forward >>> Speed="));
        DBGLN_MOTOR(safeSpeed);
        break;
      case Dir::Backward:
        analogWrite(pinMotor_IN1, 0);
        analogWrite(pinMotor_IN2, safeSpeed);
        MotorDirection = 2;
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

  void Stop() {
    setMotor(Dir::Stop, 0);
    motorDrivePending = false;  // cancel any deferred reverse drive
  }

  // Apply a reverse drive that was deferred during the non-blocking direction cooldown.
  void updateMotorReverseCooldown() {
    if (!motorDrivePending) return;
    if ((long)(millis() - motorReverseReadyAt) < 0) return;
    motorDrivePending = false;
    if (Speed <= 0) return;
    if (MotorDirection == 1) setMotor(Dir::Forward, Speed);
    else if (MotorDirection == 2) setMotor(Dir::Backward, Speed);
  }

  // Jog motor briefly without changing the stored direction/speed state machine.
  void JogDrive(Dir dir) {
    int jogPWM = pwmSteps[1];
    if (dir == Dir::Forward) {
      analogWrite(pinMotor_IN1, jogPWM);
      analogWrite(pinMotor_IN2, 0);
    } else if (dir == Dir::Backward) {
      analogWrite(pinMotor_IN1, 0);
      analogWrite(pinMotor_IN2, jogPWM);
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
  void translateIR() {
    uint8_t code = irReceive();

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
          DBG_OTHER_SENSORS(F("Battery Voltage: "));
          DBGLN_OTHER_SENSORS(vIn, 1);
          DBG_OTHER_SENSORS(F("Battery level: "));
          DBG_OTHER_SENSORS(batteryPercent);
          DBGLN_OTHER_SENSORS(F("%"));
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
  inline void clearBuzzerPattern() {
    for (int j = 0; j < BUZZER_PATTERN_MAX; ++j) buzzerPattern[j] = 0;
    buzzerIndex = 0;
  }

  // Advance the current buzzer pattern one timing step at a time.
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
  inline void writeTrainOutput(const LedRoute& route, bool Value) {
    if (trainLedExpanderDetected) {
      trainLedExpander.digitalWrite(route.expanderPin, Value ? HIGH : LOW);
    } else {
      DBGLN_LEDS(String(F("Expander pin GP")) + String(route.expanderPin) + F(" to ") + String(Value ? "ON" : "OFF"));
    }
  }

  inline void writeRGBPins(const LedRoute& rPin, const LedRoute& gPin, const LedRoute& bPin, bool R, bool G, bool B) {
    // HIGH drives the BC547 base, which sinks the selected battery-powered LED channel.
    writeTrainOutput(rPin, R);
    writeTrainOutput(gPin, G);
    writeTrainOutput(bPin, B);
  }

  // Apply raw RGB channel states to one headlight or both.
  void SetRGBLight(bool R, bool G, bool B, int led) {
    if (FrontLightOnOff == 0) return;
    if (led == 0 || led == 1) writeRGBPins(led1R, led1G, led1B, R, G, B);
    if (led == 0 || led == 2) writeRGBPins(led2R, led2G, led2B, R, G, B);
  }

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
  inline void SetRGBColor(RgbColor color, int led) {
    if (sirenActive) return;  // ignore during siren
    if (ColorSensorOnOff == 1 && color != RgbColor::Off) {
      SetRGBLightColor(RgbColor::Cyan, led);
      return;
    }
    SetRGBLightColor(color, led);
  }

  void SetGreenLightValue(int Value) {
    // analogWrite(pinLEDGreen, Value);
    // Active-HIGH matches the BC547 low-side switch used by the green LED.
    writeTrainOutput(ledGreen, (Value > 0));  // Any non-zero means ON
  }

  // Start a non-blocking 2-pulse acknowledgement blink; final state restored by updateGreenBlink().
  void GreenLEDBlink() {
    greenBlinkRemaining = 2;
    greenBlinkOn = true;
    greenBlinkStepMs = millis();
    SetGreenLightValue(255);
  }

  // Advance the acknowledgement blink and restore the auto-mode indicator when finished.
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

  struct LedRoute {
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
  // Wiring: D3 -> tilt switch -> GND. No external resistor or capacitor is required in this revision.
  // D3 uses the internal pull-up, so a closed tilt switch pulls the input LOW when active.
  // Note: tilt doesn't disable the siren
  // ================================================================================================
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
        DBGLN_OTHER_SENSORS(F("TILT: ACTIVE -> emergency stop"));
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
        DBGLN_OTHER_SENSORS(F("TILT: IDLE -> clear latch, restore LEDs"));
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
  void stopMelody() {
    noTone(pinBuzzer);
    melodyPlaying = false;
    melodyLenPairs = 0;
    melodyIdxPair = 0;
    melodyStepStarted = 0;
  }

  // Advance the current melody using millis()-based timing instead of delay().
  void updateMelody() {
    if (!melodyPlaying) return;  // Exit if no melody is playing
    if (SoundOnOff != 1 || sirenActive) {
      stopMelody();
      return;
    }  // Stop if sound is muted or siren is active

    unsigned long now = millis();

    // Start the first note
    if (melodyStepStarted == 0) {
      int f = melodySeq[melodyIdxPair * 2];      // Frequency
      int d = melodySeq[melodyIdxPair * 2 + 1];  // Duration
      if (f > 0) tone(pinBuzzer, f);
      else noTone(pinBuzzer);
      melodyStepStarted = now;
      return;
    }

    // Continue checking if the current note's duration has elapsed
    int dCur = melodySeq[melodyIdxPair * 2 + 1];
    if (now - melodyStepStarted >= (unsigned long)dCur) {
      melodyIdxPair++;  // Move to the next note
      if (melodyIdxPair >= melodyLenPairs) {
        if (melodyLoop) melodyIdxPair = 0;  // Loop back if enabled
        else {
          stopMelody();
          return;
        }  // Stop if done
      }
      int f = melodySeq[melodyIdxPair * 2];
      int d = melodySeq[melodyIdxPair * 2 + 1];
      if (f > 0) tone(pinBuzzer, f);
      else noTone(pinBuzzer);
      melodyStepStarted = now;
    }
  }

  // Copy a flat [frequency, duration] sequence into the playback buffer and arm playback.
  // Supports either RAM-backed arrays or PROGMEM-backed arrays depending on isProgmem.
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

    // Copy melody data into RAM buffer
    if (isProgmem) {
      // If sequence is in PROGMEM
  #if defined(__AVR__)
      // AVR platform: use memcpy_P to copy from flash
      memcpy_P(melodySeq, (const void*)seqFD, (size_t)melodyLenPairs * 2 * sizeof(int16_t));
  #else
      // Non-AVR platform: treat PROGMEM as regular RAM
      for (int i = 0; i < melodyLenPairs * 2; i++) {
        melodySeq[i] = seqFD[i];
      }
  #endif
    } else {
      // Sequence is already in RAM
      for (int i = 0; i < melodyLenPairs * 2; i++) {
        melodySeq[i] = seqFD[i];
      }
    }

    // Initialize playback state
    melodyIdxPair = 0;
    melodyLoop = loopPlayback;
    melodyPlaying = true;
    melodyStepStarted = 0;  // Triggers first note on next update
  }
