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
  //  8. Ultrasonic auto-speed and distance filtering
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
  //   SpeedAutoUltrasonic()        - Run the automatic speed controller.
  //   updateMotorSpeed()           - Ramp the motor toward a requested speed.
  //   Distancia_test()             - Trigger one ultrasonic ping and return distance in cm.
  //   getMedianDistance()          - Filter ultrasonic readings with a rolling median.
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
  //   blinkLED() / GreenLEDBlink() - Short visual feedback helpers.
  //   updateSiren()                - Animate siren lights and pitch sweep.
  //   updateTiltSensor()           - Debounce tilt input and latch emergency stop.
  //   stopMelody()                 - Stop melody playback and clear melody state.
  //   updateMelody()               - Advance non-blocking melody playback.
  //   playToneSequenceRaw()        - Load and start a melody from RAM or PROGMEM.

  // ===============================================================================================
  // DEBUG flag: set to 1 to enable serial output for debugging, set to 0 to save flash for production.
  // Wrapped in ifndef so arduino-cli can override it for size comparisons.
  #ifndef DEBUG
  #define DEBUG 0
  #endif

  // Conditional macros for debug logging
  #if DEBUG
  #define DBG(...) Serial.print(__VA_ARGS__)      // Print without newline
  #define DBGLN(...) Serial.println(__VA_ARGS__)  // Print with newline
  #define DBGBEGIN(...) \
    do { Serial.begin(__VA_ARGS__); } while (0)  // Initialize Serial
  #else
  #define DBG(...)       // No operation
  #define DBGLN(...)     // No operation
  #define DBGBEGIN(...)  // No operation
  #endif
  // ===============================================================================================

  // AVR-specific include for program memory storage
  #if defined(__AVR__)
  #include <avr/pgmspace.h>
  #endif

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
  void setColorSensorEnabled(bool enabled);
  void updateColorSensor();
  void refreshDriveLights();
  float getBatteryVoltageSettledForStatus();
  uint8_t classifyTrackMarkerColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
  void handleTrackMarkerAction(uint8_t markerClass);
  #if DEBUG
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
  //   D3  -> Tilt sensor input with internal pull-up; switch closes to GND.
  //   D4  -> TCS34725 breakout LED control output.
  //   D5  -> DRV8833 IN1 motor drive PWM/direction output.
  //   D6  -> DRV8833 IN2 motor drive PWM/direction output.
  //   D7  -> Free general-purpose digital pin.
  //   D8  -> Free general-purpose digital pin.
  //   D9  -> Free digital-only pin; do not use analogWrite because IRremote owns Timer1.
  //   D10 -> Free digital-only pin; do not use analogWrite because IRremote owns Timer1.
  //   D11 -> Free general-purpose digital pin if SPI is not needed; also SPI MOSI.
  //   D12 -> Active buzzer output; also SPI MISO.
  //   D13 -> DRV8833 nSLEEP output; also tied to the Nano onboard LED and SPI SCK.
  //   A0  -> Battery voltage sense analog input.
  //   A1  -> Ultrasonic echo input.
  //   A2  -> DRV8833 nFAULT input with internal pull-up; active LOW.
  //   A3  -> Ultrasonic trigger output.
  //   A4  -> I2C SDA shared by the MCP23008 LED expander and TCS34725 color sensor.
  //   A5  -> I2C SCL shared by the MCP23008 LED expander and TCS34725 color sensor.
  //   A6  -> Free analog-input-only pin.
  //   A7  -> Free analog-input-only pin.
  // SPI note: SPI is not used in this sketch, but an SPI peripheral would conflict with the buzzer
  // on D12 and DRV8833 nSLEEP on D13.
  
  const int pinBatterySense = A0;   // Battery voltage monitoring
  const int pinTiltSensor = 3;      // Tilt sensor, digital input with internal pull-up; switch closes to GND
  const int pinUltrasonicTrig = A3; // Ultrasonic sensor trigger
  const int pinUltrasonicEcho = A1; // Ultrasonic sensor echo
  const int pinIRReceiver = 2;      // IR receiver input: D2 or D3 required to wake from sleep
  const int pinEngineA_1A = 5;      // DRV8833 IN1: motor direction and speed (PWM)
  const int pinEngineA_1B = 6;      // DRV8833 IN2: motor direction and speed (PWM)
  const int pinMotorFault = A2;     // DRV8833 ULT/nFAULT, active-LOW diagnostic output
  const int pinMotorSleep = 13;     // DRV8833 EEP/nSLEEP, HIGH = enabled, LOW = sleep
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
  const int pinBuzzer = 12;                                // Active buzzer (with generator)

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
  const float R1 = 10000.0;                // Top resistor (to battery +)
  const float R2 = 4700.0;                 // Bottom resistor (to GND)
  const int AUTO_SAMPLES_FOR_MEDIAN = 5;   // number of samples for median filter
  const int AUTO_DISTANCE_STOP = 8;        // distance to obstacle <= cm to stop the train
  const int AUTO_DISTANCE_RESTART = 11;    // distance to obstacle >= cm to re-start the train
  const int AUTO_DISTANCE_MAX_SPEED = 50;  // distance to obstacle >= cm to run at max speed
  const float BATTERY_LOW_WARNING = 7.4;   // warn at this voltage
  const float BATTERY_LOW_SHUTDOWN = 7.2;  // force stop at this voltage
  const float MAX_SAFE_MOTOR_VOLTAGE = 6.0;

  // ================================================================================================
  // Control variables
  // ================================================================================================
  int FrontLightOnOff = 1;  // Front lights state (ON/OFF) - RGB LEDs only
  int SoundOnOff = 1;       // Sound state (ON/OFF)
  int UltrasonicOnOff = 0;  // Ultrasonic state (ON/OFF)
  int ColorSensorOnOff = 0; // Color sensor state (ON/OFF)
  int MotorDirection = 1;   // always has a direction
  int Speed = 0;            // stopped at start
  int Distance = 0;         // Latest distance from ultrasonic

  // --- Distance median filter state ---
  int distanceBuffer[AUTO_SAMPLES_FOR_MEDIAN];
  int bufferIndex = 0;
  bool bufferFilled = false;

  // --- Motor driver fault state ---
  bool motorFaultLatched = false;

  // --- Momentary jog state ---
  bool momentaryActive = false;
  uint16_t momentaryButton = 0;
  unsigned long momentaryLastSeen = 0;
  const unsigned long momentaryTimeout = 200;  // ms after last repeat → stop

  // --- IR repeat tracking ---
  uint8_t lastIRCommand = 0;
  bool lastWasRepeat = false;

  // Timers
  unsigned long lastActive = 0;
  const unsigned long idleTimeout = 5UL * 60UL * 1000UL;  // 5 minutes
  unsigned long lastColorSensorRead = 0;
  const unsigned long colorSensorReadEveryMs = 250;
  uint8_t lastTrackMarkerClass = 255;

  // TCS34725 classification thresholds for the white/red/green/blue/yellow marker scheme (no black).
  // Mounting: aim for ~3 mm sensor-to-marker gap (usable 2-5 mm), fixed height and angle, LED on.
  // Closer than 2 mm risks scraping; beyond 5 mm ambient light and edge blur reduce reliability.
  // Use matte, saturated markers large enough that the sensor sees one color at a time.
  // Tune these after capturing debug RGBC logs from the real track, floor, and lighting.
  const uint16_t colorClearMinThreshold = 120;   // below this the reading is too dark to trust -> no marker
  const uint16_t colorClearWhiteThreshold = 2200;
  const uint16_t colorWhiteChannelThreshold = 700;
  const uint8_t colorYellowMinRedPct = 44;
  const uint8_t colorYellowMinGreenPct = 30;
  const uint8_t colorYellowMaxBluePct = 20;
  const uint8_t colorRedMinPct = 45;
  const uint8_t colorRedVsGreenPct = 120;
  const uint8_t colorRedVsBluePct = 135;
  const uint8_t colorGreenMinPct = 40;
  const uint8_t colorGreenVsRedPct = 110;
  const uint8_t colorGreenVsBluePct = 110;
  const uint8_t colorBlueMinPct = 38;
  const uint8_t colorBlueVsRedPct = 120;
  const uint8_t colorBlueVsGreenPct = 110;

  // Per-color marker action toggles; flip to false to disable that color's action.
  // Actions run only on color transitions, so a long marker does not keep retriggering.
  // Action bodies are placeholders for now - define each behavior in handleTrackMarkerAction().
  const bool actionWhiteMarker = true;
  const bool actionRedMarker = true;
  const bool actionGreenMarker = true;
  const bool actionBlueMarker = true;
  const bool actionYellowMarker = true;

  // Speed steps (voltage → PWM)
  int pwmSteps[4];  // 0..3
  float voltageSteps[] = { 0.0, 3.5, 4.5, 6.0 };
  float batteryVoltage = 0.0;

  // Buzzer
  #define BUZZER_PATTERN_MAX 20
  int buzzerPattern[BUZZER_PATTERN_MAX];
  int buzzerIndex = 0;
  unsigned long buzzerTimer = 0;

  // Manual step index
  int currentStep = 0;  // 0=stop, 1=~3.5V, 2=~4.5V, 3=~6V

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
      DBGLN(F("MCP23008 not found; train LEDs expect the expander wiring"));
      return;
    }

    const LedRoute routes[] = { led1R, led1G, led1B, led2R, led2G, led2B, ledGreen };
    for (uint8_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
      trainLedExpander.pinMode(routes[i].expanderPin, OUTPUT);
      trainLedExpander.digitalWrite(routes[i].expanderPin, LOW);
    }

    DBGLN(F("MCP23008 ready for train LEDs"));
  }

  void initColorSensorHardware() {
    pinMode(pinColorSensorLED, OUTPUT);
    digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);

    colorSensorDetected = colorSensor.begin();
    if (colorSensorDetected) {
      DBGLN(F("TCS34725 ready"));
    } else {
      DBGLN(F("TCS34725 not detected on I2C"));
    }
  }

  void setColorSensorEnabled(bool enabled) {
    if (enabled && !colorSensorDetected) {
      DBGLN(F("Ignored: TCS34725 not detected"));
      return;
    }

    ColorSensorOnOff = enabled ? 1 : 0;
    digitalWrite(pinColorSensorLED, enabled ? colorSensorLEDOnLevel : colorSensorLEDOffLevel);

    if (enabled) {
      DBGLN(F("Color sensor ON"));
      if (!sirenActive) SetRGBLightColor(RgbColor::Cyan);
    } else {
      DBGLN(F("Color sensor OFF"));
      refreshDriveLights();
    }
  }

  enum TrackMarkerClass : uint8_t {
    MarkerUnknown = 0,
    MarkerWhite,
    MarkerRed,
    MarkerGreen,
    MarkerBlue,
    MarkerYellow
  };

  uint8_t classifyTrackMarkerColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    if (c < colorClearMinThreshold) return MarkerUnknown;  // too dark / no marker present
    if (c > colorClearWhiteThreshold && r > colorWhiteChannelThreshold && g > colorWhiteChannelThreshold && b > colorWhiteChannelThreshold) return MarkerWhite;

    uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;
    if (sum == 0) return MarkerUnknown;

    if ((uint32_t)r * 100UL > (uint32_t)colorYellowMinRedPct * sum
      && (uint32_t)g * 100UL > (uint32_t)colorYellowMinGreenPct * sum
      && (uint32_t)b * 100UL < (uint32_t)colorYellowMaxBluePct * sum) return MarkerYellow;

    if ((uint32_t)r * 100UL > (uint32_t)colorRedMinPct * sum
      && (uint32_t)r * 100UL > (uint32_t)colorRedVsGreenPct * g
      && (uint32_t)r * 100UL > (uint32_t)colorRedVsBluePct * b) return MarkerRed;

    if ((uint32_t)g * 100UL > (uint32_t)colorGreenMinPct * sum
      && (uint32_t)g * 100UL > (uint32_t)colorGreenVsRedPct * r
      && (uint32_t)g * 100UL > (uint32_t)colorGreenVsBluePct * b) return MarkerGreen;

    if ((uint32_t)b * 100UL > (uint32_t)colorBlueMinPct * sum
      && (uint32_t)b * 100UL > (uint32_t)colorBlueVsRedPct * r
      && (uint32_t)b * 100UL > (uint32_t)colorBlueVsGreenPct * g) return MarkerBlue;

    return MarkerUnknown;
  }

  #if DEBUG
  const __FlashStringHelper* trackMarkerLabel(uint8_t markerClass) {
    switch (markerClass) {
      case MarkerWhite: return F("white");
      case MarkerRed: return F("red");
      case MarkerGreen: return F("green");
      case MarkerBlue: return F("blue");
      case MarkerYellow: return F("yellow");
      default: return F("unknown");
    }
  }
  #endif

  void handleTrackMarkerAction(uint8_t markerClass) {
    switch (markerClass) {
      case MarkerWhite:
        if (!actionWhiteMarker) return;
        DBGLN(F("Track action: WHITE marker (no action defined yet)"));
        break;

      case MarkerRed:
        if (!actionRedMarker) return;
        DBGLN(F("Track action: RED marker (no action defined yet)"));
        break;

      case MarkerGreen:
        if (!actionGreenMarker) return;
        DBGLN(F("Track action: GREEN marker (no action defined yet)"));
        break;

      case MarkerBlue:
        if (!actionBlueMarker) return;
        DBGLN(F("Track action: BLUE marker (no action defined yet)"));
        break;

      case MarkerYellow:
        if (!actionYellowMarker) return;
        DBGLN(F("Track action: YELLOW marker (no action defined yet)"));
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
    #if DEBUG
    const __FlashStringHelper* markerLabel = trackMarkerLabel(markerClass);
    DBG(F("TCS34725 RGBC: "));
    DBG(r);
    DBG(F(", "));
    DBG(g);
    DBG(F(", "));
    DBG(b);
    DBG(F(", clear="));
    DBG(c);
    DBG(F(" -> "));
    DBGLN(markerLabel);
    #endif

    if (markerClass != lastTrackMarkerClass) {
      lastTrackMarkerClass = markerClass;
      #if DEBUG
      DBG(F("Track marker changed to: "));
      DBGLN(markerLabel);
      #endif
      handleTrackMarkerAction(markerClass);
    }
  }

  void refreshDriveLights() {
    if (sirenActive) return;

    SetGreenLightValue(UltrasonicOnOff ? 255 : 0);

    if (ColorSensorOnOff == 1) {
      SetRGBLightColor(RgbColor::Cyan);
      return;
    }

    if (Speed == 0) SetRGBColor(RgbColor::Red);
    else if (MotorDirection == 2) SetRGBColor(RgbColor::Blue);
    else SetRGBColor(RgbColor::White);
  }

  // One-time hardware initialization and startup behavior.
  void setup() {

    DBGBEGIN(9600);

    // Initialize Arduino pins
    initTrainLedHardware();
    pinMode(pinBuzzer, OUTPUT);
    pinMode(pinEngineA_1A, OUTPUT);
    pinMode(pinEngineA_1B, OUTPUT);
    pinMode(pinMotorFault, INPUT_PULLUP);
    pinMode(pinMotorSleep, OUTPUT);
    digitalWrite(pinMotorSleep, HIGH);  // Enable the DRV8833 after power-up.
    pinMode(pinUltrasonicTrig, OUTPUT);
    digitalWrite(pinUltrasonicTrig, LOW);
    pinMode(pinUltrasonicEcho, INPUT);
    //pinMode(pinIRReceiver, INPUT);
    pinMode(pinBatterySense, INPUT);
    pinMode(pinTiltSensor, INPUT_PULLUP);
    initColorSensorHardware();

    //playPattern(pattern_melody);  // Play melody
    SetGreenLightValue(0);
    SetRGBColor(FrontLightOnOff ? RgbColor::Red : RgbColor::Off);
    playToneSequence_P(melodyDemo, false);

    // Measure battery at startup
    batteryVoltage = getBatteryVoltageDirect();
    DBG(F("Battery measured: "));
    DBGLN(batteryVoltage, 2);

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

    // === 3. Jog watchdog (safety if button released) ===
    if (!motorFaultLatched && momentaryActive && (millis() - momentaryLastSeen > momentaryTimeout)) {
      DBGLN(F("Jog watchdog timeout → STOP"));
      Stop();
      SetRGBColor(RgbColor::Red);
      momentaryActive = false;
    }

    // === 4. Auto-speed mode ===
    if (!motorFaultLatched && UltrasonicOnOff == 1) {
      SpeedAutoUltrasonic();
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

    delay(10);  // small loop delay
  }

  // ================================================================================================
  // Idle / Sleep
  // ================================================================================================
  // Enter the lowest-power idle mode after an extended period without user activity.
  void goToIdle() {
    DBGLN(F("Idle: turning everything off..."));

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

    DBGLN(F("Entering sleep mode..."));

    digitalWrite(pinMotorSleep, LOW);  // Disable the DRV8833 while the Arduino sleeps.
    attachInterrupt(digitalPinToInterrupt(pinIRReceiver), wakeUp, CHANGE);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(digitalPinToInterrupt(pinIRReceiver));

    digitalWrite(pinMotorSleep, HIGH);
    delay(1);  // DRV8833 wake-up time before checking its diagnostic output.
    lastActive = millis();
  }

  void wakeUp() {}

  // Measure battery voltage directly under the current load.
  float getBatteryVoltageDirect() {
    // 5V ADC reference scaled back up through the R1/R2 divider, folded into one constant.
    static const float kBatteryAdcScale = (5.0 / 1023.0) * ((R1 + R2) / R2);
    return analogRead(pinBatterySense) * kBatteryAdcScale;
  }

  float getBatteryVoltageSettledForStatus() {
    UltrasonicOnOff = 0;
    momentaryActive = false;
    currentStep = 0;
    Speed = 0;
    MotorDirection = 1;
    ColorSensorOnOff = 0;
    sirenActive = false;
    stopMelody();
    noTone(pinBuzzer);
    clearBuzzerPattern();

    SetRGBColor(RgbColor::Off);
    SetGreenLightValue(0);
    Stop();
    if (pinColorSensorLED >= 0) {
      digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);
    }

    delay(200);  // Blocking: let the pack settle unloaded before sampling for an accurate reading.
    return getBatteryVoltageDirect();
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
    for (int i = 0; i < 4; i++) {
      pwmSteps[i] = safePWMFromVoltage(voltageSteps[i]);
    }
    DBGLN(F("Configured speed steps (PWM values):"));
    for (int i = 0; i < 4; i++) {
      DBG(min(MAX_SAFE_MOTOR_VOLTAGE, voltageSteps[i]));
      DBG(F("V → PWM "));
      DBGLN(pwmSteps[i]);
    }
  }

  void playStepBeep(int step) {
    if (SoundOnOff != 1) return;  // respect mute
    for (int j = 0; j < 20; j++) buzzerPattern[j] = 0;
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

    DBG(F("Step "));
    DBG(currentStep);
    DBG(F(": target "));
    DBG(voltageSteps[currentStep]);
    DBG(F("V → PWM "));
    DBGLN(Speed);

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
  // Ultrasonic Auto-speed
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
  void SpeedAutoUltrasonic() {
    Distance = getMedianDistance();

    DBG(F("Ultrasonic Distance: "));
    DBG(Distance);
    DBGLN(F(" cm"));

    float targetV = motorVoltageFromDistance(Distance);
    int targetSpeed = safePWMFromVoltage(targetV);

    if (targetSpeed == 0) {
      SetRGBColor(RgbColor::Red);
      DBGLN(F("Auto: STOP"));
    } else {
      SetRGBColor(RgbColor::White);
      DBG(F("Auto target speed = "));
      DBG(targetSpeed);
      DBG(" (≈ ");
      DBG(targetV, 2);
      DBGLN(" V)");
    }

    updateMotorSpeed(targetSpeed);
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

  // Single raw ultrasonic ping. Returns distance in cm, or AUTO_DISTANCE_MAX_SPEED on no-echo.
  int Distancia_test() {
    // Proper trigger: LOW→(2 µs)→HIGH(10 µs)→LOW
    digitalWrite(pinUltrasonicTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(pinUltrasonicTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinUltrasonicTrig, LOW);

    // Bigger timeout for debugging (e.g., 30 ms ≈ max ~5 m)
    unsigned long duration = pulseIn(pinUltrasonicEcho, HIGH, 30000UL);

    if (duration == 0) return AUTO_DISTANCE_MAX_SPEED;   // treat "no echo" as far
    int cm = (int)(duration / 58UL);
    if (cm > AUTO_DISTANCE_MAX_SPEED) cm = AUTO_DISTANCE_MAX_SPEED;
    return cm;
  }

  // Rolling median removes single bad echoes without adding a large control delay.
  int getMedianDistance() {

    static unsigned long lastSample = 0;
    unsigned long now = millis();
    // Throttle to ~50 ms between pings so echoes settle and readings stay stable.
    if (now - lastSample < 50) return Distance;
    lastSample = now;

    int raw = Distancia_test();         // raw single measurement
    distanceBuffer[bufferIndex] = raw;  // insert into buffer
    bufferIndex = (bufferIndex + 1) % AUTO_SAMPLES_FOR_MEDIAN;
    if (bufferIndex == 0) bufferFilled = true;

    int size = bufferFilled ? AUTO_SAMPLES_FOR_MEDIAN : bufferIndex;
    int temp[AUTO_SAMPLES_FOR_MEDIAN];
    for (int i = 0; i < size; i++) temp[i] = distanceBuffer[i];

    // simple bubble sort for median
    for (int i = 0; i < size - 1; i++) {
      for (int j = i + 1; j < size; j++) {
        if (temp[j] < temp[i]) {
          int swap = temp[i];
          temp[i] = temp[j];
          temp[j] = swap;
        }
      }
    }
    int median = temp[size / 2];

    // Serial debug
    DBG(F("Ultrasonic raw="));
    DBG(raw);
    DBG(F(" cm  |  median="));
    DBG(median);
    DBGLN(F(" cm"));

    Distance = median;
    return median;
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
            DBGLN(received, HEX); // keep this for now
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
    UltrasonicOnOff = 0;
    momentaryActive = false;
    currentStep = 0;
    motorFaultLatched = true;

    sirenActive = false;
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    SetGreenLightValue(0);
    SetRGBLightColor(RgbColor::Red);  // Bypass siren suppression for a safety indication.
    playPattern(pattern_batteryWarn);
    DBGLN(F("DRV8833 fault: motor disabled until a motor command re-arms it"));
  }

  bool rearmMotorDriver() {
    digitalWrite(pinMotorSleep, HIGH);
    delay(1);  // Allow nSLEEP to release and nFAULT to report the current condition.
    if (digitalRead(pinMotorFault) == LOW) {
      digitalWrite(pinMotorSleep, LOW);
      DBGLN(F("DRV8833 fault is still active"));
      return false;
    }

    motorFaultLatched = false;
    DBGLN(F("DRV8833 fault cleared; motor command accepted"));
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
        analogWrite(pinEngineA_1A, safeSpeed);
        analogWrite(pinEngineA_1B, 0);
        MotorDirection = 1;
        DBG(F("Driving Forward >>> Speed="));
        DBGLN(safeSpeed);
        break;
      case Dir::Backward:
        analogWrite(pinEngineA_1A, 0);
        analogWrite(pinEngineA_1B, safeSpeed);
        MotorDirection = 2;
        DBG(F("Driving Backward >> Speed="));
        DBGLN(safeSpeed);
        break;
      default:  // Stop
        analogWrite(pinEngineA_1A, 0);
        analogWrite(pinEngineA_1B, 0);
        Speed = 0;
        DBGLN(F("Motor OFF"));
        break;
    }
  }

  void GoForward() {
    if (MotorDirection == 2 && Speed > 0) {
      DBGLN(F("Ignored: cannot switch to FORWARD while moving"));
      return;
    }
    if (MotorDirection == 2 && Speed == 0) {
      Stop();
      delay(DIR_DELAY);  // Blocking pause lets the motor settle before reversing direction.
    }
    MotorDirection = 1;
    if (Speed > 0) {
      setMotor(Dir::Forward, Speed);
    }
  }

  void GoBackward() {
    if (MotorDirection == 1 && Speed > 0) {
      DBGLN(F("Ignored: cannot switch to BACKWARD while moving"));
      return;
    }
    if (MotorDirection == 1 && Speed == 0) {
      Stop();
      delay(DIR_DELAY);  // Blocking pause lets the motor settle before reversing direction.
    }
    MotorDirection = 2;
    if (Speed > 0) {
      setMotor(Dir::Backward, Speed);
      DBGLN(F("Driving Backward >>"));
    }
  }

  void Stop() {
    setMotor(Dir::Stop, 0);
  }

  // Jog motor briefly without changing the stored direction/speed state machine.
  void JogDrive(Dir dir) {
    int jogPWM = pwmSteps[1];
    if (dir == Dir::Forward) {
      analogWrite(pinEngineA_1A, jogPWM);
      analogWrite(pinEngineA_1B, 0);
    } else if (dir == Dir::Backward) {
      analogWrite(pinEngineA_1A, 0);
      analogWrite(pinEngineA_1B, jogPWM);
    } else {
      analogWrite(pinEngineA_1A, 0);
      analogWrite(pinEngineA_1B, 0);
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
      DBGLN(F("Cancelling jog due to new key"));
      Stop();
      SetRGBColor(RgbColor::Red);
      momentaryActive = false;
    }

    // Ignore repeats for non-jog use cases (prevents CH± spam)
    if (lastWasRepeat && !momentaryActive) {
      return;
    }

    // No new code; if jogging and repeats stopped → timeout
    if (code == 0) {
      if (momentaryActive && (millis() - momentaryLastSeen > momentaryTimeout)) {
        DBGLN(F("Jog timeout → STOP"));
        Stop();
        SetRGBColor(RgbColor::Red);
        momentaryActive = false;
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
            DBGLN(F("Ignored: CH- during jog"));
            break;
          }
          if (UltrasonicOnOff == 0) {
            decreaseStep();
            DBG(F("Manual Speed Down: "));
            DBGLN(Speed);
            if (Speed == 0) {
              Stop();
              SetRGBColor(RgbColor::Red);
            } else {
              if (MotorDirection == 1) GoForward();
              if (MotorDirection == 2) GoBackward();
            }
          } else {
            DBGLN(F("Ignored: Auto-speed active"));
          }
          break;
        }

      case buttonCH:
        {  // Stop
          if (UltrasonicOnOff == 1) {
            UltrasonicOnOff = 0;
            SetGreenLightValue(0);
            DBGLN(F("Switched from AUTO to MANUAL mode"));
          }
          DBGLN(F("STOP pressed → Motors stopped"));
          SetRGBColor(RgbColor::Red);
          MotorDirection = 1;  // default to forward when stopped
          Stop();
          currentStep = 0;
          DBGLN(F("Manual mode reset: next CH+ will start at step 1 (≈3.5V)"));
          break;
        }

      case buttonCHplus:
        {  // Speed +
          if (momentaryActive) {
            DBGLN(F("Ignored: CH+ during jog"));
            break;
          }
          if (UltrasonicOnOff == 0) {
            increaseStep();
            DBG(F("Manual Speed Up: "));
            DBGLN(Speed);
            if (MotorDirection == 1) {
              GoForward();
              SetRGBColor(RgbColor::White);
            }
            if (MotorDirection == 2) {
              GoBackward();
              SetRGBColor(RgbColor::Blue);
            }
          } else {
            DBGLN(F("Ignored: Auto-speed active"));
          }
          break;
        }

      case buttonBackward:
        {  // << momentary backward (jog)
          if (UltrasonicOnOff == 0 && Speed == 0) {
            JogDrive(Dir::Backward);
            SetRGBColor(RgbColor::Blue);
            momentaryActive = true;
            momentaryButton = buttonBackward;
            momentaryLastSeen = millis();
            DBGLN(F("Momentary BACKWARD running (hold to move)"));
          } else {
            DBGLN(F("Ignored: << only when stationary & not in AUTO"));
          }
          break;
        }

      case buttonForward:
        {  // >> momentary forward (jog)
          if (UltrasonicOnOff == 0 && Speed == 0) {
            JogDrive(Dir::Forward);
            SetRGBColor(RgbColor::White);
            momentaryActive = true;
            momentaryButton = buttonForward;
            momentaryLastSeen = millis();
            DBGLN(F("Momentary FORWARD running (hold to move)"));
          } else {
            DBGLN(F("Ignored: >> only when stationary & not in AUTO"));
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
          UltrasonicOnOff = !UltrasonicOnOff;
          SetGreenLightValue(UltrasonicOnOff ? 255 : 0);
          if (UltrasonicOnOff) {
            DBGLN(F("Driving started (auto-speed)"));
            SetRGBColor(RgbColor::White);
            GoForward();
            playPattern(pattern_double);
          } else {
            DBGLN(F("Driving stopped"));
            SetRGBColor(RgbColor::Red);
            Stop();
            Speed = 0;
            playPattern(pattern_descend);
            currentStep = 0;
            DBGLN(F("Manual mode rearmed: next step = 1 (≈3.5V)"));
          }
          break;
        }

      case buttonEQ:
        {  // Mute / Unmute
          SoundOnOff = !SoundOnOff;
          DBGLN(SoundOnOff ? "Sound ON" : "Sound OFF");

          if (!SoundOnOff) {
            // Hard stop any audio that's playing
            noTone(pinBuzzer);
            digitalWrite(pinBuzzer, LOW);
            for (int j = 0; j < 20; j++) buzzerPattern[j] = 0;
            buzzerIndex = 0;
            // DO NOT touch sirenActive or LEDs -> lights continue flashing if sirenActive==true
          } else {
            playPattern(pattern_double);  // short confirmation chirp
          }
          break;
        }

      case button100plus:
        {  // Horn
          DBGLN(F("Horn activated"));
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
          DBGLN(SoundOnOff ? F("Siren ON (with sound)") : F("Siren ON (lights only, muted)"));
        }
        break;

      case button1:
        {
          playToneSequence_P(melodyDemo, false);
          break;
        }

      case button2:
        {
          playToneSequence_P(melodyTwinkle, false);
          break;
        }

      case button3:
        {
          playToneSequence_P(melodyOdeToJoy, false);
          break;
        }

      case button4:
        {
          playToneSequence_P(melodyMary, false);
          break;
        }

      case button5:
        {
          playToneSequence_P(melodyWheels, false);
          break;
        }

      case button6:
        {
          playToneSequence_P(melodyHappy, false);
          break;
        }

      case button7:
        {
          playToneSequence_P(melodyBabyShark, false);
          break;
        }

      case button8:
        {
          playToneSequence_P(melodyJingle, false);
          break;
        }

      case button9:
        {  // Speak battery voltage
          float vIn = getBatteryVoltageSettledForStatus();
          DBG("Battery Voltage (pattern): ");
          DBGLN(vIn, 1);
          playVoltagePattern(vIn);
          break;
        }
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
  // Initiate battery voltage sound pattern. Then updateBuzzer plays the pattern.
  //    - long beeps for integer volts
  //     - short beeps for tenths
  // ------------------------------------------------------------------------------------------------
  void playVoltagePattern(float vIn) {
    if (SoundOnOff != 1) return;
    digitalWrite(pinBuzzer, LOW);
    clearBuzzerPattern();

    // Round once so volts and tenths stay consistent (e.g. 7.96 V -> 8.0, not 7.0).
    int totalTenths = (int)round(vIn * 10);
    int volts = totalTenths / 10;
    int tenths = totalTenths % 10;

    int cap = BUZZER_PATTERN_MAX - 1;
    int idx = 0;
    for (int i = 0; i < volts && idx < cap; ++i) {                 // long beeps
      if (idx < cap) buzzerPattern[idx++] = 400;                   // ON
      if (i < volts - 1 && idx < cap) buzzerPattern[idx++] = 200;  // OFF between longs
    }
    if (idx < cap && volts > 0) buzzerPattern[idx++] = 600;         // separator OFF
    for (int i = 0; i < tenths && idx < cap; ++i) {                 // short beeps
      if (idx < cap) buzzerPattern[idx++] = 150;                    // ON
      if (i < tenths - 1 && idx < cap) buzzerPattern[idx++] = 150;  // OFF between shorts
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

  // Blocking blink helper used only for short user-feedback flashes.
  void blinkLED(int count, int onDelay, int offDelay) {
    for (int i = 0; i < count; i++) {
      SetGreenLightValue(255);
      delay(onDelay);
      SetGreenLightValue(0);
      delay(offDelay);
    }
  }

  void GreenLEDBlink() {
    blinkLED(2, 100, 50);
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

    DBGLN(F("Direct train LED pins ready"));
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
        DBGLN(F("TILT: ACTIVE -> emergency stop"));
        if (!tiltStopLatched) {
          Stop();
          UltrasonicOnOff = 0;  // optional: exit AUTO
          currentStep = 0;      // optional: reset manual steps
          tiltStopLatched = true;
        }
        SetRGBColor(RgbColor::Red);
        playPattern(pattern_tiltBeep);
        noTone(pinBuzzer);  // stop siren/music
      } else {
        DBGLN(F("TILT: IDLE -> clear latch, restore LEDs"));
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
    for (int j = 0; j < 20; j++) buzzerPattern[j] = 0;
    buzzerIndex = 0;

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