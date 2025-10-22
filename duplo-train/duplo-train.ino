  // Library includes
  #define IR_USE_AVR_TIMER1 // Configure IRremote to use Timer1 on AVR
  #include <IRremote.hpp>  // IR sensor library
  #include <LowPower.h>    // Low power/sleep mode library
  #include <math.h>        // Math functions

  #include "src/melodies.h" // Load music patterns

  // ===============================================================================================
  // DEBUG flag: set to 1 to enable serial output for debugging, set to 0 to disable for production
  #define DEBUG 1

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

  // Stringify macros
  #ifndef STR_HELPER
  #define STR_HELPER(x) #x  // Converts token to string literal
  #endif
  #ifndef STR
  #define STR(x) STR_HELPER(x)  // Ensures macro expansion before stringification
  #endif

  // AVR-specific include for program memory storage
  #if defined(__AVR__)
  #include <avr/pgmspace.h>
  #endif

  void SetRGBLightColor(const char* colorName, int led = 0);
  void SetRGBColor(const char* colorName, int led = 0);

  void playToneSequenceRaw(const int* seqFD, int pairCount, bool loopPlayback, bool isProgmem = false);

  // PROGMEM wrapper (for PROGMEM melodies)
  template<size_t N>
  void playToneSequence_P(const int16_t (&seqFD)[N], bool loopPlayback = false) {
    static_assert(N % 2 == 0, "Melody array must have even length [freq,dur,...]");
    playToneSequenceRaw((const int*)seqFD, (int)(N / 2), loopPlayback, true);
  }

  void updateMelody();
  void stopMelody();

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

  // Button assignments
  const int buttonCHminus = 69;    // Speed -
  const int buttonCH = 70;         // Stop
  const int buttonCHplus = 71;     // Speed +
  const int buttonBackward = 68;   // Momentary backward
  const int buttonForward = 64;    // Momentary forward
  const int buttonPlayPause = 67;  // Auto-speed toggle, start/stop
  const int buttonEQ = 9;          // Mute / Unmute
  const int button0 = 22;
  const int button100plus = 25;  // Horn
  const int button200plus = 13;  // Siren
  const int button1 = 12;
  const int button2 = 24;
  const int button3 = 94;
  const int button4 = 8;
  const int button5 = 28;
  const int button6 = 90;
  const int button7 = 66;
  const int button8 = 82;
  const int button9 = 74;  // Battery Test

  // ================================================================================================
  // Arduino Pin Mapping
  // ================================================================================================
  const int pinBatterySense = A0;                          // Battery voltage monitoring
  const int pinTiltSensor = A2;                            // Tilt sensor
  const int pinUltrasonicTrig = A3;                        // Ultrasonic sensor trigger
  const int pinUltrasonicEcho = A4;                        // Ultrasonic sensor echo
  const int pinIRReceiver = 2;                             // IR receiver input: D2 or D3 required to wake from sleep
  const int pinEngineA_1A = 5;                             // Motor direction and speed (PWM)
  const int pinEngineA_1B = 6;                             // Motor direction and speed (PWM)
  const int pinLED1_R = 11, pinLED1_G = 3, pinLED1_B = 4;  // RGB LED #1 (ON/OFF only, no PWM required)
  const int pinLED2_R = 7, pinLED2_G = 8, pinLED2_B = 9;   // RGB LED #2 (ON/OFF only, no PWM required)
  const int pinLEDGreen = 10;                              // Green LED (no PWM required)
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

  #define PAIRS_OF(a) (int)(sizeof(a) / (2 * sizeof((a)[0])))

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
  int MotorDirection = 1;   // always has a direction
  int Speed = 0;            // stopped at start
  int Distance = 0;         // Latest distance from ultrasonic

  // --- Distance median filter state ---
  int distanceBuffer[AUTO_SAMPLES_FOR_MEDIAN];
  int bufferIndex = 0;
  bool bufferFilled = false;

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

  // Speed steps (voltage → PWM)
  int pwmSteps[4];  // 0..3
  float voltageSteps[] = { 0.0, 3.5, 4.5, 6.0 };
  float batteryVoltage = 0.0;

  // Buzzer
  #define BUZZER_PATTERN_MAX 20
  int buzzerPattern[BUZZER_PATTERN_MAX];
  int buzzerIndex = 0;
  unsigned long buzzerTimer = 0;

  // add near siren globals
  unsigned long sirenStepTimer = 0;
  const uint16_t sirenStepEveryMs = 10;  // change pitch every 10 ms

  // Manual step index
  int currentStep = 0;  // 0=stop, 1=~3.5V, 2=~4.5V, 3=~6V

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
  int sirenFreq = 400;
  int sirenDirection = 1;  // 1 = up, -1 = down

  // Siren timing (time-based sweep → stable even with loop jitter)
  const int sirenFmin = 400;
  const int sirenFmax = 800;
  const unsigned long sirenSweepMs = 800;  // up in 800 ms, down in 800 ms
  unsigned long sirenStartMs = 0;          // set when siren toggles ON

  // ── Tilt sensor debounce config/state ─────────────────────────────────────────
  const bool TILT_ACTIVE_LOW = true;         // tilt at LOW value from sensor
  const unsigned long TILT_STABLE_MS = 1000;  // must hold this long to confirm state
  const unsigned long TILT_QUIET_MS = 500;   // ignore flips for a short time after change
  int tiltStableState = HIGH;                // using INPUT_PULLUP: OPEN=HIGH (idle)
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
  void setup() {

    DBGBEGIN(9600);

    // Initialize Arduino pins
    pinMode(pinLEDGreen, OUTPUT);
    pinMode(pinLED1_R, OUTPUT);
    pinMode(pinLED1_G, OUTPUT);
    pinMode(pinLED1_B, OUTPUT);
    pinMode(pinLED2_R, OUTPUT);
    pinMode(pinLED2_G, OUTPUT);
    pinMode(pinLED2_B, OUTPUT);
    pinMode(pinBuzzer, OUTPUT);
    pinMode(pinEngineA_1A, OUTPUT);
    pinMode(pinEngineA_1B, OUTPUT);
    pinMode(pinUltrasonicTrig, OUTPUT);
    digitalWrite(pinUltrasonicTrig, LOW);
    pinMode(pinUltrasonicEcho, INPUT);
    //pinMode(pinIRReceiver, INPUT);
    pinMode(pinBatterySense, INPUT);
    pinMode(pinTiltSensor, INPUT);
    digitalWrite(pinTiltSensor, LOW);  // ensure internal pull-up is OFF

    //playPattern(pattern_melody);  // Play melody
    SetGreenLightValue(0);
    SetRGBColor(FrontLightOnOff ? "red" : "off");
    playToneSequence_P(melodyDemo, false);

    // Measure battery at startup
    batteryVoltage = getBatteryVoltageDirect();
    DBG(F("Battery measured: "));
    DBGLN(batteryVoltage, 2);

    // Configure dynamic speed steps
    configureSpeedSteps();

    lastActive = millis();            // seed idle timer
    IrReceiver.begin(pinIRReceiver, ENABLE_LED_FEEDBACK);  // Start IR Receiver
  }

  // ================================================================================================
  // Main Loop
  // ================================================================================================
  void loop() {

    // // --- Overcurrent protection with persistence ---
    //   float motorCurrent = getMotorCurrent();
    //   if (motorCurrent > maxSafeCurrent) {
    //     if (!overCurrentActive) {
    //       overCurrentActive = true;
    //       overCurrentStart = millis();
    //     } else {
    //       if (millis() - overCurrentStart > 200) { // >200 ms continuous
    //         DBG(F("⚠️ Motor Overcurrent: "));
    //         DBG(motorCurrent, 2);
    //         DBGLN(F(" A → STOPPING!"));

    //         Stop();
    //         SetRGBColor("red");
    //         playPattern(pattern_descend); // warning sound
    //         delay(500);

    //         overCurrentActive = false; // reset
    //       }
    //     }
    //   } else {
    //     overCurrentActive = false; // reset if current goes back to normal
    //   }

    // // === 1. Battery monitor first (critical safety) ===
    // float vNow = getBatteryVoltageDirect();
    // if (vNow < BATTERY_LOW_SHUTDOWN) {

    //   DBGLN(F("Battery critically low → stopping train"));

    //   Stop();
    //   SetRGBColor("off");
    //   SetGreenLightValue(0);
    //   digitalWrite(pinBuzzer, LOW); // end beep
    //   buzzerPattern[0] = 0;
    //   buzzerIndex = 0;
    //   LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // } else if (vNow < BATTERY_LOW_WARNING) {
    //   static unsigned long lastWarn = 0;
    //   if (millis() - lastWarn > 60000) {  // warn max once per minute
    //     DBGLN(F("Battery low warning"));
    //     playPattern(pattern_batteryWarn);
    //     lastWarn = millis();
    //   }
    // }

    // === 2. Idle timeout watchdog ===
    if (millis() - lastActive > idleTimeout) {
      goToIdle();
    }

    // === 3. Jog watchdog (safety if button released) ===
    if (momentaryActive && (millis() - momentaryLastSeen > momentaryTimeout)) {
      DBGLN(F("Jog watchdog timeout → STOP"));
      Stop();
      SetRGBColor("red");
      momentaryActive = false;
    }

    // === 4. Auto-speed mode ===
    if (UltrasonicOnOff == 1) {
      SpeedAutoUltrasonic();
    }

    // === 6. Tilt sensor ===
    updateTiltSensor();

    // === 5. IR remote handler ===
    translateIR();

    // Non-blocking playback ===
    updateBuzzer();
    updateSiren();
    updateMelody();

    delay(10);  // small loop delay
  }

  // ================================================================================================
  // Idle / Sleep
  // ================================================================================================
  void goToIdle() {
    DBGLN(F("Idle: turning everything off..."));

    digitalWrite(pinBuzzer, LOW);
    buzzerPattern[0] = 0;
    buzzerIndex = 0;

    SetRGBColor("off");
    SetGreenLightValue(0);

    playPattern(pattern_descend);
    delay(2000);
    digitalWrite(pinBuzzer, LOW);

    DBGLN(F("Entering sleep mode..."));

    attachInterrupt(digitalPinToInterrupt(pinIRReceiver), wakeUp, CHANGE);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(digitalPinToInterrupt(pinIRReceiver));

    lastActive = millis();
  }

  void wakeUp() {}

  // Measure battery voltage directly
  float getBatteryVoltageDirect() {
    int raw = analogRead(pinBatterySense);
    float vOut = raw * 5.0 / 1023.0;
    return vOut * (R1 + R2) / R2;
  }

  // Convert desired motor voltage into PWM duty cycle
  int safePWMFromVoltage(float desiredMotorV) {
    batteryVoltage = getBatteryVoltageDirect();
    if (batteryVoltage <= 0) return 0;
    // Clamp to max 6 V effective
    float vm = min(desiredMotorV, MAX_SAFE_MOTOR_VOLTAGE);
    int pwm = (int)(255.0 * vm / batteryVoltage);
    return constrain(pwm, 0, 255);
  }

  // float getMotorCurrent() {
  //   int raw = analogRead(pinMotorSense);
  //   float vSense = (raw * 5.0) / 1023.0;     // voltage across shunt
  //   return vSense / shuntResistor;           // I = V / R
  //   DBG(F("Motor current = "));
  //   DBG(iNow, 2);
  //   DBGLN(F(" A"));
  // }

  // ================================================================================================
  // Manual speed (steps)
  // ================================================================================================
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

  void SpeedAutoUltrasonic() {
    Distance = getMedianDistance();

    DBG(F("Ultrasonic Distance: "));
    DBG(Distance);
    DBGLN(F(" cm"));

    float targetV = motorVoltageFromDistance(Distance);
    int targetSpeed = safePWMFromVoltage(targetV);

    if (targetSpeed == 0) {
      SetRGBColor("red");
      DBGLN(F("Auto: STOP"));
    } else {
      SetRGBColor("white");
      DBG(F("Auto target speed = "));
      DBG(targetSpeed);
      DBG(" (≈ ");
      DBG(targetV, 2);
      DBGLN(" V)");
    }

    updateMotorSpeed(targetSpeed);
  }

  const int rampStep = 5;
  unsigned long rampDelay = 80;
  static unsigned long lastRamp = 0;

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

  int getMedianDistance() {

    static unsigned long lastSample = 0;
    unsigned long now = millis();
    if (now - lastSample < 50) return Distance;  // reuse last value if called too soon
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
  // Motor Control
  // ================================================================================================
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
      delay(DIR_DELAY);
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
      delay(DIR_DELAY);
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

  // Jog motor (doesn’t affect MotorDirection or Speed state)
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
  void translateIR() {
    uint8_t code = irReceive();

    // Cancel jog if a different non-repeat key appears
    if (momentaryActive && code != 0 && !lastWasRepeat && code != momentaryButton) {
      DBGLN(F("Cancelling jog due to new key"));
      Stop();
      SetRGBColor("red");
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
        SetRGBColor("red");
        momentaryActive = false;
      }
      return;
    }

    lastActive = millis();

    // Blink green LED once for any valid button press
    GreenLEDBlink();

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
              SetRGBColor("red");
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
          SetRGBColor("red");
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
              SetRGBColor("white");
            }
            if (MotorDirection == 2) {
              GoBackward();
              SetRGBColor("blue");
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
            SetRGBColor("blue");
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
            SetRGBColor("white");
            momentaryActive = true;
            momentaryButton = buttonForward;
            momentaryLastSeen = millis();
            DBGLN(F("Momentary FORWARD running (hold to move)"));
          } else {
            DBGLN(F("Ignored: >> only when stationary & not in AUTO"));
          }
          break;
        }

      case buttonPlayPause:
        {  // Auto-speed toggle
          UltrasonicOnOff = !UltrasonicOnOff;
          digitalWrite(pinLEDGreen, UltrasonicOnOff ? HIGH : LOW);
          if (UltrasonicOnOff) {
            DBGLN(F("Driving started (auto-speed)"));
            SetRGBColor("white");
            GoForward();
            playPattern(pattern_double);
          } else {
            DBGLN(F("Driving stopped"));
            SetRGBColor("red");
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
          SetRGBColor("off");
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
          float vIn = getBatteryVoltageDirect();
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
  inline void clearBuzzerPattern() {
    for (int j = 0; j < BUZZER_PATTERN_MAX; ++j) buzzerPattern[j] = 0;
    buzzerIndex = 0;
  }

  void updateBuzzer() {
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

    int volts = (int)floor(vIn);
    int tenths = (int)round(vIn * 10) % 10;

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

  inline void writeRGBPins(int rPin, int gPin, int bPin, bool R, bool G, bool B) {
    digitalWrite(rPin, R ? LOW : HIGH);
    digitalWrite(gPin, G ? LOW : HIGH);
    digitalWrite(bPin, B ? LOW : HIGH);
  }

  void SetRGBLight(bool R, bool G, bool B, int led) {
    if (FrontLightOnOff == 0) return;
    if (led == 0 || led == 1) writeRGBPins(pinLED1_R, pinLED1_G, pinLED1_B, R, G, B);
    if (led == 0 || led == 2) writeRGBPins(pinLED2_R, pinLED2_G, pinLED2_B, R, G, B);
  }

  struct Color {
    const char* name;
    bool r, g, b;
  };

  // Simple RAM LUT (no PROGMEM name pointers)
  static const Color COLORS[] = {
    { "red", 1, 0, 0 }, { "green", 0, 1, 0 }, { "blue", 0, 0, 1 }, { "yellow", 1, 1, 0 }, { "cyan", 0, 1, 1 }, { "magenta", 1, 0, 1 }, { "white", 1, 1, 1 }, { "off", 0, 0, 0 }
  };

  void SetRGBLightColor(const char* colorName, int led) {
    // lookup by full name only (no single-letter aliases)
    bool R = false, G = false, B = false;
    for (uint8_t i = 0; i < sizeof(COLORS) / sizeof(COLORS[0]); ++i) {
      if (strcmp(colorName, COLORS[i].name) == 0) {
        R = COLORS[i].r;
        G = COLORS[i].g;
        B = COLORS[i].b;
        break;
      }
    }
    SetRGBLight(R, G, B, led);
  }

  inline void SetRGBColor(const char* colorName, int led) {
    if (sirenActive) return;  // ignore during siren
    SetRGBLightColor(colorName, led);
  }

  void SetGreenLightValue(int Value) {
    // analogWrite(pinLEDGreen, Value);
    digitalWrite(pinLEDGreen, (Value > 0) ? HIGH : LOW);  // Any non-zero means ON
  }

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
  // Tilt sensor (SW-520D / SW-200D family) – debounced, RGB + 0.5s beep on tilt
  // Uses TILT_ACTIVE_LOW to support either pull-up (active LOW) or pull-down (active HIGH).
  // Note: tilt doesn't disable the siren
  // ================================================================================================
  void updateTiltSensor() {
    unsigned long now = millis();
    int reading = digitalRead(pinTiltSensor);

    auto tiltActive = [](int level) -> bool {
      if (TILT_ACTIVE_LOW) return (level == LOW);
      else return (level == HIGH);
    };

    if (reading != tiltLastRead) {
      tiltLastRead = reading;
      tiltEdgeAt = now;
    }

    if (now >= tiltQuietUntil && reading != tiltStableState && (now - tiltEdgeAt) >= TILT_STABLE_MS) {

      tiltStableState = reading;
      tiltQuietUntil = now + TILT_QUIET_MS;

      if (tiltActive(tiltStableState)) {
        DBGLN(F("TILT: ACTIVE -> emergency stop"));
        if (!tiltStopLatched) {
          Stop();
          UltrasonicOnOff = 0;  // optional: exit AUTO
          currentStep = 0;      // optional: reset manual steps
          tiltStopLatched = true;
        }
        SetRGBColor("red");
        playPattern(pattern_tiltBeep);
        noTone(pinBuzzer);  // stop siren/music
      } else {
        DBGLN(F("TILT: IDLE -> clear latch, restore LEDs"));
        tiltStopLatched = false;
        SetRGBColor("yellow");
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

  // Stops melody playback and resets related state
  void stopMelody() {
    noTone(pinBuzzer);
    melodyPlaying = false;
    melodyLenPairs = 0;
    melodyIdxPair = 0;
    melodyStepStarted = 0;
  }

  // Updates the melody playback based on timing and current step
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

  // Starts playback of a tone sequence from RAM or PROGMEM
  // seqFD: pointer to frequency-duration pairs
  // pairCount: number of (frequency, duration) pairs
  // loopPlayback: whether to loop the melody
  // isProgmem: true if seqFD is stored in PROGMEM (flash), false if in RAM
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