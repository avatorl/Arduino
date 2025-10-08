// ================================================================================================
// Train compatible with Lego DUPLO (3D-printed parts + Arduino)
// Based on: https://cults3d.com/en/3d-model/game/locomotora-sofia-controlada-por-infrarojos-con-control-de-velocidad-por-ultrasonidos-y-motor-superior-multiusos-lego-duplo
// Arduino code fully rewritten & extended by Andrzej Leszkiewicz
// Get the most recent version of the code at https://github.com/avatorl/Arduino/blob/main/duplo-train/duplo-train.ino
// 3D-printing profile: https://makerworld.com/en/models/1854728-arduino-train-locomotive-remote-controlled#profileId-1983131
// ================================================================================================

// 💡 Features ====================================================================================
// 3 speed levels: effective voltage on the motor: 3.5V, 4.5V, 6.0V; measured voltage (unloaded train) - around 4.4V, 5.0V, 6.0V volts
// Stop button: stops, red lights
// Speed up button: speed level +1, moves forward; white lights
// Speed down button: speed level -1, moves forward until speed level = 0; white lights
// Move backward button: moves backward at speed level 1 while the button is pressed; blue lights
// Move forward button: moves forward at speed level 1 while the button is pressed; white lights
// Auto button: moves forward with obstacle detection enabled. Stops if there is an obstacle.
//    Moves forward if the obstacle is removed. Speed depends on the distance to the nearest obstacle. White lights and green light.
// Horn button: horn sound effect
// Siren button: red and blue lights, siren sounds
// Mute button: sound off/on
// Battery status button: indicates battery level by sound beeps, e.g. 7 long beeps and 3 short beeps = 7.3V
// Battery status detection: warning level with red lights and sound; shutdown level
// Sleep mode: powers down automatically after 5 minutes without IR remote input (can be woken up again with the remote)
// TBD: motor overcurrent protection (shunt resistor)
// TBD v.2: tilt sensor

// ================================================================================================
// Electronic components
//   Arduino Nano 3.0 ATMEGA328 CH340
//   IR sensor HX1838 with wiring adapter
//   Active buzzer
//   2 x RGB LED , 1 x green LED
//   DC motor with 1:48 gear, motor driver HG7881 L9110S
//   Ultrasonic distance sensor HC-SR04
//
// Resistors:
//   LED RGB R: 100 Ω + 47 Ω, G: 100 Ω, B: 100 Ω
//   LED Green: 100 Ω + 47 Ω
//   Voltage divider: 10K Ω and 4.7K Ω
// ===============================================================================================

#define DEBUG 1
#if DEBUG
  #define DBG(...)   Serial.print(__VA_ARGS__)
  #define DBGLN(...) Serial.println(__VA_ARGS__)
  #define DBGBEGIN(...) do { Serial.begin(__VA_ARGS__); } while(0)
#else
  #define DBG(...)
  #define DBGLN(...)
  #define DBGBEGIN(...)
#endif

#ifndef STR_HELPER
  #define STR_HELPER(x) #x
#endif
#ifndef STR
  #define STR(x) STR_HELPER(x)
#endif

#define IR_USE_AVR_TIMER1 // move IRremote to Timer1
#include <IRremote.hpp>   // for IR sensor
#include <LowPower.h>     // for sleep mode
#include <math.h>

void SetRGBLightColor(const char* colorName, int led = 0);
void SetRGBColor(const char* colorName, int led = 0);

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
const int button100plus = 25;   // Horn
const int button200plus = 13;   // Siren
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

// ================================================================================================
// Other constants
// ================================================================================================
const unsigned long DIR_DELAY = 1000;  // delay before motor direction change, ms
const float R1 = 10000.0;              // Top resistor (to battery +)
const float R2 = 4700.0;               // Bottom resistor (to GND)
const int AUTO_SAMPLES_FOR_MEDIAN = 5;                 // number of samples for median filter
const int AUTO_DISTANCE_STOP = 8;            // distance to obstacle <= cm to stop the train
const int AUTO_DISTANCE_RESTART = 11;          // distance to obstacle >= cm to re-start the train
const int AUTO_DISTANCE_MAX_SPEED = 50;       // distance to obstacle >= cm to run at max speed
const float BATTERY_LOW_WARNING = 7.4;      // warn at this voltage
const float BATTERY_LOW_SAVER = 7.2;      // disable LEDs and sounds
const float BATTERY_LOW_SHUTDOWN = 7.0;      // force stop at this voltage
const float MAX_SAFE_MOTOR_VOLTAGE = 6.0;

const float BATTERY_HYST_V      = 0.10; // Hysteresis: voltage must rise this much to exit a level
// Debounce / hold times
const unsigned long BATTERY_LOW_WARNING_MS     = 3000;
const unsigned long BATTERY_LOW_SAVER_MS    = 1500;
const unsigned long BATTERY_LOW_SHUTDOWN_MS      = 1000;

// Battery manager globals — define ONCE
enum BatState { BAT_OK, BAT_WARN, BAT_SAVER, BAT_OFF };
BatState batState = BAT_OK;
BatState lastBatState = BAT_OK;   // track transitions for warning beeps
bool saverMode = false;
unsigned long tWarn = 0, tSaver = 0, tOff = 0;
unsigned long warnBeepTimer = 0;   // for periodic beep in BAT_WARN

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
uint16_t lastIRCommand = 0;
bool lastWasRepeat = false;

// Timers
unsigned long lastActive = 0;
const unsigned long idleTimeout = 5UL * 60UL * 1000UL;  // 5 minutes

// Speed steps (voltage → PWM)
int pwmSteps[4];  // 0..3
float voltageSteps[] = { 0.0, 3.5, 4.5, 6.0 };
float batteryVoltage = 0.0;

// Buzzer player
int buzzerPattern[20];
int buzzerIndex = 0;
unsigned long buzzerTimer = 0;

// add near siren globals
unsigned long sirenStepTimer = 0;
const uint16_t sirenStepEveryMs = 10;   // change pitch every 10 ms

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
int sirenPhase = 0;     // 0=LED1 red, 1=LED1 blue
int sirenFreq = 400;
int sirenDirection = 1; // 1 = up, -1 = down

// Siren timing (time-based sweep → stable even with loop jitter)
const int  sirenFmin = 400;
const int  sirenFmax = 800;
const unsigned long sirenSweepMs = 800;       // up in 800 ms, down in 800 ms
unsigned long sirenStartMs = 0;               // set when siren toggles ON

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
  pinMode(pinUltrasonicEcho, INPUT);
  pinMode(pinIRReceiver, INPUT);
  pinMode(pinBatterySense, INPUT);

  playPattern(pattern_melody);  // Play melody
  SetGreenLightValue(0);
  SetRGBColor(FrontLightOnOff ? "red" : "off");

  // Measure battery at startup
  batteryVoltage = getBatteryVoltageDirect();
  DBG(F("Battery measured: "));
  DBGLN(batteryVoltage, 2);

  // Configure dynamic speed steps
  configureSpeedSteps();

  lastActive = millis();            // seed idle timer
  IrReceiver.begin(pinIRReceiver);  // Start IR Receiver
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

  updateBatteryManager();

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

  // === 5. IR remote handler ===
  translateIR();

  // Non-blocking playback ===
  updateBuzzer();
  updateSiren();

  delay(10); // small loop delay

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
  else setMotor("forward", Speed);  // auto always forward
}

int Distancia_test() {
  // Trigger 10 µs pulse
  digitalWrite(pinUltrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinUltrasonicTrig, LOW);

  // Timeout derived from max distance (≈58 µs per cm) with 1.25x margin
  const unsigned long echoTimeoutUs =
      (unsigned long)(AUTO_DISTANCE_MAX_SPEED * 58UL * 5 / 4);

  // Read echo with timeout
  unsigned long duration = pulseIn(pinUltrasonicEcho, HIGH, echoTimeoutUs);

  // No echo → treat as "far enough"
  if (duration == 0) return AUTO_DISTANCE_MAX_SPEED;

  // Convert to cm and clamp
  int cm = (int)(duration / 58UL);
  if (cm > AUTO_DISTANCE_MAX_SPEED) cm = AUTO_DISTANCE_MAX_SPEED;
  return cm;
}


int getMedianDistance() {
  
  static unsigned long lastSample = 0;
  unsigned long now = millis();
  if (now - lastSample < 50) return Distance; // reuse last value if called too soon
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
uint16_t irReceive() {
  lastWasRepeat = false;
  uint16_t received{ 0 };

  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == NEC) {
      if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
        lastWasRepeat = true;
        received = lastIRCommand;  // reuse last command on repeat
      } else {
        received = IrReceiver.decodedIRData.command;
        lastIRCommand = received;
        DBGLN(received, DEC);
      }
    }
    IrReceiver.resume();
  }
  return received;  // 0 means "no new code this loop"
}

// ================================================================================================
// Motor Control
// ================================================================================================
void setMotor(const char* direction, int speed) {
  int safeSpeed = constrain(speed, 0, 255);

  if (strcmp(direction, "forward") == 0) {
    analogWrite(pinEngineA_1A, safeSpeed);
    analogWrite(pinEngineA_1B, 0);
    MotorDirection = 1;
    DBG(F("Driving Forward >>> Speed="));
    DBGLN(safeSpeed);
  } else if (strcmp(direction, "backward") == 0) {
    analogWrite(pinEngineA_1A, 0);
    analogWrite(pinEngineA_1B, safeSpeed);
    MotorDirection = 2;
    DBG(F("Driving Backward >> Speed="));
    DBGLN(safeSpeed);
  } else {  // stop
    analogWrite(pinEngineA_1A, 0);
    analogWrite(pinEngineA_1B, 0);
    Speed = 0;
    DBGLN(F("Motor OFF"));
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
    setMotor("forward", Speed);
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
    setMotor("backward", Speed);
    DBGLN(F("Driving Backward >>"));
  }
}

void Stop() {
  setMotor("stop", 0);
}

// Jog motor (doesn’t affect MotorDirection or Speed state)
void JogDrive(const char* direction) {
  int jogPWM = pwmSteps[1]; // speed level 1
  if (strcmp(direction, "forward") == 0) {
    analogWrite(pinEngineA_1A, jogPWM);
    analogWrite(pinEngineA_1B, 0);
  } else if (strcmp(direction, "backward") == 0) {
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
  uint16_t code = irReceive();

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
          JogDrive("backward");
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
          JogDrive("forward");
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

    case buttonEQ: {  // Mute / Unmute
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
        playPattern(pattern_double); // short confirmation chirp
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
        SetRGB1Light(false, false, false);
        SetRGB2Light(false, false, false);
      } else {
        sirenTimer   = millis();      // for LED swap
        sirenStartMs = sirenTimer;    // for deterministic audio sweep
        DBGLN(SoundOnOff ? F("Siren ON (with sound)") : F("Siren ON (lights only, muted)"));
      }
      break;

  
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
void updateBuzzer() {
  if (sirenActive) return;     // siren owns the buzzer
  if (buzzerPattern[buzzerIndex] == 0) return;

  unsigned long now = millis();
  if (now - buzzerTimer >= (unsigned long)buzzerPattern[buzzerIndex]) {
    buzzerIndex++;
    buzzerTimer = now;

    if (buzzerPattern[buzzerIndex] == 0) {
      digitalWrite(pinBuzzer, LOW);
      buzzerIndex = 0;
      buzzerPattern[0] = 0;
      return;
    }

    if (buzzerIndex % 2 == 0) digitalWrite(pinBuzzer, HIGH);
    else digitalWrite(pinBuzzer, LOW);
  }
}

// ------------------------------------------------------------------------------------------------
// Initiate one of the predefined sound patterns. Then updateBuzzer plays the pattern.
// ------------------------------------------------------------------------------------------------
void playPattern(const int* pattern) {
  if (SoundOnOff != 1) return;
  if (saverMode) return;

  digitalWrite(pinBuzzer, LOW);  // ensure buzzer off first

  for (int j = 0; j < 10; j++) buzzerPattern[j] = 0;

  int i = 0;
  while (i < 9) {
    buzzerPattern[i] = pattern[i];
    if (pattern[i] == 0) break;
    i++;
  }
  buzzerPattern[i] = 0;

  buzzerIndex = 0;
  buzzerTimer = millis();
  if (buzzerPattern[0] > 0) digitalWrite(pinBuzzer, HIGH);
}

// ------------------------------------------------------------------------------------------------
// Initiate battery voltage sound pattern. Then updateBuzzer plays the pattern.
//    - long beeps for integer volts
//     - short beeps for tenths
// ------------------------------------------------------------------------------------------------
void playVoltagePattern(float vIn) {
  if (SoundOnOff != 1) return;   // respect mute

  digitalWrite(pinBuzzer, LOW);  // ensure buzzer off first

  int volts = (int)floor(vIn);
  int tenths = (int)round(vIn * 10) % 10;

  int idx = 0;
  // Long beeps for integer volts
  for (int i = 0; i < volts; i++) {
    buzzerPattern[idx++] = 400; // ON
    if (i < volts - 1) {
      buzzerPattern[idx++] = 200; // OFF only between longs
    }
  }

  // Separator pause (clear gap before tenths)
  buzzerPattern[idx++] = 600; // OFF

  // Short beeps for tenths
  for (int i = 0; i < tenths; i++) {
    buzzerPattern[idx++] = 150; // ON
    if (i < tenths - 1) {
      buzzerPattern[idx++] = 150; // OFF only between shorts
    }
  }

  // End marker
  buzzerPattern[idx] = 0;
  buzzerIndex = 0;
  buzzerTimer = millis();
  if (buzzerPattern[0] > 0) digitalWrite(pinBuzzer, HIGH);
}

// ================================================================================================
// LEDs
// ================================================================================================

void SetRGB1Light(bool R, bool G, bool B) {
  if (FrontLightOnOff == 0) return;
  digitalWrite(pinLED1_R, R ? LOW : HIGH);
  digitalWrite(pinLED1_G, G ? LOW : HIGH);
  digitalWrite(pinLED1_B, B ? LOW : HIGH);
}

void SetRGB2Light(bool R, bool G, bool B) {
  if (FrontLightOnOff == 0) return;
  digitalWrite(pinLED2_R, R ? LOW : HIGH);
  digitalWrite(pinLED2_G, G ? LOW : HIGH);
  digitalWrite(pinLED2_B, B ? LOW : HIGH);
}

void SetRGBLight(bool R, bool G, bool B, int led) {
  if (led == 0 || led == 1) SetRGB1Light(R, G, B);
  if (led == 0 || led == 2) SetRGB2Light(R, G, B);
}

void SetRGBLightColor(const char* colorName, int led) {
  if (strcmp(colorName, "red") == 0) SetRGBLight(true, false, false, led);
  else if (strcmp(colorName, "green") == 0) SetRGBLight(false, true, false, led);
  else if (strcmp(colorName, "blue") == 0) SetRGBLight(false, false, true, led);
  else if (strcmp(colorName, "yellow") == 0) SetRGBLight(true, true, false, led);
  else if (strcmp(colorName, "cyan") == 0) SetRGBLight(false, true, true, led);
  else if (strcmp(colorName, "magenta") == 0) SetRGBLight(true, false, true, led);
  else if (strcmp(colorName, "white") == 0) SetRGBLight(true, true, true, led);
  else if (strcmp(colorName, "off") == 0) SetRGBLight(false, false, false, led);
  else SetRGBLight(false, false, false, led);
}

void SetGreenLightValue(int Value) {
  // analogWrite(pinLEDGreen, Value);
  digitalWrite(pinLEDGreen, (Value > 0) ? HIGH : LOW); // Any non-zero means ON
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

// Put near your LED helpers:
void SetRGBColor(const char* colorName, int led) {
  if (sirenActive) return;   // ignore LED color changes during siren
  SetRGBLightColor(colorName, led);
}

void updateSiren() {
  if (!sirenActive) return;
  if (saverMode) { noTone(pinBuzzer); return; }

  unsigned long now = millis();

  // LED swap every 300 ms (always runs, even when muted)
  if (now - sirenTimer > 300) {
    sirenPhase = !sirenPhase;
    if (sirenPhase) {
      SetRGB1Light(true,  false, false); // LED1 red
      SetRGB2Light(false, false, true);  // LED2 blue
    } else {
      SetRGB1Light(false, false, true);  // LED1 blue
      SetRGB2Light(true,  false, false); // LED2 red
    }
    sirenTimer = now;
  }

  // ---- Deterministic triangle sweep for pitch (immune to loop jitter) ----
  // Total cycle = up + down
  const unsigned long cycleMs = 2UL * sirenSweepMs;
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
  else                 noTone(pinBuzzer);
}

void updateBatteryManager() {
  float v = getBatteryVoltageDirect();

  // --- Descend with debounce (under-voltage path) ---
  if (v < BATTERY_LOW_SHUTDOWN) {
    if (!tOff) tOff = millis();
    if (millis() - tOff >= BATTERY_LOW_SHUTDOWN_MS) batState = BAT_OFF;
  } else {
    tOff = 0;
    if (v < BATTERY_LOW_SAVER) {
      if (!tSaver) tSaver = millis();
      if (millis() - tSaver >= BATTERY_LOW_SAVER_MS) batState = BAT_SAVER;
    } else {
      tSaver = 0;
      if (v < BATTERY_LOW_WARNING) {
        if (!tWarn) tWarn = millis();
        if (millis() - tWarn >= BATTERY_LOW_WARNING_MS) batState = BAT_WARN;
      } else {
        tWarn = 0;

        // --- Recover upward with hysteresis (rise in voltage) ---
        if (batState == BAT_WARN  && v > (BATTERY_LOW_WARNING  + BATTERY_HYST_V)) batState = BAT_OK;
        if (batState == BAT_SAVER && v > (BATTERY_LOW_SAVER    + BATTERY_HYST_V)) batState = BAT_WARN;   // step up one level
        if (batState == BAT_OFF   && v > (BATTERY_LOW_SHUTDOWN + BATTERY_HYST_V)) batState = BAT_SAVER;  // step up one level
      }
    }
  }

  // --- On state change: one-time actions / beep ---
  if (batState != lastBatState) {
    if (batState == BAT_WARN && SoundOnOff == 1 && !saverMode) {
      playPattern(pattern_batteryWarn);  // immediate beep on entering WARN
    }
    warnBeepTimer = millis();  // reset periodic timer on any transition
    lastBatState = batState;
  }

  // --- Enforce behaviors for each state ---
  switch (batState) {
    case BAT_OK:
      saverMode = false;
      break;

    case BAT_WARN:
      // Periodic reminder (every 60s) while in WARN
      if (!saverMode && SoundOnOff == 1 && (millis() - warnBeepTimer) >= 60000UL) {
        playPattern(pattern_batteryWarn);
        warnBeepTimer = millis();
      }
      break;

    case BAT_SAVER:
      // Enter power-saver: mute extras and cap speed
      saverMode = true;
      SetRGBColor("off");
      SetGreenLightValue(0);            // kill green status LED
      if (currentStep > 2) {            // cap manual to step 2
        currentStep = 2;
        applySpeedStep();
      }
      break;

    case BAT_OFF:
      // Hard stop & deep sleep
      Stop();
      SetRGBColor("off");
      SetGreenLightValue(0);
      while (true) {
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
      }
  }
}
