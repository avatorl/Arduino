// ================================================================================================
// Train compatible with Lego DUPLO (3D-printed parts + Arduino)
// Based on: https://cults3d.com/en/3d-model/game/locomotora-sofia-controlada-por-infrarojos-con-control-de-velocidad-por-ultrasonidos-y-motor-superior-multiusos-lego-duplo
// Arduino code fully rewritten & extended by Andrzej Leszkiewicz
// Get the most recent version of the code at https://github.com/avatorl/Arduino/blob/main/duplo-train/duplo-train.ino
// 3D-printing profile: https://makerworld.com/en/models/1854728-arduino-train-locomotive-remote-controlled#profileId-1983131
// ================================================================================================

// 💡 Features ====================================================================================
// Red lights → train stationary
// Blue lights → train moving backward
// White lights → train moving forward
// Green top light → obstacle detection mode enabled
// Horn sound effect
// Battery status feedback (in volts, by sound beeps)
// Sleep mode → powers down automatically after 5 minutes without IR remote input (can be woken up again with the remote)
// TBD v.2: motor over current protection (shunt resistor)
// TBD v.2: tilt sensor

// ================================================================================================
// Electronic components
//   Arduino Nano 3.0 ATMEGA328 CH340
//   IR sensor HX1838 with wiring adapter
//   Active buzzer (with generator)
//   2 x RGB LED , 1 x green LED
//   DC motor, motor driver HG7881 L9110S
//   Ultrasonic distance sensor HC-SR04
//
// Resistors:
//   LED RGB R: 100 Ω + 47 Ω, G: 100 Ω, B: 100 Ω
//   LED Green: 100 Ω + 47 Ω
//   Voltage divider: 10K Ω and 4.7K Ω
// ===============================================================================================

#include <IRremote.hpp>  // for IR sensor
#include <LowPower.h>    // for sleep mode
#include <math.h>

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
const int button100plus = 25;  // Horn
const int button200plus = 13;
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
const int pinLEDGreen = 10;                              // Green LED (PWM required)
const int pinBuzzer = 12;                                // Active buzzer (with generator)

// ================================================================================================
// Buzzer sound patterns
// ================================================================================================
const int pattern_melody[] = { 150, 80, 200, 80, 250, 80, 300, 150, 250, 0 };
const int pattern_batteryWarn[] = { 3000, 100, 0 };
const int pattern_double[] = { 150, 100, 150, 0 };
const int pattern_descend[] = { 120, 80, 120, 80, 120, 0 };
const int pattern_horn[] = { 400, 200, 400, 200, 400, 0 };

// ================================================================================================
// Other constants
// ================================================================================================
const unsigned long DIR_DELAY = 1000;  // delay before motor direction change, ms
const float R1 = 10000.0;              // Top resistor (to battery +)
const float R2 = 4700.0;               // Bottom resistor (to GND)
const int NMedian = 5;                 // number of samples for median filter
const int maxSafeSpeed = 180;          // Motor safety speed limit (battery)
const int distanceStop = 8;            // distance to obstacle <= cm to stop the train
const int distanceStart = 11;          // distance to obstacle >= cm to re-start the train
const int distanceMaxSpeed = 50;       // distance to obstacle >= cm to run at max speed
const float lowBatteryWarn = 6.6;      // warn at this voltage
const float lowBatteryStop = 6.4;      // force stop at this voltage

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
int distanceBuffer[NMedian];
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
const float maxSafeVoltage = 6.0;

// Buzzer player
int buzzerPattern[40];
int buzzerIndex = 0;
unsigned long buzzerTimer = 0;

// Manual step index
int currentStep = 0;  // 0=stop, 1=~3.5V, 2=~4.5V, 3=~6V

// // Current sense
// const int pinMotorSense = A2;   // shunt resistor voltage
// const float shuntResistor = 0.1; // ohms
// const float maxSafeCurrent = 0.8; // amps (cutoff threshold)

// // Overcurrent detection
// unsigned long overCurrentStart = 0;
// bool overCurrentActive = false;

// ================================================================================================
// Setup
// ================================================================================================
void setup() {
  Serial.begin(9600);

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
  Serial.print("Battery measured: ");
  Serial.println(batteryVoltage, 2);

  // Configure dynamic speed steps
  configureSpeedSteps();

  lastActive = millis();            // seed idle timer
  IrReceiver.begin(pinIRReceiver);  // Start IR Receiver
}

// ================================================================================================
// Main Loop
// ================================================================================================
void loop() {
  if (UltrasonicOnOff == 1) {
    SpeedAutoUltrasonic();
  }
  translateIR();
  updateBuzzer();

// // --- Overcurrent protection with persistence ---
//   float motorCurrent = getMotorCurrent();
//   if (motorCurrent > maxSafeCurrent) {
//     if (!overCurrentActive) {
//       overCurrentActive = true;
//       overCurrentStart = millis();
//     } else {
//       if (millis() - overCurrentStart > 200) { // >200 ms continuous
//         Serial.print("⚠️ Motor Overcurrent: ");
//         Serial.print(motorCurrent, 2);
//         Serial.println(" A → STOPPING!");

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

  // === Battery monitor ===
  float vNow = getBatteryVoltageDirect();
  if (vNow < lowBatteryStop) {
    Serial.println("Battery critically low → stopping train");
    Stop();
    SetRGBColor("red");
    playPattern(pattern_descend);  // sad beep
    while (true) {
      // infinite sleep until reset/charging
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    }
  } else if (vNow < lowBatteryWarn) {
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 60000) {  // warn max once per minute
      Serial.println("Battery low warning");
      playPattern(pattern_batteryWarn);
      lastWarn = millis();
    }
  }

  // Jog release watchdog (extra safety)
  if (momentaryActive && (millis() - momentaryLastSeen > momentaryTimeout)) {
    Serial.println("Jog watchdog timeout → STOP");
    Stop();
    SetRGBColor("red");
    momentaryActive = false;
  }

  delay(10);
}

// ================================================================================================
// Idle / Sleep
// ================================================================================================
void goToIdle() {
  Serial.println("Idle: turning everything off...");

  digitalWrite(pinBuzzer, LOW);
  buzzerPattern[0] = 0;
  buzzerIndex = 0;

  SetRGBColor("off");
  SetGreenLightValue(0);

  playPattern(pattern_descend);
  delay(2000);
  digitalWrite(pinBuzzer, LOW);

  Serial.println("Entering sleep mode...");

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
int pwmFromVoltage(float desiredMotorV) {
  batteryVoltage = getBatteryVoltageDirect();
  if (batteryVoltage <= 0) return 0;
  // Clamp to max 6 V effective
  float vm = min(desiredMotorV, maxSafeVoltage);
  int pwm = (int)(255.0 * vm / batteryVoltage);
  return constrain(pwm, 0, 255);
}

// float getMotorCurrent() {
//   int raw = analogRead(pinMotorSense);
//   float vSense = (raw * 5.0) / 1023.0;     // voltage across shunt
//   return vSense / shuntResistor;           // I = V / R
// }

// ================================================================================================
// Manual speed (steps)
// ================================================================================================
void configureSpeedSteps() {
  for (int i = 0; i < 4; i++) {
    pwmSteps[i] = pwmFromVoltage(min(maxSafeVoltage, voltageSteps[i]));
  }
  Serial.println("Configured speed steps (PWM values):");
  for (int i = 0; i < 4; i++) {
    Serial.print(min(maxSafeVoltage, voltageSteps[i]));
    Serial.print("V → PWM ");
    Serial.println(pwmSteps[i]);
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

  Serial.print("Step ");
  Serial.print(currentStep);
  Serial.print(": target ");
  Serial.print(voltageSteps[currentStep]);
  Serial.print("V → PWM ");
  Serial.println(Speed);

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
// ================================================================================================
// Ultrasonic Auto-speed mapping
// ================================================================================================
float motorVoltageFromDistance(int distance) {
  if (distance < distanceStop) return 0.0;
  if (distance <= distanceStart) return 0.0;

  int lastIdx = sizeof(voltageSteps) / sizeof(voltageSteps[0]) - 1;
  float minV = min(maxSafeVoltage, voltageSteps[1]);        // ≈3.5V
  float maxV = min(maxSafeVoltage, voltageSteps[lastIdx]);  // ≈6.0V

  if (distance < distanceMaxSpeed) {
    float spanV = (maxV - minV);
    float spanD = (float)(distanceMaxSpeed - distanceStart);
    float rawV = minV + (distance - distanceStart) * (spanV / spanD);
    return constrain(rawV, minV, maxV);
  }
  return maxV;  // ≥ 50 cm → full speed
}

void SpeedAutoUltrasonic() {
  Distance = getMedianDistance();

  Serial.print("Ultrasonic Distance: ");
  Serial.print(Distance);
  Serial.println(" cm");

  float targetV = motorVoltageFromDistance(Distance);
  int targetSpeed = pwmFromVoltage(targetV);

  if (targetSpeed == 0) {
    SetRGBColor("red");
    Serial.println("Auto: STOP");
  } else {
    SetRGBColor("white");
    Serial.print("Auto target speed = ");
    Serial.print(targetSpeed);
    Serial.print(" (≈ ");
    Serial.print(targetV, 2);
    Serial.println(" V)");
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
  digitalWrite(pinUltrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinUltrasonicTrig, LOW);

  // timeout = 100 ms
  long duration = pulseIn(pinUltrasonicEcho, HIGH, 100000L);

  if (duration == 0) return distanceMaxSpeed;  // no echo → treat as "far enough"

  int cm = (int)(duration / 58);
  return min(cm, distanceMaxSpeed);  // cap at 50 cm
}

int getMedianDistance() {
  int raw = Distancia_test();         // raw single measurement
  distanceBuffer[bufferIndex] = raw;  // insert into buffer
  bufferIndex = (bufferIndex + 1) % NMedian;
  if (bufferIndex == 0) bufferFilled = true;

  int size = bufferFilled ? NMedian : bufferIndex;
  int temp[NMedian];
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
  Serial.print("Ultrasonic raw=");
  Serial.print(raw);
  Serial.print(" cm  |  median=");
  Serial.print(median);
  Serial.println(" cm");

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
        Serial.println(received, DEC);
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
    Serial.print("Driving Forward >>> Speed=");
    Serial.println(safeSpeed);
  } else if (strcmp(direction, "backward") == 0) {
    analogWrite(pinEngineA_1A, 0);
    analogWrite(pinEngineA_1B, safeSpeed);
    MotorDirection = 2;
    Serial.print("Driving Backward >> Speed=");
    Serial.println(safeSpeed);
  } else {  // stop
    analogWrite(pinEngineA_1A, 0);
    analogWrite(pinEngineA_1B, 0);
    Speed = 0;
    Serial.println("Motor OFF");
  }
}

void GoForward() {
  if (MotorDirection == 2 && Speed > 0) {
    Serial.println("Ignored: cannot switch to FORWARD while moving");
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
    Serial.println("Ignored: cannot switch to BACKWARD while moving");
    return;
  }
  if (MotorDirection == 1 && Speed == 0) {
    Stop();
    delay(DIR_DELAY);
  }
  MotorDirection = 2;
  if (Speed > 0) {
    setMotor("backward", Speed);
    Serial.println("Driving Backward >>");
  }
}

void Stop() {
  setMotor("stop", 0);
}

// Jog motor (doesn’t affect MotorDirection or Speed state)
void JogDrive(const char* direction, int speed) {
  int safeSpeed = constrain(speed, 0, maxSafeSpeed);
  if (strcmp(direction, "forward") == 0) {
    analogWrite(pinEngineA_1A, safeSpeed);
    analogWrite(pinEngineA_1B, 0);
  } else if (strcmp(direction, "backward") == 0) {
    analogWrite(pinEngineA_1A, 0);
    analogWrite(pinEngineA_1B, safeSpeed);
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
    Serial.println("Cancelling jog due to new key");
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
      Serial.println("Jog timeout → STOP");
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
          Serial.println("Ignored: CH- during jog");
          break;
        }
        if (UltrasonicOnOff == 0) {
          decreaseStep();
          Serial.print("Manual Speed Down: ");
          Serial.println(Speed);
          if (Speed == 0) {
            Stop();
            SetRGBColor("red");
          } else {
            if (MotorDirection == 1) GoForward();
            if (MotorDirection == 2) GoBackward();
          }
        } else {
          Serial.println("Ignored: Auto-speed active");
        }
        break;
      }

    case buttonCH:
      {  // Stop
        if (UltrasonicOnOff == 1) {
          UltrasonicOnOff = 0;
          SetGreenLightValue(0);
          Serial.println("Switched from AUTO to MANUAL mode");
        }
        Serial.println("STOP pressed → Motors stopped");
        SetRGBColor("red");
        MotorDirection = 1;  // default to forward when stopped
        Stop();
        currentStep = 0;
        Serial.println("Manual mode reset: next CH+ will start at step 1 (≈3.5V)");
        break;
      }

    case buttonCHplus:
      {  // Speed +
        if (momentaryActive) {
          Serial.println("Ignored: CH+ during jog");
          break;
        }
        if (UltrasonicOnOff == 0) {
          increaseStep();
          Serial.print("Manual Speed Up: ");
          Serial.println(Speed);
          if (MotorDirection == 1) {
            GoForward();
            SetRGBColor("white");
          }
          if (MotorDirection == 2) {
            GoBackward();
            SetRGBColor("blue");
          }
        } else {
          Serial.println("Ignored: Auto-speed active");
        }
        break;
      }

    case buttonBackward:
      {  // << momentary backward (jog)
        if (UltrasonicOnOff == 0 && Speed == 0) {
          int pwm = pwmSteps[1];
          JogDrive("backward", pwm);
          SetRGBColor("blue");
          momentaryActive = true;
          momentaryButton = buttonBackward;
          momentaryLastSeen = millis();
          Serial.println("Momentary BACKWARD running (hold to move)");
        } else {
          Serial.println("Ignored: << only when stationary & not in AUTO");
        }
        break;
      }

    case buttonForward:
      {  // >> momentary forward (jog)
        if (UltrasonicOnOff == 0 && Speed == 0) {
          int pwm = pwmSteps[1];
          JogDrive("forward", pwm);
          SetRGBColor("white");
          momentaryActive = true;
          momentaryButton = buttonForward;
          momentaryLastSeen = millis();
          Serial.println("Momentary FORWARD running (hold to move)");
        } else {
          Serial.println("Ignored: >> only when stationary & not in AUTO");
        }
        break;
      }

    case buttonPlayPause:
      {  // Auto-speed toggle
        UltrasonicOnOff = !UltrasonicOnOff;
        digitalWrite(pinLEDGreen, UltrasonicOnOff ? HIGH : LOW);
        if (UltrasonicOnOff) {
          Serial.println("Driving started (auto-speed)");
          SetRGBColor("white");
          GoForward();
          playPattern(pattern_double);
        } else {
          Serial.println("Driving stopped");
          SetRGBColor("red");
          Stop();
          Speed = 0;
          playPattern(pattern_descend);
          currentStep = 0;
          Serial.println("Manual mode rearmed: next step = 1 (≈3.5V)");
        }
        break;
      }

    case buttonEQ:
      {  // Mute / Unmute
        SoundOnOff = !SoundOnOff;
        Serial.println(SoundOnOff ? "Sound ON" : "Sound OFF");
        if (SoundOnOff) playPattern(pattern_double);
        break;
      }

    case button100plus:
      {  // Horn
        Serial.println("Horn activated");
        playPattern(pattern_horn);
        break;
      }

    case button9:
      {  // Speak battery voltage
        float vIn = getBatteryVoltageDirect();
        Serial.print("Battery Voltage (pattern): ");
        Serial.println(vIn, 1);
        playVoltagePattern(vIn);
        break;
      }

    case button0:
    case button200plus:
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
void updateBuzzer() {
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

void playPattern(const int* pattern) {
  if (SoundOnOff != 1) return;

  digitalWrite(pinBuzzer, LOW);  // 🔹 ensure buzzer off first

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

void playVoltagePattern(float vIn) {
  if (SoundOnOff != 1) return;   // respect mute
  digitalWrite(pinBuzzer, LOW);  // ensure buzzer off first

  int volts = (int)floor(vIn);
  int tenths = ((int)(vIn * 10)) % 10;

  int idx = 0;
  for (int i = 0; i < volts; i++) {
    buzzerPattern[idx++] = 400;
    buzzerPattern[idx++] = 200;
  }
  if (idx % 2 == 0) {
    buzzerPattern[idx++] = 1;
    buzzerPattern[idx++] = 600;
  } else {
    buzzerPattern[idx++] = 600;
  }
  for (int i = 0; i < tenths; i++) {
    buzzerPattern[idx++] = 150;
    buzzerPattern[idx++] = 150;
  }

  buzzerPattern[idx] = 0;
  buzzerIndex = 0;
  buzzerTimer = millis();

  if (buzzerPattern[0] > 0) {
    digitalWrite(pinBuzzer, HIGH);
  }
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

void SetRGBColor(const char* colorName, int led) {
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
  analogWrite(pinLEDGreen, Value);
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
