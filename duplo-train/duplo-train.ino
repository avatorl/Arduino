// ================================================================================================
// Train compatible with Lego DUPLO (3D-printed parts + Arduino)
// Based on: https://cults3d.com/en/3d-model/game/locomotora-sofia-controlada-por-infrarojos-con-control-de-velocidad-por-ultrasonidos-y-motor-superior-multiusos-lego-duplo
// Arduino code rewritten & extended by Andrzej Leszkiewicz
// Printing profile: https://makerworld.com/en/models/1854728-arduino-train-locomotive-remote-controlled#profileId-1983131
// ================================================================================================

// Electronic components
    // Arduino Nano 3.0 ATMEGA328 CH340
    // IR sensor HX1838 with wiring adapter
    // Active buzzer
    // 2 x RGB LED , 1 x green LED
    // DC motor, motor driver HG7881 L9110S
    // Ultrasonic distance sensor HC-SR04
// Resistors:
// LED RGB R: 100 Ω + 47 Ω, G: 100 Ω, B: 100 Ω
// LED Green: 100 Ω + 47 Ω
// Voltage divider: 10K kΩ and 4.7 kΩ
// ================================================================================================

#include <IRremote.hpp> // required for IR sensor
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
const int buttonCHminus   = 69;  // Speed -
const int buttonCH        = 70;  // Stop
const int buttonCHplus    = 71;  // Speed +
const int buttonBackward  = 68;  // Direction: Backward
const int buttonForward   = 64;  // Direction: Forward
const int buttonPlayPause = 67;  // Auto-speed toggle, start/stop
const int buttonEQ        = 9;   // Mute / Unmute
const int button0         = 22;  
const int button100plus   = 25;  
const int button200plus   = 13;  
const int button1 = 12;
const int button2 = 24;
const int button3 = 94;
const int button4 = 8;
const int button5 = 28;
const int button6 = 90;
const int button7 = 66;
const int button8 = 82;
const int button9 = 74; // Battery Test

// ================================================================================================
// Arduino Pin Mapping
// ================================================================================================
const int pinBatterySense =     A0;   // Battery voltage monitoring
const int pinUltrasonicTrig   = A3;   // Ultrasonic sensor trigger
const int pinUltrasonicEcho   = A4;   // Ultrasonic sensor echo
const int pinIRReceiver       = A5;   // IR receiver input

const int pinEngineA_1A       = 5;    // Motor direction and speed (PWN)
const int pinEngineA_1B       = 6;    // Motor direction and speed (PWN)
const int pinLED1_R = 2, pinLED1_G = 3, pinLED1_B = 4;  // RGB LED #1 (ON/OFF only, no PWN required)
const int pinLED2_R = 7, pinLED2_G = 8, pinLED2_B = 9;  // RGB LED #2 (ON/OFF only, no PWN required)
const int pinLEDGreen         = 10;   // Green LED (PWN required)
const int pinBuzzer           = 12;   // Passive buzzer

// ================================================================================================
// Buzzer sound patterns
// ================================================================================================
const int pattern_melody[]   = {150, 80, 200, 80, 250, 80, 300, 150, 250, 0};
const int pattern_batteryWarn[] = {3000, 100, 0};
const int pattern_double[]   = {150, 100, 150, 0};
const int pattern_single[]   = {200, 0};
const int pattern_descend[]  = {120, 80, 120, 80, 120, 0};
const int pattern_double2[]  = {120, 80, 120, 0};
const int pattern_horn[]     = {400, 200, 400, 200, 400, 0};

// ================================================================================================
// Other constants
// ================================================================================================
const unsigned long DIR_DELAY = 1000; // delay before motor direction change, ms
const float R1 = 10000.0;   // Top resistor (to battery +)
const float R2 = 4700.0;  // Bottom resistor (to GND)
const int NMedian   = 5;   // number of samples for median filter
const int maxSafeSpeed = 180;   // Motor safety speed limit (batte)

// ================================================================================================
// Control variables
// ================================================================================================
int FrontLightOnOff = 1;   // Front lights state (ON/OFF) - RGB LEDs only
int SoundOnOff      = 1;   // Sound state (ON/OFF)
int UltrasonicOnOff = 0;   // Ultrasonic state (ON/OFF)
int MotorDirection = 1;   // always has a direction
int Speed = 0;            // stopped at start
int SpeedStep       = 60;  // Increment/decrement step
int Distance        = 0;   // Latest distance from ultrasonic
//int VStopGo         = 0;   // Auto restart flag after obstacle

int distanceBuffer[NMedian];
int bufferIndex = 0;
bool bufferFilled = false;

#include <LowPower.h>   // RocketScream LowPower library

unsigned long lastActive = 0;
const unsigned long idleTimeout = 15UL * 60UL * 1000UL; // 15 minutes
// In this state, the Arduino will never wake up unless you configure an external interrupt pin (like D2 or D3) tied to your IR receiver.
// Right now, with A5 for IR, sleep means hard freeze until reset.

// ================================================================================================
// Motor speed steps based on battery voltage
// ================================================================================================
// Computed PWM values (0–255)
int pwmSteps[4];   // same size as voltageSteps
// Desired motor voltages for steps (V)
float voltageSteps[] = {0.0, 3.5, 4.5, 6.0};
float batteryVoltage = 0.0;

// ================================================================================================
// Active Buzzer Pattern Player
// ================================================================================================
int buzzerPattern[40];
int buzzerIndex = 0;
unsigned long buzzerTimer = 0;

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

  playPattern(pattern_melody); // Play melody

  SetGreenLightValue(0);

  if (FrontLightOnOff == 1) {
    SetRGBColor("red");
  } else { // required because default state is ON (common anode LEDs)
    SetRGBColor("off");
  }

  // Measure battery at startup
  batteryVoltage = getBatteryVoltageDirect();
  Serial.print("Battery measured: ");
  Serial.println(batteryVoltage, 2);

  // Configure dynamic speed steps
  configureSpeedSteps();  

  lastActive = millis();   // seed idle timer

  IrReceiver.begin(pinIRReceiver); // Start IR Receiver

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

  // static unsigned long lastCheck = 0;
  // if (millis() - lastCheck > 10000) {  // every N sec get battery voltage (time in miliseconds)
  //   getBatteryVoltage();
  //   lastCheck = millis();
  // }

  // check idle timeout
  if (millis() - lastActive > idleTimeout) {
    goToIdle();
  }

  delay(10);

}

void goToIdle() {
  Serial.println("Idle: turning everything off...");

  // stop buzzer and clear pattern
  digitalWrite(pinBuzzer, LOW);
  buzzerPattern[0] = 0;
  buzzerIndex = 0;

  // turn off LEDs
  SetRGBColor("off");
  SetGreenLightValue(0);

  // notify with short melody
  playPattern(pattern_descend);
  delay(2000);  // let it play fully
  digitalWrite(pinBuzzer, LOW);

  // finally sleep
  Serial.println("Entering sleep mode...");
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

  // after wake up (external interrupt only!)
  lastActive = millis();  
}

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
  float vm = min(desiredMotorV, 6.0);
  int pwm = (int)(255.0 * vm / batteryVoltage);
  return constrain(pwm, 0, 255);
}


// ================================================================================================
// Manual speed up / down (use steps instead of fixed increments)
// ================================================================================================
int currentStep = 0;   // 0 = stop, 1 = 3V, 2 = 4.5V, 3 = 6V

void applySpeedStep() {
  Speed = pwmSteps[currentStep];
  
  Serial.print("Step ");
  Serial.print(currentStep);
  Serial.print(": target ");
  Serial.print(voltageSteps[currentStep]);
  Serial.print("V → PWM ");
  Serial.println(Speed);

  playStepBeep(currentStep);   // 🔊 NEW: sound feedback

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
// Ultrasonic speed control (auto mode)
// ================================================================================================
const int autoRestartMin = 50;  // don’t restart unless >= this
int rampStep             = 5;   // PWM increment per update
unsigned long rampDelay  = 80;  // ms between ramp changes
static unsigned long lastRamp = 0;

void SpeedAutoUltrasonic() {
  Distance = getMedianDistance();

  Serial.print("Ultrasonic Distance: ");
  Serial.print(Distance);
  Serial.println(" cm");

  // --- Map distance → effective motor voltage ---
  float targetV = motorVoltageFromDistance(Distance);

  // Convert to PWM relative to battery voltage
  int targetSpeed = pwmFromVoltage(targetV);

  // LED indication
  if (targetSpeed == 0) {
    SetRGBColor("red");   // stopped
    Serial.println("Auto: STOP");
  } else {
    SetRGBColor("white"); // moving
    Serial.print("Auto target speed = ");
    Serial.print(targetSpeed);
    Serial.print(" (≈ ");
    Serial.print(targetV, 2);
    Serial.println(" V effective)");
  }

  updateMotorSpeed(targetSpeed);
}

void updateMotorSpeed(int targetSpeed) {
  unsigned long now = millis();
  if (now - lastRamp < rampDelay) return; // only update at intervals
  lastRamp = now;

  // Smooth ramp up/down
  if (Speed < targetSpeed) {
    Speed = min(Speed + rampStep, targetSpeed);
  } else if (Speed > targetSpeed) {
    Speed = max(Speed - rampStep, targetSpeed);
  }

  if (targetSpeed == 0) {
    Speed = 0;
    Stop();
    return;   // override ramp
  }

  // Apply motor control
  if (Speed == 0) {
    Stop();
  } else {
    setMotor("forward", Speed);  // auto mode always forward
  }
}

// ================================================================================================
// Read battery voltage using a resistive divider
//
// Voltage divider circuit:
//   Battery+ ── R1 ──┬── R2 ── GND
//                    │
//                    └── Arduino analog pin (pinBatterySense)
//
// Formula:
//   Vout = Vin * R2 / (R1 + R2)
//   Vin  = Vout * (R1 + R2) / R2
// ================================================================================================
void getBatteryVoltage() {
  
  int raw = analogRead(pinBatterySense); // ADC value (0–1023)

  // Convert ADC value to voltage at the analog pin (Vout)
  float vOut = raw * 5.0 / 1023.0;      

  // Recalculate the actual battery voltage (Vin) from divider ratio
  float vIn  = vOut * (R1 + R2) / R2;   

  // Optional: trigger audio battery indicator
  indicateBattery(vIn);

  // Print result to serial monitor with 2 decimal places
  Serial.print("Battery Voltage: ");
  Serial.println(vIn, 2);
}

// Map distance to effective motor voltage (3V → 6V)
float motorVoltageFromDistance(int distance) {
  if (distance < 7) return 0.0;    // too close → stop
  if (distance <= 15) return 0.0;  // stay stopped until clear
  
  // Linear mapping: distance → voltage
  // Example: for distance 20 → ~3V, for large distance → up to 6V
  float rawV = distance * 0.1 + 1.0;  // tune scaling as needed
  return constrain(rawV, 3.0, 6.0);
}

// ================================================================================================
// Battery status indicator (using LEDs and Buzzer)
// ================================================================================================
void indicateBattery(float vIn) {
  if (vIn > 6.5) {
    // Do nothing (normal operation)
    return;
  } 
  else if (vIn > 6.0) {
    // Medium battery: turn off lights and sound alert
    SetRGBColor("off");
    FrontLightOnOff = 0;
    // Trigger 3-second beep (non-blocking)
    playPattern(pattern_batteryWarn);
  } 
  else {
    // Low battery: red warning lights
    SetGreenLightValue(0);
    SetRGBColor("red");
    // Trigger 3-second beep (non-blocking)
    playPattern(pattern_batteryWarn);    
  }
}

// ================================================================================================
// IR Remote receiver wrapper
// ================================================================================================
uint16_t irReceive() {
  uint16_t received{ 0 };
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == NEC) {
      if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
        IrReceiver.resume();
        return 0; // ignore repeats
      }
      received = IrReceiver.decodedIRData.command;
      Serial.println(received, DEC); // Debug
    }
    IrReceiver.resume();
  }
  return received;
}

// ================================================================================================
// IR Remote command handler
// ================================================================================================
void translateIR() {
  uint16_t code = irReceive();
  if (code == 0) return;          // no key → do nothing
  lastActive = millis();           // ✅ real user activity seen

  switch (code) {

    case buttonCHminus: // Speed -
      GreenLEDBlink();
      if (UltrasonicOnOff == 0) {
        decreaseStep();
        //Speed = decreaseSpeed(Speed, SpeedStep);
        Serial.print("Manual Speed Down: ");
        Serial.println(Speed);
        if (Speed == 0) {
          Stop();
          Serial.println("Motors stopped.");
          SetRGBColor("red");
        } else {
          if (MotorDirection == 1) GoForward();
          if (MotorDirection == 2) GoBackward();
        }
      } else {
        Serial.println("Ignored: Auto-speed active");
      }
      break;

    case buttonCH: // Stop
      GreenLEDBlink();
      if (UltrasonicOnOff == 1) {
        UltrasonicOnOff = 0;
        SetGreenLightValue(0);
        Serial.println("Switched from AUTO to MANUAL mode");
      }
      Serial.println("STOP button pressed → Motors stopped");
      SetRGBColor("red");
      MotorDirection = 1; // stop forces forward direction
      Stop();
      Speed = 0;

      // 🔧 Reset manual mode step
      currentStep = 0;
      Serial.println("Manual mode reset: next CH+ will start at step 1 (≈3.5V)");
      break;

    case buttonCHplus: // Speed +
      GreenLEDBlink();
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
      
    case buttonBackward: // Direction: Backward
      GreenLEDBlink();
      if (Speed == 0) {
        MotorDirection = 2;
        Serial.println("Direction set: BACKWARD (waiting for speed increase)");
        SetRGBColor("red", 1);
        SetRGBColor("blue", 2);
      } else {
        Serial.println("Ignored: cannot change direction while moving");
      }
      break;

    case buttonForward: // Direction: Forward
      GreenLEDBlink();
      if (Speed == 0) {
        MotorDirection = 1;
        SetRGBColor("white", 1);
        SetRGBColor("red", 2);
        Serial.println("Direction set: FORWARD (waiting for speed increase)");
      } else {
        Serial.println("Ignored: cannot change direction while moving");
      }
      break;

    case buttonPlayPause: // Auto-speed toggle
      GreenLEDBlink();
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

        // Reset manual mode to minimal speed step
        currentStep = 0;   // so next CH+ or CH- starts at lowest speed
        Serial.println("Manual mode rearmed: next step = 1 (≈3.5V)");
      }
      break;

    case buttonEQ: // Mute / Unmute
      GreenLEDBlink();
      SoundOnOff = !SoundOnOff;
      Serial.println(SoundOnOff ? "Sound ON" : "Sound OFF");
      if (SoundOnOff) {
        playPattern(pattern_double);
      } else {
        //GreenLEDBlink();
      }
      break;

    case button5:
      GreenLEDBlink();
      break;

    case button0:
      GreenLEDBlink();
      // VStopGo = !VStopGo;
      // Serial.println(VStopGo ? "Auto-restart ON" : "Auto-restart OFF");
      // VStopGo ? playPattern(pattern_descend) : playPattern(pattern_double);
      break;

    case button100plus: // Horn
      GreenLEDBlink();
      Serial.println("Horn activated");
      playPattern(pattern_horn);
      break;

    case button200plus:
      GreenLEDBlink();
      break;

    case button9: // Speak battery voltage
      GreenLEDBlink();
      float vIn = getBatteryVoltageDirect();
      Serial.print("Battery Voltage (pattern): ");
      Serial.println(vIn, 1);
      playVoltagePattern(vIn);
      break;
            
  }
}

// ================================================================================================
// Motor Control
// ================================================================================================
void setMotor(const char* direction, int speed) {

  int safeSpeed = constrain(speed, 0, maxSafeSpeed);

  if (strcmp(direction, "forward") == 0) {
    analogWrite(pinEngineA_1A, safeSpeed);
    analogWrite(pinEngineA_1B, 0);
    MotorDirection = 1;
    Serial.print("Driving Forward >>> Speed=");
    Serial.println(safeSpeed);
  }
  else if (strcmp(direction, "backward") == 0) {
    analogWrite(pinEngineA_1A, 0);
    analogWrite(pinEngineA_1B, safeSpeed);
    MotorDirection = 2;
    Serial.print("Driving Backward >> Speed=");
    Serial.println(safeSpeed);
  }
  else if (strcmp(direction, "stop") == 0) {
    analogWrite(pinEngineA_1A, 0);
    analogWrite(pinEngineA_1B, 0);
    Speed = 0;
    Serial.println("Motor OFF");
  }
}

void GoForward() {
  if (MotorDirection == 2 && Speed > 0) {
    // Don’t change direction while moving
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
    Serial.println("Driving Forward >>>");
  }
}

void GoBackward() {
  if (MotorDirection == 1 && Speed > 0) {
    // Don’t change direction while moving
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
    Serial.print("Motor OFF\n");
}

int getMedianDistance() {
  // Store new reading in the circular buffer
  distanceBuffer[bufferIndex] = Distancia_test();
  bufferIndex = (bufferIndex + 1) % NMedian;
  if (bufferIndex == 0) bufferFilled = true;

  // Determine buffer size (before it fills for the first time)
  int size = bufferFilled ? NMedian : bufferIndex;

  // Copy values to temp array for sorting
  int temp[NMedian];
  for (int i = 0; i < size; i++) temp[i] = distanceBuffer[i];

  // Simple bubble sort (small N, so it's fine)
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (temp[j] < temp[i]) {
        int swap = temp[i];
        temp[i] = temp[j];
        temp[j] = swap;
      }
    }
  }

  // Return the median value
  return temp[size / 2];
}

// ================================================================================================
// Ultrasonic Sensor
// ================================================================================================
int Distancia_test() {
  digitalWrite(pinUltrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinUltrasonicTrig, LOW);
  long duration = pulseIn(pinUltrasonicEcho, HIGH, 30000); // max 30ms
  if (duration == 0) return 400; // no echo → assume "very far" (400cm)
  return (int)(duration / 58);
}




// ================================================================================================
// Play beep pattern for current step
// Step 0 = stop (special pattern), Step 1..3 = N short beeps
// ================================================================================================
void playStepBeep(int step) {
  // Clear any old pattern
  for (int j = 0; j < 20; j++) buzzerPattern[j] = 0;

  int idx = 0;
  if (step == 0) {
    // Descending pattern (STOP)
    buzzerPattern[idx++] = 300; // ON
    buzzerPattern[idx++] = 200; // OFF
    buzzerPattern[idx++] = 200; // ON
    buzzerPattern[idx++] = 300; // OFF
  } else {
    // Short beeps = number of step
    for (int i = 0; i < step; i++) {
      buzzerPattern[idx++] = 150; // ON
      buzzerPattern[idx++] = 150; // OFF
    }
  }
  buzzerPattern[idx] = 0;   // terminate

  buzzerIndex = 0;
  buzzerTimer = millis();

  // Start playback (sound + green LED)
  if (buzzerPattern[0] > 0) {
    digitalWrite(pinBuzzer, HIGH);
    analogWrite(pinLEDGreen, 255); // LED ON
  } else {
    digitalWrite(pinBuzzer, LOW);
    analogWrite(pinLEDGreen, 0);   // LED OFF
  }
}

// Setup speed steps (call in setup)
void configureSpeedSteps() {
  for (int i = 0; i < 4; i++) {
    pwmSteps[i] = pwmFromVoltage(voltageSteps[i]);
  }

  Serial.println("Configured speed steps (PWM values):");
  for (int i = 0; i < 4; i++) {
    Serial.print(voltageSteps[i]); Serial.print("V → PWM ");
    Serial.println(pwmSteps[i]);
  }
}

// ================================================================================================
//Instead of delay() beeping (which would freeze the Arduino), it uses millis() timing and a pattern array 
//  so the buzzer can “play” while the rest of the program keeps running.
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

    if (buzzerIndex % 2 == 0) {
      digitalWrite(pinBuzzer, HIGH);
    } else {
      digitalWrite(pinBuzzer, LOW);
    }
  }
}

// ================================================================================================
// Play sound pattern
void playPattern(const int *pattern) {
  if (SoundOnOff != 1) return;

  for (int j = 0; j < 10; j++) {
    buzzerPattern[j] = 0;
  }

  int i = 0;
  while (i < 9) {
    buzzerPattern[i] = pattern[i];
    if (pattern[i] == 0) break;
    i++;
  }
  buzzerPattern[i] = 0;

  buzzerIndex = 0;
  buzzerTimer = millis();

  if (buzzerPattern[0] > 0) {
    digitalWrite(pinBuzzer, HIGH);
  } else {
    digitalWrite(pinBuzzer, LOW);
  }
}

// ================================================================================================
// Change RGB LED #1 state (default state is HIGH for all)
void SetRGB1Light(bool R, bool G, bool B) {
  if (FrontLightOnOff == 0) return;
  digitalWrite(pinLED1_R, R ? LOW : HIGH);
  digitalWrite(pinLED1_G, G ? LOW : HIGH);
  digitalWrite(pinLED1_B, B ? LOW : HIGH);
}

// Change RGB LED #2 state (default state is HIGH for all)
void SetRGB2Light(bool R, bool G, bool B) {
  if (FrontLightOnOff == 0) return;
  digitalWrite(pinLED2_R, R ? LOW : HIGH);
  digitalWrite(pinLED2_G, G ? LOW : HIGH);
  digitalWrite(pinLED2_B, B ? LOW : HIGH);
}

// ================================================================================================
// Change RGB LED color
// led = 0 → both, led = 1 → only LED1, led = 2 → only LED2
void SetRGBLight(bool R, bool G, bool B, int led) {
  if (led == 0 || led == 1) SetRGB1Light(R, G, B);
  if (led == 0 || led == 2) SetRGB2Light(R, G, B);
}

// ================================================================================================
// Wrapper: set RGB light by color name
// led = 0 → both, led = 1 → only LED1, led = 2 → only LED2
void SetRGBColor(const char* colorName, int led) {
  if (strcmp(colorName, "red") == 0) {
    SetRGBLight(true, false, false, led);
  } else if (strcmp(colorName, "green") == 0) {
    SetRGBLight(false, true, false, led);
  } else if (strcmp(colorName, "blue") == 0) {
    SetRGBLight(false, false, true, led);
  } else if (strcmp(colorName, "yellow") == 0) {
    SetRGBLight(true, true, false, led);
  } else if (strcmp(colorName, "cyan") == 0) {
    SetRGBLight(false, true, true, led);
  } else if (strcmp(colorName, "magenta") == 0) {
    SetRGBLight(true, false, true, led);
  } else if (strcmp(colorName, "white") == 0) {
    SetRGBLight(true, true, true, led);
  } else if (strcmp(colorName, "off") == 0) {
    SetRGBLight(false, false, false, led);
  } else {
    // default fallback
    SetRGBLight(false, false, false, led);
  }
}

// change Green LED brightness
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

void LightBlink3Times() {
  for (int i = 0; i < 3; i++) {
    SetRGBColor("white");
    delay(150);
    SetRGBColor("off");
    delay(150);
  }
}

void playVoltagePattern(float vIn) {
  // Truncate parts safely
  int volts  = (int)floor(vIn);
  int tenths = ((int)(vIn * 10)) % 10;

  int idx = 0;

  // --- Integer volts: long beeps (ON 400, OFF 200)
  for (int i = 0; i < volts; i++) {
    buzzerPattern[idx++] = 400; // ON
    buzzerPattern[idx++] = 200; // OFF
  }

  // --- Pause BEFORE decimal part must be OFF time.
  // If idx is even, the next slot would be treated as ON by updateBuzzer(),
  // so we insert a 1ms ON spacer first, then the OFF pause.
  if (idx % 2 == 0) {
    buzzerPattern[idx++] = 1;   // tiny ON (inaudible)
    buzzerPattern[idx++] = 600; // OFF pause
  } else {
    buzzerPattern[idx++] = 600; // already OFF slot → clean pause
  }

  // --- Decimal tenths: short beeps (ON 150, OFF 150)
  for (int i = 0; i < tenths; i++) {
    buzzerPattern[idx++] = 150; // ON
    buzzerPattern[idx++] = 150; // OFF
  }

  // Terminate
  buzzerPattern[idx] = 0;

  // Start playback
  buzzerIndex = 0;
  buzzerTimer = millis();
  if (buzzerPattern[0] > 0) {
    digitalWrite(pinBuzzer, HIGH);
    analogWrite(pinLEDGreen, 255);  // sync LED
  }
}
