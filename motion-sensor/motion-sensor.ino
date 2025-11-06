const int PIN_PIR_SENSOR = 7;   // PIR OUT pin
const int PIN_LED = 13;

int pinStateCurrent  = LOW;    // current PIR reading
int pinStatePrevious = LOW;    // previous PIR reading

unsigned long lowStartedAt = 0;          // when PIR went LOW
bool lowTimerActive = false;             // are we currently in the 3s grace period?

const unsigned long EXTRA_ON_MS = 5000;  // stay ON, milliseconds after sensor PIN_TO_SENSOR went LOW

void setup() {
  Serial.begin(9600);
  pinMode(PIN_PIR_SENSOR, INPUT);
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  pinStateCurrent = digitalRead(PIN_PIR_SENSOR);
  unsigned long now = millis();

  // Detect transitions
  // 1. HIGH -> LOW: PIR stopped seeing motion
  if (pinStatePrevious == HIGH && pinStateCurrent == LOW) {
    lowStartedAt = now;
    lowTimerActive = true;
  }

  // 2. LOW -> HIGH: new motion again
  if (pinStatePrevious == LOW && pinStateCurrent == HIGH) {
    // motion is back, so LED should be on no matter what
    // also cancel any "countdown to off"
    lowTimerActive = false;
  }

  // LED logic:
  // - If PIR is HIGH: LED ON.
  // - Else (PIR LOW): keep LED ON if we're still within 3s after LOW started.
  bool ledShouldBeOn = false;

  if (pinStateCurrent == HIGH) {
    ledShouldBeOn = true;
  } else {
    // PIR is LOW
    if (lowTimerActive && (now - lowStartedAt < EXTRA_ON_MS)) {
      ledShouldBeOn = true;
    } else {
      ledShouldBeOn = false;
      // after timeout expires, no need to keep counting
      if (lowTimerActive && (now - lowStartedAt >= EXTRA_ON_MS)) {
        lowTimerActive = false;
      }
    }
  }

  digitalWrite(PIN_LED, ledShouldBeOn ? HIGH : LOW);

  // debug (optional)
  // Serial.print("PIR=");
  // Serial.print(pinStateCurrent);
  // Serial.print(" LED=");
  // Serial.print(ledShouldBeOn);
  // Serial.print(" tSinceLow=");
  // Serial.println(now - lowStartedAt);

  // remember for next loop
  pinStatePrevious = pinStateCurrent;

  delay(10);
}