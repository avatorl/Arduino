const byte REED_PIN = 2;     // D2
const byte LED_PIN  = 13;    // onboard LED

bool readDebounced(byte pin, uint16_t ms = 10) {
  bool s1 = digitalRead(pin);
  uint32_t t = millis() + ms;
  while (millis() < t) {
    bool s2 = digitalRead(pin);
    if (s2 != s1) {          // changed -> restart timeout
      s1 = s2;
      t = millis() + ms;
    }
  }
  return s1;                 // stable state after 'ms'
}

void setup() {
  pinMode(REED_PIN, INPUT_PULLUP);   // NO reed: closed = LOW
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Get a stable reading
  bool level = readDebounced(REED_PIN, 10);   // HIGH=open, LOW=closed (with INPUT_PULLUP)
  bool closed = (level == LOW);

  // Drive LED and edge-report
  digitalWrite(LED_PIN, closed ? HIGH : LOW);

  static bool last = !closed;
  if (closed != last) {
    Serial.println(closed ? "CLOSED" : "OPEN");
    last = closed;
  }

  // optional small idle time so loop isn't 100% busy
  delay(1);
}
