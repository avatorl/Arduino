const byte tiltPin = 12;
const byte ledPin = LED_BUILTIN;

void setup() {
	pinMode(tiltPin, INPUT_PULLUP);
	pinMode(ledPin, OUTPUT);

	Serial.begin(115200);
	while (!Serial) {
		;
	}

	Serial.println(F("Tilt sensor test ready"));
}

void loop() {
	const bool tilted = (digitalRead(tiltPin) == HIGH);

	digitalWrite(ledPin, tilted ? HIGH : LOW);

	Serial.print(F("tilted="));
	Serial.println(tilted ? F("YES") : F("NO"));

	delay(100);
}
