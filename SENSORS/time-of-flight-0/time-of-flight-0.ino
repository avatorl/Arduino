#include <Wire.h>
#include <VL53L0X.h> // https://github.com/pololu/vl53l0x-arduino/blob/master/README.md

#define WIRE Wire

VL53L0X distanceSensor;

void setup() {

  byte error, address;

  address = 0x27;

  Serial.begin(115200);
  
  Wire.begin();

  // set the i2c address of the sensor to 0x27

  Serial.println("setting sensor address to 0x27");
  distanceSensor.setAddress(address);

  delay(1000);

  Wire.beginTransmission(address);
  error = Wire.endTransmission();

  if (error == 0) {
      Serial.print("i2c device found at address 0x");
      Serial.println(address, HEX);
  }
  else {
    Serial.print("no i2c device found at address 0x");
    Serial.println(address, HEX);
  }

  delay(5000);

// // ===========================================

  distanceSensor.setTimeout(500);
  if (!distanceSensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  if(!distanceSensor.setSignalRateLimit(1)) {
    Serial.println("Failed to set signal rate limit!");
  }

  // Start continuous back-to-back mode (fastest possible)
  // distanceSensor.startContinuous();

  // Alternative: Continuous timed mode with 100ms between measurements
  distanceSensor.startContinuous(100);

}

void loop() {
  
  // Read range in continuous mode
  uint16_t distance = distanceSensor.readRangeContinuousMillimeters();
  const int buzzerPin = 12;

  Serial.print(distance);
  pinMode(buzzerPin, OUTPUT);

   // Parking sensor sound logic
  if (distance > 500) {
    // Too far away, keep buzzer silent
    digitalWrite(buzzerPin, LOW);
  } 
  else if (distance <= 500 && distance > 300) {
    // Safe zone warning (slow beeps)
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
    delay(500); 
  } 
  else if (distance <= 300 && distance > 100) {
    // Approach zone warning (fast beeps)
    digitalWrite(buzzerPin, HIGH);
    delay(80);
    digitalWrite(buzzerPin, LOW);
    delay(150);
  } 
  else if (distance <= 100 && distance > 0) {
    // Critical stop zone (continuous solid tone)
    digitalWrite(buzzerPin, HIGH);
  }

  if (distanceSensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  Serial.println();
}