// Updated 2026-08-28 OK
// Documentation: https://github.com/pololu/vl53l0x-arduino/blob/master/README.md
#include <Wire.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"

// Define the digital pin connected to the VL53L0X XSHUT pin
#define XSHUT_PIN 4 

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire

VL53L0X sensor;

  byte address = 0x27;
  byte address2 = 0x29;

void setup() {
  // =========================================================================
  // STEP 1: HARDWARE ISOLATION (Fixes address conflict safely)
  // =========================================================================
  // Immediately put VL53L0X into shutdown mode before enabling I2C bus communications
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(10); // Wait for the VL53L0X to fully turn off electrically

  

  byte error;


  Serial.begin(115200);
  while (!Serial) {
  ; // Wait for the USB serial port to connect.
}
  Wire.begin();

  // =========================================================================
  // STEP 2: INITIALIZE THE TCS34725 COLOR SENSOR
  // =========================================================================
  // The VL53L0X is asleep, so TCS34725 is the only device listening to 0x29
  float red, green, blue;
  if (tcs.begin(address2)) {
    Serial.println("Found TCS34725 sensor");
    tcs.getRGB(&red, &green, &blue);
    Serial.print("Red: "); Serial.println(red);
    Serial.print("Green: "); Serial.println(green);
    Serial.print("Blue: "); Serial.println(blue);
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  // =========================================================================
  // STEP 3: WAKE UP AND RE-ADDRESS THE VL53L0X DISTANCE SENSOR
  // =========================================================================
  // Wake up VL53L0X. It boots up looking for address 0x29
  digitalWrite(XSHUT_PIN, HIGH);
  delay(10); // Wait for the VL53L0X boot cycle to complete

  // Change its address to 0x27. TCS34725 ignores this register write
  Serial.println("Setting VL53L0X sensor address to 0x27");
  sensor.setAddress(address);
  delay(1000);

  // Validate VL53L0X at its new address
  WIRE.beginTransmission(address);
  error = WIRE.endTransmission();
  if (error == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
  } else {
    Serial.print("No I2C device found at address 0x");
    Serial.println(address, HEX);
  }

  delay(1000);

  // Verify TCS34725 is still responding at default 0x29
  WIRE.beginTransmission(address2);
  error = WIRE.endTransmission();
  if (error == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address2, HEX);
  } else {
    Serial.print("No I2C device found at address 0x");
    Serial.println(address2, HEX);
  }

  delay(5000);

  // Initialize VL53L0X logic parameters now that it is isolated at 0x27
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L0X sensor!");
    while (1) {}
  }

  if(!sensor.setSignalRateLimit(1)) {
    Serial.println("Failed to set signal rate limit!");
  }

  // Start continuous back-to-back mode (fastest possible)
  //sensor.startContinuous();
  // Continuous timed mode with 100ms between measurements
  sensor.startContinuous(100);
}

void loop() {
  // Read range in continuous mode
  uint16_t distance = sensor.readRangeContinuousMillimeters();

  Serial.print(distance);
  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  Serial.println();

    float red, green, blue;

    tcs.getRGB(&red, &green, &blue);
    Serial.print("Red: "); Serial.println(red);
    Serial.print("Green: "); Serial.println(green);
    Serial.print("Blue: "); Serial.println(blue);

}