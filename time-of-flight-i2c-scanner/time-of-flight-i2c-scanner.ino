#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Set to 1 to release XSHUT by floating the pin (relying on the sensor board's own pull-up to
// bring it back to VCC), as used in a Pololu forum fix for a VL53L1X address-change problem.
// Set to 0 to actively drive XSHUT HIGH instead, as this project's sketches did before.
#define RELEASE_XSHUT_VIA_PULLUP 1

const uint8_t PIN_VL53L0X_XSHUT = 3;
const uint8_t SENSOR_ADDRESS = 0x29;
const uint16_t XSHUT_WAKE_DELAY_MS = 100;
const uint16_t PROBE_RETRY_DELAY_MS = 50;
const uint16_t SENSOR_WAKE_TIMEOUT_MS = 2000;
const uint8_t NEW_SENSOR_ADDRESS = 0x2A;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

bool deviceResponds(uint8_t address);
bool waitForDevice(uint8_t address, uint16_t* elapsedMs);
bool scanFindsDevice(uint8_t address);
void scanAllDevices();
void printHex8(uint8_t value);

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  // 1000us is too tight for the sensor's clock-stretched register access and falsely trips a
  // TWI reset (same root cause already found in time-of-flight.ino), corrupting the bus before
  // lox.begin() runs later in setup().
  Wire.setWireTimeout(25000, true);

  pinMode(PIN_VL53L0X_XSHUT, OUTPUT);

  Serial.println(F("VL53L0X XSHUT scanner"));

  digitalWrite(PIN_VL53L0X_XSHUT, LOW);
  delay(10);
  Serial.println(F("XSHUT LOW: sensor should be hidden."));
  if (deviceResponds(SENSOR_ADDRESS))
  {
    Serial.println(F("Unexpected response at 0x29 while XSHUT is LOW."));
  }
  else
  {
    Serial.println(F("No response at 0x29 while XSHUT is LOW."));
  }

#if RELEASE_XSHUT_VIA_PULLUP
  pinMode(PIN_VL53L0X_XSHUT, INPUT); // float the pin; the sensor board's own pull-up brings XSHUT back to VCC
#else
  digitalWrite(PIN_VL53L0X_XSHUT, HIGH);
#endif
  delay(XSHUT_WAKE_DELAY_MS);
  Serial.println(F("XSHUT HIGH: sensor should respond."));
  uint16_t wakeElapsedMs = 0;
  if (waitForDevice(SENSOR_ADDRESS, &wakeElapsedMs))
  {
    Serial.println(F("Found sensor at 0x29 with XSHUT HIGH."));
    Serial.print(F("Wake probe delay: "));
    Serial.print(wakeElapsedMs);
    Serial.println(F(" ms"));
  }
  else
  {
    Serial.println(F("No response at 0x29 with XSHUT HIGH."));
    Serial.print(F("Waited "));
    Serial.print(wakeElapsedMs);
    Serial.println(F(" ms before giving up."));
  }

  Serial.println(F("Full I2C scan:"));
  scanAllDevices();

  if (scanFindsDevice(SENSOR_ADDRESS))
  {
    Serial.println(F("Scan-based detection confirmed sensor at 0x29."));
  }

  // Adafruit_VL53L0X::begin() does its own DataInit + setAddress() + full sensor init in one
  // call, unlike the raw register write this sketch used before; trying it here to see whether
  // that combined sequence changes the address more reliably.
  Serial.println(F("Attempting lox.begin() with address 0x2A..."));
  if (lox.begin(NEW_SENSOR_ADDRESS))
  {
    Serial.println(F("lox.begin() succeeded at 0x2A."));
  }
  else
  {
    Serial.println(F("lox.begin() failed at 0x2A."));
  }

  delay(50);

  if (deviceResponds(NEW_SENSOR_ADDRESS))
  {
    Serial.println(F("Sensor now responds at 0x2A."));
  }
  else
  {
    Serial.println(F("Sensor does not respond at 0x2A."));
  }

  if (deviceResponds(SENSOR_ADDRESS))
  {
    Serial.println(F("Sensor still responds at 0x29 (address change did not take)."));
  }
  else
  {
    Serial.println(F("Sensor no longer responds at 0x29, as expected after a successful change."));
  }
}

void loop()
{
}

bool deviceResponds(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool waitForDevice(uint8_t address, uint16_t* elapsedMs)
{
  *elapsedMs = 0;

  while (*elapsedMs <= SENSOR_WAKE_TIMEOUT_MS)
  {
    if (deviceResponds(address))
    {
      return true;
    }

    delay(PROBE_RETRY_DELAY_MS);
    *elapsedMs += PROBE_RETRY_DELAY_MS;
  }

  return false;
}

bool scanFindsDevice(uint8_t address)
{
  for (uint8_t currentAddress = 1; currentAddress < 127; currentAddress++)
  {
    if (deviceResponds(currentAddress) && currentAddress == address)
    {
      return true;
    }
  }

  return false;
}

void scanAllDevices()
{
  uint8_t foundCount = 0;

  for (uint8_t address = 1; address < 127; address++)
  {
    if (deviceResponds(address))
    {
      Serial.print(F("I2C device at 0x"));
      printHex8(address);
      Serial.println();
      foundCount++;
    }
  }

  if (foundCount == 0)
  {
    Serial.println(F("No I2C devices found."));
  }
}

void printHex8(uint8_t value)
{
  if (value < 0x10)
  {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}