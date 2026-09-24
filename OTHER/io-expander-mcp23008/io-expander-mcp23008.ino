// Assuming there are 8 LEDs connected to the MCP23008 expander, one per GPIO pin.
// This code turns each LED on for a second in sequence from GPIO 0 to GPIO 7. Then repeats indefinitely.
// Tested with https://wiki.kamamilabs.com/index.php?title=KAmod_I2C-Mini_Out_8 module.
// The KAmod I2C-Mini Out8 module contains 8 outputs with low-power N-MOSFET transistors, with a maximum output current of 1 A and a maximum voltage of 50 V.
#include <Wire.h>

constexpr uint8_t mcp23008Address = 0x20;
constexpr uint8_t mcp23008RegisterIodir = 0x00;
constexpr uint8_t mcp23008RegisterGpio = 0x09;
constexpr unsigned long ledOnMs = 1000UL;

uint8_t activeLed = 0;
unsigned long ledChangedAt = 0;

bool writeMcp23008Register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mcp23008Address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool initMcp23008() {
  Wire.beginTransmission(mcp23008Address);
  if (Wire.endTransmission() != 0) return false;

  // Turn outputs off before changing their direction, then make GP0-GP7 outputs.
  if (!writeMcp23008Register(mcp23008RegisterGpio, 0x00)) return false;
  return writeMcp23008Register(mcp23008RegisterIodir, 0x00);
}

void showActiveLed() {
  Serial.println("Active LED: " + String(activeLed));
  writeMcp23008Register(mcp23008RegisterGpio, (uint8_t)(1U << activeLed));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  if (!initMcp23008()) {
    while (true) {
      delay(1000);
    }
  }

  showActiveLed();
  ledChangedAt = millis();
}

void loop() {
  if (millis() - ledChangedAt < ledOnMs) return;

  ledChangedAt = millis();
  activeLed = (activeLed + 1U) % 8U;
  showActiveLed();
}
