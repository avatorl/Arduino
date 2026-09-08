// Backup of the custom TCS34725 driver replaced by Adafruit_TCS34725.
struct TrainColorSensorTCS34725 {
  static const uint8_t DefaultAddress = 0x29;
  static const uint8_t CommandBit = 0x80;
  static const uint8_t CommandAutoIncrement = 0x20;
  static const uint8_t RegisterEnable = 0x00;
  static const uint8_t RegisterAtime = 0x01;
  static const uint8_t RegisterId = 0x12;
  static const uint8_t RegisterControl = 0x0F;
  static const uint8_t RegisterClearDataLow = 0x14;
  static const uint8_t EnablePowerOn = 0x01;
  static const uint8_t EnableAdc = 0x02;
  static const uint8_t IntegrationTime50ms = 0xEB;
  static const uint8_t Gain4x = 0x01;

  uint8_t address = DefaultAddress;

  bool begin_I2C(uint8_t i2cAddress = DefaultAddress) {
    address = i2cAddress;
    uint8_t deviceId = 0;
    if (!probe()) return false;
    if (!readRegister(RegisterId, &deviceId)) return false;
    if (!isSupportedDeviceId(deviceId)) return false;
    if (!writeRegister(RegisterAtime, IntegrationTime50ms)) return false;
    return writeRegister(RegisterControl, Gain4x);
  }

  bool enable() {
    if (!writeRegister(RegisterEnable, EnablePowerOn)) return false;
    delay(3);
    return writeRegister(RegisterEnable, (uint8_t)(EnablePowerOn | EnableAdc));
  }

  bool disable() {
    return writeRegister(RegisterEnable, 0x00);
  }

  bool readRawData(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
    uint8_t raw[8] = { 0 };
    if (!readRegisters(RegisterClearDataLow, raw, sizeof(raw))) {
      *r = 0;
      *g = 0;
      *b = 0;
      *c = 0;
      return false;
    }
    *c = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    *r = (uint16_t)raw[2] | ((uint16_t)raw[3] << 8);
    *g = (uint16_t)raw[4] | ((uint16_t)raw[5] << 8);
    *b = (uint16_t)raw[6] | ((uint16_t)raw[7] << 8);
    return true;
  }

 private:
  bool isSupportedDeviceId(uint8_t deviceId) {
    return deviceId == 0x44 || deviceId == 0x4D || deviceId == 0x10;
  }

  bool probe() {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
  }

  bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write((uint8_t)(CommandBit | reg));
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }

  bool readRegister(uint8_t reg, uint8_t* value) {
    return readRegisters(reg, value, 1);
  }

  bool readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(address);
    Wire.write((uint8_t)(CommandBit | CommandAutoIncrement | startReg));
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)address, (int)length) != length) return false;
    for (uint8_t i = 0; i < length; ++i) buffer[i] = (uint8_t)Wire.read();
    return true;
  }
};
