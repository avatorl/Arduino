  // ================================================================================================
  // File description
  // ================================================================================================
  // Color sensor, distance sensor, and tilt sensor helpers live here.
  // These settings mainly affect sensor power control, marker-color detection, obstacle sensing,
  // and tilt protection behavior.

  // TCS34725 color-sensor helper.
  // This struct is a minimal, hand-written driver for the TCS34725 RGB color sensor, talking to it
  // directly over I2C register reads/writes instead of using the official "Adafruit_TCS34725"
  // library. This is intentional: the Adafruit library pulls in extra code/RAM for features this
  // sketch does not need (gain/integration-time enums, interrupt thresholds, Adafruit_Sensor
  // framework glue, etc.), and an 8-bit AVR chip like the Nano has very little flash (32 KB) and RAM
  // (2 KB) to spare. By only implementing the handful of registers actually used here, this driver
  // keeps the compiled program much smaller.
  // If you wanted to use the official library instead, the equivalent calls would be:
  //   Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);  // ~= begin_I2C() below
  //   tcs.begin();          // probes the sensor and configures ATIME/gain, same as begin_I2C()
  //   tcs.setInterrupt(false);  // roughly equivalent to enable()/disable() below (turns the ADC on/off)
  //   tcs.getRawData(&r, &g, &b, &c);  // same result as readRawData() below
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
      Wire.begin();
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.setWireTimeout(3000, true);
      Wire.clearWireTimeoutFlag();
      #endif
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

      // The sensor sends each 16-bit color channel as two separate 8-bit bytes, low byte first
      // ("little-endian"). Combining them back into one 16-bit number uses the same bitmask/shift
      // idiom seen elsewhere in this codebase (e.g., TrainLedMCP23008 above): "raw[1] << 8" moves
      // the high byte into the upper 8 bits of a 16-bit value, and "|" merges it with the low byte.
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
      // Wire.endTransmission(false) sends what was queued but keeps the I2C bus held open with a
      // "repeated start" instead of a full stop (the plain endTransmission() used elsewhere in this
      // file releases the bus). This is required here because the sensor needs to see "here's the
      // register I want, now immediately read back its value" as one unbroken transaction; a
      // separate stop+start could let another device interrupt in between on a shared bus.
      if (Wire.endTransmission(false) != 0) return false;

      // Wire.requestFrom(address, length) asks the sensor to send back "length" bytes; the return
      // value is how many bytes it actually received (compared against the requested length below to
      // detect a short/failed read). Wire.read() then pops one buffered byte at a time out of the
      // I2C receive buffer that requestFrom() just filled.
      uint8_t bytesRead = Wire.requestFrom((int)address, (int)length);
      if (bytesRead != length) return false;

      for (uint8_t i = 0; i < length; ++i) {
        buffer[i] = (uint8_t)Wire.read();
      }
      return true;
    }
  };

  struct TrainAccelerometerMPU6050 {
    static const uint8_t RegisterWhoAmI = 0x75;
    static const uint8_t RegisterPowerManagement1 = 0x6B;
    static const uint8_t RegisterAccelConfig = 0x1C;
    static const uint8_t RegisterAccelXoutHigh = 0x3B;

    uint8_t address = mpu6050Address;

    bool begin_I2C() {
      Wire.begin();
      address = mpu6050Address;

      uint8_t whoAmI = 0;
      if (!readRegister(RegisterWhoAmI, &whoAmI)) return false;
      if (whoAmI != 0x68 && whoAmI != 0x69) return false;
      if (!writeRegister(RegisterPowerManagement1, 0x00)) return false;
      return writeRegister(RegisterAccelConfig, mpu6050AccelConfig);
    }

    bool readAcceleration(int16_t* x, int16_t* y, int16_t* z) {
      uint8_t bytes[6] = { 0 };
      if (!readRegisters(RegisterAccelXoutHigh, bytes, sizeof(bytes))) return false;

      const uint16_t rawX = ((uint16_t)bytes[0] << 8) | bytes[1];
      const uint16_t rawY = ((uint16_t)bytes[2] << 8) | bytes[3];
      const uint16_t rawZ = ((uint16_t)bytes[4] << 8) | bytes[5];
      *x = (int16_t)rawX;
      *y = (int16_t)rawY;
      *z = (int16_t)rawZ;
      return true;
    }

   private:
    bool writeRegister(uint8_t reg, uint8_t value) {
      Wire.beginTransmission(address);
      Wire.write(reg);
      Wire.write(value);
      return Wire.endTransmission() == 0;
    }

    bool readRegister(uint8_t reg, uint8_t* value) {
      return readRegisters(reg, value, 1);
    }

    bool readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length) {
      Wire.beginTransmission(address);
      Wire.write(startReg);
      if (Wire.endTransmission(false) != 0) return false;

      if (Wire.requestFrom((int)address, (int)length) != length) return false;
      for (uint8_t i = 0; i < length; ++i) {
        buffer[i] = (uint8_t)Wire.read();
      }
      return true;
    }
  };

  // VL53L0X distance-sensor helper.
  // Like the TCS34725 helper above, this is a hand-written register-level driver for the VL53L0X
  // time-of-flight distance sensor, instead of using a ready-made library such as Pololu's
  // "VL53L0X" Arduino library or ST's official API. The full official library includes a much larger
  // initialization sequence, calibration routines, and configuration options for use-cases this
  // train doesn't need (long-range mode, high-accuracy timing budgets, continuous interrupt-driven
  // ranging, etc.), all of which cost flash and RAM this sketch cannot spare alongside the color
  // sensor, LED expander, and IR remote code. This driver keeps only the initialization/ranging
  // steps actually needed to get single-shot distance readings.
  // If you wanted to use the official library instead, the equivalent calls would be:
  //   VL53L0X sensor;              // Pololu library object, roughly ~= this whole struct
  //   sensor.init();                // full sensor init/calibration, ~= the startup sequence below
  //   sensor.setTimeout(500);        // ~= ioTimeoutMs handling in this driver
  //   sensor.readRangeSingleMillimeters();  // ~= a single ranging cycle performed by this driver
  struct TrainDistanceSensorVL53L0X {
    static const uint8_t DefaultAddress = 0x29;
    static const uint8_t RegisterSysrangeStart = 0x00;
    static const uint8_t RegisterSystemSequenceConfig = 0x01;
    static const uint8_t RegisterSystemIntermeasurementPeriod = 0x04;
    static const uint8_t RegisterSystemInterruptConfigGpio = 0x0A;
    static const uint8_t RegisterSystemInterruptClear = 0x0B;
    static const uint8_t RegisterResultInterruptStatus = 0x13;
    static const uint8_t RegisterResultRangeStatus = 0x14;
    static const uint8_t RegisterFinalRangeConfigMinCountRateRtnLimit = 0x44;
    static const uint8_t RegisterMsrcConfigTimeoutMacrop = 0x46;
    static const uint8_t RegisterMsrcConfigControl = 0x60;
    static const uint8_t RegisterSystemHistogramBin = 0x81;
    static const uint8_t RegisterGpioHvMuxActiveHigh = 0x84;
    static const uint8_t RegisterVhvConfigPadSclSdaExtsupHv = 0x89;
    static const uint8_t RegisterI2cSlaveDeviceAddress = 0x8A;
    static const uint8_t RegisterGlobalConfigSpadEnablesRef0 = 0xB0;
    static const uint8_t RegisterGlobalConfigRefEnStartSelect = 0xB6;
    static const uint8_t RegisterIdentificationModelId = 0xC0;
    static const uint8_t RegisterOscCalibrateVal = 0xF8;

    uint8_t address = DefaultAddress;
    uint16_t ioTimeoutMs = 0;
    uint8_t stopVariable = 0;
    uint32_t measurementTimingBudgetUs = 33000UL;
    bool didTimeout = false;
    bool lastReadValid = false;
    unsigned long timeoutStartMs = 0;

    void setTimeout(uint16_t timeoutMs) {
      ioTimeoutMs = timeoutMs;
    }

    bool setAddress(uint8_t newAddress) {
      if (!writeRegAt(address, RegisterI2cSlaveDeviceAddress, newAddress & 0x7F)) {
        if (address == DefaultAddress) return false;
        if (!writeRegAt(DefaultAddress, RegisterI2cSlaveDeviceAddress, newAddress & 0x7F)) return false;
      }
      address = newAddress;
      return true;
    }

    bool init(bool io2v8 = true) {
      // This whole function replicates ST's official (and largely undocumented) VL53L0X
      // initialization sequence, exactly as reverse-engineered/published in well-known open-source
      // VL53L0X drivers (e.g. Pololu's). Many of the register addresses/values below (0x88, 0x80,
      // 0xFF, etc.) are undocumented "magic numbers" required by the sensor's internal calibration
      // and timing hardware; they are copied verbatim rather than explained line-by-line because ST
      // never published what each one individually does - only that the full sequence, in this
      // order, is required to bring the sensor into a working state.
      Wire.begin();
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.setWireTimeout(3000, true);
      Wire.clearWireTimeoutFlag();
      #endif
      didTimeout = false;
      lastReadValid = false;

      if (readReg(RegisterIdentificationModelId) != 0xEE) return false;

      if (io2v8) {
        if (!writeReg(RegisterVhvConfigPadSclSdaExtsupHv,
                      (uint8_t)(readReg(RegisterVhvConfigPadSclSdaExtsupHv) | 0x01))) return false;
      }

      if (!writeReg(0x88, 0x00)) return false;
      if (!writeReg(0x80, 0x01)) return false;
      if (!writeReg(0xFF, 0x01)) return false;
      if (!writeReg(0x00, 0x00)) return false;
      stopVariable = readReg(0x91);
      if (!writeReg(0x00, 0x01)) return false;
      if (!writeReg(0xFF, 0x00)) return false;
      if (!writeReg(0x80, 0x00)) return false;

      if (!writeReg(RegisterMsrcConfigControl, (uint8_t)(readReg(RegisterMsrcConfigControl) | 0x12))) return false;
      if (!setSignalRateLimit(0.25f)) return false;
      if (!writeReg(RegisterSystemSequenceConfig, 0xFF)) return false;

      uint8_t spadCount = 0;
      bool spadTypeIsAperture = false;
      if (!getSpadInfo(&spadCount, &spadTypeIsAperture)) return false;

      uint8_t refSpadMap[6] = { 0 };
      if (!readMulti(RegisterGlobalConfigSpadEnablesRef0, refSpadMap, sizeof(refSpadMap))) return false;

      if (!writeReg(0xFF, 0x01)) return false;
      if (!writeReg(0x4F, 0x00)) return false;
      if (!writeReg(0x4E, 0x2C)) return false;
      if (!writeReg(0xFF, 0x00)) return false;
      if (!writeReg(RegisterGlobalConfigRefEnStartSelect, 0xB4)) return false;

      uint8_t firstSpadToEnable = spadTypeIsAperture ? 12 : 0;
      uint8_t spadsEnabled = 0;
      // SPADs (Single Photon Avalanche Diodes) are the sensor's individual light-detecting elements;
      // refSpadMap is a 6-byte array where each bit represents one of 48 SPADs (byte "i / 8", bit
      // "i % 8" - the same per-bit addressing idiom as the MCP23008 shadow-register code earlier in
      // this project). This loop walks all 48 SPADs and clears (disables) any bit outside the
      // sensor-specific enabled range, leaving only the correct reference set turned on, as required
      // by the calibration step that follows.
      for (uint8_t i = 0; i < 48; ++i) {
        if (i < firstSpadToEnable || spadsEnabled == spadCount) {
          refSpadMap[i / 8] &= (uint8_t)~(1 << (i % 8));
        } else if ((refSpadMap[i / 8] >> (i % 8)) & 0x01) {
          ++spadsEnabled;
        }
      }
      if (!writeMulti(RegisterGlobalConfigSpadEnablesRef0, refSpadMap, sizeof(refSpadMap))) return false;

      // This table is the bulk of ST's magic init sequence: a flat list of [register, value] pairs
      // applied in order. It is stored in PROGMEM (flash) - not a regular array - because at ~150
      // bytes it would otherwise permanently use up a meaningful chunk of the Nano's tiny 2 KB of
      // SRAM for a table that is only ever read once, sequentially, at startup. "static const ...
      // PROGMEM = { ... }" declares the table once (shared across calls, not recreated each time)
      // and tells the compiler to place it in flash instead of RAM; pgm_read_byte() (used in the
      // loop below) is then required to actually fetch each byte from flash, the same way
      // pgm_read_word() was used for color patterns in 30-lights-and-sounds.ino.
      static const uint8_t initRegisters[][2] PROGMEM = {
        { 0xFF, 0x01 }, { 0x00, 0x00 }, { 0xFF, 0x00 }, { 0x09, 0x00 }, { 0x10, 0x00 },
        { 0x11, 0x00 }, { 0x24, 0x01 }, { 0x25, 0xFF }, { 0x75, 0x00 }, { 0xFF, 0x01 },
        { 0x4E, 0x2C }, { 0x48, 0x00 }, { 0x30, 0x20 }, { 0xFF, 0x00 }, { 0x30, 0x09 },
        { 0x54, 0x00 }, { 0x31, 0x04 }, { 0x32, 0x03 }, { 0x40, 0x83 }, { 0x46, 0x25 },
        { 0x60, 0x00 }, { 0x27, 0x00 }, { 0x50, 0x06 }, { 0x51, 0x00 }, { 0x52, 0x96 },
        { 0x56, 0x08 }, { 0x57, 0x30 }, { 0x61, 0x00 }, { 0x62, 0x00 }, { 0x64, 0x00 },
        { 0x65, 0x00 }, { 0x66, 0xA0 }, { 0xFF, 0x01 }, { 0x22, 0x32 }, { 0x47, 0x14 },
        { 0x49, 0xFF }, { 0x4A, 0x00 }, { 0xFF, 0x00 }, { 0x7A, 0x0A }, { 0x7B, 0x00 },
        { 0x78, 0x21 }, { 0xFF, 0x01 }, { 0x23, 0x34 }, { 0x42, 0x00 }, { 0x44, 0xFF },
        { 0x45, 0x26 }, { 0x46, 0x05 }, { 0x40, 0x40 }, { 0x0E, 0x06 }, { 0x20, 0x1A },
        { 0x43, 0x40 }, { 0xFF, 0x00 }, { 0x34, 0x03 }, { 0x35, 0x44 }, { 0xFF, 0x01 },
        { 0x31, 0x04 }, { 0x4B, 0x09 }, { 0x4C, 0x05 }, { 0x4D, 0x04 }, { 0xFF, 0x00 },
        { 0x44, 0x00 }, { 0x45, 0x20 }, { 0x47, 0x08 }, { 0x48, 0x28 }, { 0x67, 0x00 },
        { 0x70, 0x04 }, { 0x71, 0x01 }, { 0x72, 0xFE }, { 0x76, 0x00 }, { 0x77, 0x00 },
        { 0xFF, 0x01 }, { 0x0D, 0x01 }, { 0xFF, 0x00 }, { 0x80, 0x01 }, { 0x01, 0xF8 },
        { 0xFF, 0x01 }, { 0x8E, 0x01 }, { 0x00, 0x01 }, { 0xFF, 0x00 }, { 0x80, 0x00 }
      };
      for (uint8_t i = 0; i < sizeof(initRegisters) / sizeof(initRegisters[0]); ++i) {
        uint8_t reg = pgm_read_byte(&initRegisters[i][0]);
        uint8_t value = pgm_read_byte(&initRegisters[i][1]);
        if (!writeReg(reg, value)) return false;
      }

      if (!writeReg(RegisterSystemInterruptConfigGpio, 0x04)) return false;
      if (!writeReg(RegisterGpioHvMuxActiveHigh, (uint8_t)(readReg(RegisterGpioHvMuxActiveHigh) & ~0x10))) return false;
      if (!writeReg(RegisterSystemInterruptClear, 0x01)) return false;
      if (!writeReg(RegisterSystemSequenceConfig, 0xE8)) return false;
      measurementTimingBudgetUs = 33000UL;

      if (!performSingleRefCalibration(0x40)) return false;
      if (!writeReg(RegisterSystemSequenceConfig, 0x02)) return false;
      if (!performSingleRefCalibration(0x00)) return false;
      return writeReg(RegisterSystemSequenceConfig, 0xE8);
    }

    bool setMeasurementTimingBudget(uint32_t budgetUs) {
      if (budgetUs < 20000UL) return false;
      measurementTimingBudgetUs = budgetUs;
      return true;
    }

    void startContinuous(uint32_t periodMs = 0) {
      didTimeout = false;
      lastReadValid = false;

      writeReg(0x80, 0x01);
      writeReg(0xFF, 0x01);
      writeReg(0x00, 0x00);
      writeReg(0x91, stopVariable);
      writeReg(0x00, 0x01);
      writeReg(0xFF, 0x00);
      writeReg(0x80, 0x00);

      if (periodMs != 0) {
        uint16_t oscCalibrateVal = readReg16Bit(RegisterOscCalibrateVal);
        if (oscCalibrateVal != 0) {
          periodMs *= oscCalibrateVal;
        }
        writeReg32Bit(RegisterSystemIntermeasurementPeriod, periodMs);
        writeReg(RegisterSysrangeStart, 0x04);
      } else {
        writeReg(RegisterSysrangeStart, 0x02);
      }
    }

    uint16_t readRangeContinuousMillimeters() {
      didTimeout = false;
      if ((readReg(RegisterResultInterruptStatus) & 0x07) == 0) {
        lastReadValid = false;
        return 65535;
      }

      uint16_t range = readReg16Bit(RegisterResultRangeStatus + 10);
      writeReg(RegisterSystemInterruptClear, 0x01);
      lastReadValid = range != 0 && range != 65535;
      return range;
    }

    bool timeoutOccurred() {
      bool timedOut = didTimeout;
      didTimeout = false;
      return timedOut;
    }

    bool lastRangeReadValid() const {
      return lastReadValid;
    }

   private:
    void startTimeout() {
      timeoutStartMs = millis();
    }

    bool hasTimedOut() const {
      return ioTimeoutMs > 0 && (uint16_t)(millis() - timeoutStartMs) > ioTimeoutMs;
    }

    bool writeReg(uint8_t reg, uint8_t value) {
      return writeRegAt(address, reg, value);
    }

    bool writeRegAt(uint8_t targetAddress, uint8_t reg, uint8_t value) {
      Wire.beginTransmission(targetAddress);
      Wire.write(reg);
      Wire.write(value);
      return Wire.endTransmission() == 0;
    }

    bool writeReg16Bit(uint8_t reg, uint16_t value) {
      Wire.beginTransmission(address);
      Wire.write(reg);
      // Sends the 16-bit value as two bytes, most-significant byte first ("big-endian"), which is
      // what this sensor's registers expect (opposite order from the color sensor's little-endian
      // data above) - always check a chip's datasheet for its expected byte order.
      Wire.write((uint8_t)(value >> 8));
      Wire.write((uint8_t)value);
      return Wire.endTransmission() == 0;
    }

    bool writeReg32Bit(uint8_t reg, uint32_t value) {
      Wire.beginTransmission(address);
      Wire.write(reg);
      Wire.write((uint8_t)(value >> 24));
      Wire.write((uint8_t)(value >> 16));
      Wire.write((uint8_t)(value >> 8));
      Wire.write((uint8_t)value);
      return Wire.endTransmission() == 0;
    }

    uint8_t readReg(uint8_t reg) {
      uint8_t value = 0;
      readMulti(reg, &value, 1);
      return value;
    }

    uint16_t readReg16Bit(uint8_t reg) {
      uint8_t data[2] = { 0, 0 };
      readMulti(reg, data, sizeof(data));
      return ((uint16_t)data[0] << 8) | data[1];
    }

    bool writeMulti(uint8_t reg, const uint8_t* data, uint8_t length) {
      Wire.beginTransmission(address);
      Wire.write(reg);
      for (uint8_t i = 0; i < length; ++i) {
        Wire.write(data[i]);
      }
      return Wire.endTransmission() == 0;
    }

    bool readMulti(uint8_t reg, uint8_t* data, uint8_t length) {
      Wire.beginTransmission(address);
      Wire.write(reg);
      if (Wire.endTransmission(false) != 0) return false;

      uint8_t bytesRead = Wire.requestFrom((int)address, (int)length);
      if (bytesRead != length) return false;

      for (uint8_t i = 0; i < length; ++i) {
        data[i] = (uint8_t)Wire.read();
      }
      return true;
    }

    bool setSignalRateLimit(float limitMcps) {
      if (limitMcps < 0.0f || limitMcps > 511.99f) return false;
      return writeReg16Bit(RegisterFinalRangeConfigMinCountRateRtnLimit, (uint16_t)(limitMcps * 128.0f));
    }

    bool getSpadInfo(uint8_t* count, bool* typeIsAperture) {
      // Another block of ST's undocumented sensor-internal register sequence (see the comment at
      // the top of init() above) - it puts the sensor into a special mode that reports how many
      // SPADs (light-sensing elements) it has and which type, then restores normal operation
      // afterward.
      if (!writeReg(0x80, 0x01)) return false;
      if (!writeReg(0xFF, 0x01)) return false;
      if (!writeReg(0x00, 0x00)) return false;
      if (!writeReg(0xFF, 0x06)) return false;
      if (!writeReg(0x83, (uint8_t)(readReg(0x83) | 0x04))) return false;
      if (!writeReg(0xFF, 0x07)) return false;
      if (!writeReg(0x81, 0x01)) return false;
      if (!writeReg(0x80, 0x01)) return false;
      if (!writeReg(0x94, 0x6B)) return false;
      if (!writeReg(0x83, 0x00)) return false;

      // Poll register 0x83 until the sensor signals it's ready (non-zero), bailing out via
      // hasTimedOut() if it never responds in time rather than hanging forever.
      startTimeout();
      while (readReg(0x83) == 0x00) {
        if (hasTimedOut()) return false;
      }

      if (!writeReg(0x83, 0x01)) return false;
      uint8_t value = readReg(0x92);
      // The SPAD count and "is aperture type" flag are packed together into one byte: the low 7 bits
      // (mask 0x7F) hold the count, and the top bit (shifted down with ">> 7", then masked with
      // 0x01) holds the type flag - a common way to squeeze two small pieces of data into one
      // register.
      *count = value & 0x7F;
      *typeIsAperture = ((value >> 7) & 0x01) != 0;

      if (!writeReg(0x81, 0x00)) return false;
      if (!writeReg(0xFF, 0x06)) return false;
      if (!writeReg(0x83, (uint8_t)(readReg(0x83) & ~0x04))) return false;
      if (!writeReg(0xFF, 0x01)) return false;
      if (!writeReg(0x00, 0x01)) return false;
      if (!writeReg(0xFF, 0x00)) return false;
      return writeReg(0x80, 0x00);
    }

    bool performSingleRefCalibration(uint8_t vhvInitByte) {
      if (!writeReg(RegisterSysrangeStart, (uint8_t)(0x01 | vhvInitByte))) return false;

      startTimeout();
      while ((readReg(RegisterResultInterruptStatus) & 0x07) == 0) {
        if (hasTimedOut()) return false;
      }

      if (!writeReg(RegisterSystemInterruptClear, 0x01)) return false;
      return writeReg(RegisterSysrangeStart, 0x00);
    }
  };

  // D4 is dedicated to the TCS34725 breakout LED control input in this revision.
  TrainColorSensorTCS34725 colorSensor;
  bool colorSensorDetected = false;
  TrainAccelerometerMPU6050 accelerometer;
  bool accelerometerDetected = false;
  unsigned long lastAccelerometerReadMs = 0;
  unsigned long accelerometerTiltStartedMs = 0;
  bool accelerometerTiltTiming = false;
  unsigned long lastAccelerometerDebugMs = 0;
  bool accelerometerPreviousForwardSampleValid = false;
  int32_t accelerometerPreviousForwardRaw = 0;

  // TCS34725 Low-Power Sleep / Power-Down Notes:
  // - The sensor IC has an internal sleep/power-down state (~1-2 uA) controlled via I2C.
  // - Turning off the onboard LED via D4 saves ~15-20 mA (the dominant current draw).
  // - setColorSensorEnabled(false) powers down both the LED and the sensor core.

  // Color-sensor sampling and classification state.
  unsigned long lastColorSensorRead = 0;
  const unsigned long colorSensorReadEveryMs = 60;
  uint8_t lastTrackMarkerClass = 255;
  const uint16_t colorClearMinThreshold = 160;
  const uint16_t colorMatchClearThreshold = 304;
  const float whiteBalanceRedGain = 1.000f;
  const float whiteBalanceGreenGain = 1.212f;
  const float whiteBalanceBlueGain = 2.318f;

  // markerClusters is a lookup table of known track-marker colors (measured RGB "center" plus an
  // allowed matching radius) used by the color-matching code further down to classify what color the
  // sensor is currently seeing. It's declared "PROGMEM" (like the melody/pattern tables elsewhere in
  // this project) because it's a fairly large read-only table (19 entries) that never changes at
  // runtime - keeping it in flash instead of RAM leaves much more of the Nano's tiny 2 KB SRAM free
  // for everything else. Because it's in flash, code that reads these entries must use
  // "memcpy_P"/"pgm_read_*" rather than a plain struct copy/array index (see classifyTrackMarkerColor
  // further below for how this table is actually read, field by field, with pgm_read_word/byte).
  const MarkerClusterDefinition markerClusters[] PROGMEM = {
    { MarkerWhite, { 334, 333, 333 }, 32 },
    { MarkerBrown, { 392, 322, 287 }, 31 },
    { MarkerBrown, { 415, 305, 280 }, 22 },
    { MarkerBrown, { 428, 289, 284 }, 25 },
    { MarkerCyan, { 158, 313, 529 }, 60 },
    { MarkerGreen, { 160, 518, 322 }, 62 },
    { MarkerGreen, { 203, 482, 315 }, 52 },
    { MarkerGrey, { 303, 345, 352 }, 38 },
    { MarkerGrey, { 327, 331, 342 }, 22 },
    { MarkerGrey, { 347, 317, 336 }, 27 },
    { MarkerMagenta, { 455, 201, 344 }, 14 },
    { MarkerMagenta, { 476, 193, 331 }, 18 },
    { MarkerMagenta, { 488, 186, 326 }, 10 },
    { MarkerOrange, { 542, 254, 204 }, 22 },
    { MarkerRed, { 514, 180, 305 }, 27 },
    { MarkerRed, { 550, 191, 259 }, 36 },
    { MarkerRed, { 572, 176, 252 }, 22 },
    { MarkerYellow, { 420, 368, 212 }, 14 },
    { MarkerYellow, { 432, 353, 215 }, 18 },
  };

  const uint8_t markerClusterCount = sizeof(markerClusters) / sizeof(markerClusters[0]);

  // Distance-sensor runtime state and filtering.
  uint8_t Distance = 0;
  uint8_t distanceBuffer[AUTO_SAMPLES_FOR_MEDIAN];
  uint8_t bufferIndex = 0;
  bool bufferFilled = false;
  TrainDistanceSensorVL53L0X distanceTof;
  bool distanceTofDetected = false;
  bool distanceTofFaultLatched = false;
  uint8_t distanceTofConsecutiveFailures = 0;
  unsigned long lastTofReadMs = 0;
  unsigned long lastGoodTofReadMs = 0;
  const uint16_t distanceTofTimeoutMs = 50;
  const uint32_t distanceTofTimingBudgetUs = 33000UL;
  const uint32_t distanceTofContinuousPeriodMs = 50UL;
  const unsigned long tofReadEveryMs = 50;
  const uint8_t maxConsecutiveTofFailures = 3;
  const unsigned long tofFailureGraceMs = 250UL;

  // Sensor-wide hardware setup groups tilt pin setup, distance reset, and color-sensor startup.
  void initSensorHardware() {
    pinMode(pinTiltSensor, INPUT_PULLUP);
    accelerometerDetected = accelerometer.begin_I2C();
    if (accelerometerDetected) {
      DBGLN_ACCELEROMETER(F("MPU-6050 ready"));
    } else {
      DBGLN_ACCELEROMETER(F("MPU-6050 not detected; accelerometer safety disabled"));
    }
    initDistanceSensorHardware();
    initColorSensorHardware();
  }

  // Put the TCS34725 core into sleep when the system is shutting down permanently.
  void powerDownColorSensorCore() {
    if (colorSensorDetected) {
      colorSensor.disable();
    }
  }

  // Sort a small sample buffer in place and return its median element.
  // Uses a simple "selection sort" (fine for the very small AUTO_SAMPLES_FOR_MEDIAN buffer size used
  // here - not efficient for large arrays, but simplicity/small code size matter more than raw speed
  // for a handful of samples). The "median" (middle value once sorted) is used instead of an average
  // because it ignores one-off spurious readings (e.g. a single bad distance sample) much better
  // than an average would - a classic noise-filtering technique for sensor data.
  uint8_t medianFromUnsortedSamples(uint8_t* values, uint8_t size) {
    if (values == nullptr || size == 0) return 0;

    for (uint8_t i = 0; i < (uint8_t)(size - 1); ++i) {
      for (uint8_t j = (uint8_t)(i + 1); j < size; ++j) {
        if (values[j] < values[i]) {
          const uint8_t swap = values[i];
          values[i] = values[j];
          values[j] = swap;
        }
      }
    }

    return values[size / 2];
  }

  // Configure the TCS34725 and its onboard lamp control.
  void initColorSensorHardware() {
    pinMode(pinColorSensorLED, OUTPUT);
    digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);

    colorSensorDetected = colorSensor.begin_I2C();
    if (colorSensorDetected) {
      colorSensor.disable();  // Keep the sensor IC core in low-power sleep (~2 uA) until enabled
      DBGLN_COLOR_SENSOR(F("TCS34725 ready (in sleep mode)"));
    } else {
      DBGLN_COLOR_SENSOR(F("TCS34725 not detected on I2C"));
    }
  }

  // Toggle color-sensor mode and related status lighting.
  void setColorSensorEnabled(bool enabled) {
    if (enabled && !isColorSensorAllowed()) {
      DBGLN_COLOR_SENSOR(F("Ignored: color sensor disabled by battery policy"));
      return;
    }
    if (enabled && !colorSensorDetected) {
      DBGLN_COLOR_SENSOR(F("Ignored: TCS34725 not detected"));
      return;
    }

    ColorSensorOnOff = enabled ? 1 : 0;
    digitalWrite(pinColorSensorLED, enabled ? colorSensorLEDOnLevel : colorSensorLEDOffLevel);

    if (enabled) {
      if (colorSensorDetected) colorSensor.enable();
      DBGLN_COLOR_SENSOR(F("Color sensor ON"));
      if (!sirenActive) SetRGBLightColor(RgbColor::Cyan);
    } else {
      if (colorSensorDetected) colorSensor.disable();
      DBGLN_COLOR_SENSOR(F("Color sensor OFF"));
      refreshDriveLights();
    }
  }

  // Apply per-channel gain correction to raw RGBC data.
  // Real sensors respond a little differently to red/green/blue light depending on their physical
  // construction, so "white" light doesn't naturally read back as equal R=G=B without correction.
  // These fixed calibration gains (measured once against a known-white surface) scale each channel
  // so a genuinely white/grey object reads back roughly balanced, which the color-classification
  // code below depends on. "+ 0.5f" before truncating to an integer is a common rounding trick
  // (rounds to nearest instead of always rounding down when converting float -> uint16_t).
  BalancedRgbs applyWhiteBalance(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    BalancedRgbs balanced;
    balanced.r = (uint16_t)(r * whiteBalanceRedGain + 0.5f);
    balanced.g = (uint16_t)(g * whiteBalanceGreenGain + 0.5f);
    balanced.b = (uint16_t)(b * whiteBalanceBlueGain + 0.5f);
    balanced.c = c;
    return balanced;
  }

  // Convert RGB counts to a 0-1000 normalized triple.
  PrototypeRgb normalizePrototype(uint16_t r, uint16_t g, uint16_t b) {
    PrototypeRgb prototype = { 0, 0, 0 };
    uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;
    if (sum == 0) return prototype;

    prototype.r = (uint16_t)(((uint32_t)r * 1000UL + (sum / 2)) / sum);
    prototype.g = (uint16_t)(((uint32_t)g * 1000UL + (sum / 2)) / sum);
    prototype.b = (uint16_t)(((uint32_t)b * 1000UL + (sum / 2)) / sum);
    return prototype;
  }

  // Manhattan distance between two normalized RGB prototypes.
  // "Manhattan distance" (also called taxicab distance) adds up the absolute difference of each
  // coordinate separately (|dR| + |dG| + |dB|), unlike straight-line ("Euclidean") distance which
  // would involve squaring and a square root. It's used here because it's much cheaper to compute on
  // a small AVR chip (no multiplication or sqrt() needed) while still being a perfectly reasonable
  // way to measure "how different are these two colors".
  uint16_t prototypeDistance(const PrototypeRgb& a, const PrototypeRgb& b) {
    uint16_t distance = 0;
    distance += (a.r > b.r) ? (a.r - b.r) : (b.r - a.r);
    distance += (a.g > b.g) ? (a.g - b.g) : (b.g - a.g);
    distance += (a.b > b.b) ? (a.b - b.b) : (b.b - a.b);
    return distance;
  }

  // Convert raw RGBC values into track marker classes.
  uint8_t classifyTrackMarkerColor(uint16_t rawR, uint16_t rawG, uint16_t rawB, uint16_t rawC) {
    if (rawC < colorClearMinThreshold || rawC < colorMatchClearThreshold) return MarkerUnknown;

    BalancedRgbs balanced = applyWhiteBalance(rawR, rawG, rawB, rawC);
    PrototypeRgb measured = normalizePrototype(balanced.r, balanced.g, balanced.b);
    if (measured.r == 0 && measured.g == 0 && measured.b == 0) return MarkerUnknown;

    uint16_t bestDistance = 0xFFFF;
    TrackMarkerClass bestClass = MarkerUnknown;

    // Walk every known marker color in the PROGMEM table (see the markerClusters comment above) and
    // keep the closest match that's still within that cluster's own allowed radius
    // (clusterMaxDistance) - this rejects colors that don't clearly belong to any known marker
    // instead of always forcing a "best guess" match.
    for (uint8_t i = 0; i < markerClusterCount; ++i) {
      PrototypeRgb clusterCenter = {
        pgm_read_word(&markerClusters[i].center.r),
        pgm_read_word(&markerClusters[i].center.g),
        pgm_read_word(&markerClusters[i].center.b)
      };
      uint16_t clusterMaxDistance = pgm_read_word(&markerClusters[i].maxDistance);
      TrackMarkerClass clusterClass = (TrackMarkerClass)pgm_read_byte(&markerClusters[i].markerClass);
      uint16_t distance = prototypeDistance(measured, clusterCenter);
      if (distance <= clusterMaxDistance && distance < bestDistance) {
        bestDistance = distance;
        bestClass = clusterClass;
      }
    }

    return (uint8_t)bestClass;
  }

  #if DEBUG_COLOR_SENSOR
  // Debug-only marker name lookup.
  const __FlashStringHelper* trackMarkerLabel(uint8_t markerClass) {
    switch (markerClass) {
      case MarkerWhite: return F("white");
      case MarkerBrown: return F("brown");
      case MarkerCyan: return F("cyan");
      case MarkerGreen: return F("green");
      case MarkerGrey: return F("grey");
      case MarkerMagenta: return F("magenta");
      case MarkerOrange: return F("orange");
      case MarkerYellow: return F("yellow");
      case MarkerRed: return F("red");
      default: return F("unknown");
    }
  }
  #endif

  // Run the action associated with a detected marker color.
  void handleTrackMarkerAction(uint8_t markerClass) {
    switch (markerClass) {
      case MarkerWhite:
        DBGLN_COLOR_SENSOR(F("Track action: WHITE marker (no action defined yet)"));
        break;

      case MarkerBrown:
        DBGLN_COLOR_SENSOR(F("Track action: BROWN marker (no action defined yet)"));
        break;

      case MarkerCyan:
        DBGLN_COLOR_SENSOR(F("Track action: CYAN marker (no action defined yet)"));
        break;

      case MarkerRed:
        DBGLN_COLOR_SENSOR(F("Track action: RED marker (no action defined yet)"));
        break;

      case MarkerGreen:
        DBGLN_COLOR_SENSOR(F("Track action: GREEN marker (no action defined yet)"));
        break;

      case MarkerGrey:
        DBGLN_COLOR_SENSOR(F("Track action: GREY marker (no action defined yet)"));
        break;

      case MarkerMagenta:
        DBGLN_COLOR_SENSOR(F("Track action: MAGENTA marker (no action defined yet)"));
        break;

      case MarkerOrange:
        DBGLN_COLOR_SENSOR(F("Track action: ORANGE marker (no action defined yet)"));
        break;

      case MarkerYellow:
        DBGLN_COLOR_SENSOR(F("Track action: YELLOW marker (no action defined yet)"));
        break;

      default:
        break;
    }
  }

  // Poll and process the color sensor without blocking.
  void updateColorSensor() {
    if (ColorSensorOnOff == 0 || !colorSensorDetected) return;

    unsigned long now = millis();
    if (now - lastColorSensorRead < colorSensorReadEveryMs) return;
    lastColorSensorRead = now;

    uint16_t r = 0, g = 0, b = 0, c = 0;
    colorSensor.readRawData(&r, &g, &b, &c);
    uint8_t markerClass = classifyTrackMarkerColor(r, g, b, c);
    #if DEBUG_COLOR_SENSOR
    const __FlashStringHelper* markerLabel = trackMarkerLabel(markerClass);
    DBG_COLOR_SENSOR(F("TCS34725 RGBC: "));
    DBG_COLOR_SENSOR(r);
    DBG_COLOR_SENSOR(F(", "));
    DBG_COLOR_SENSOR(g);
    DBG_COLOR_SENSOR(F(", "));
    DBG_COLOR_SENSOR(b);
    DBG_COLOR_SENSOR(F(", clear="));
    DBG_COLOR_SENSOR(c);
    DBG_COLOR_SENSOR(F(" -> "));
    DBGLN_COLOR_SENSOR(markerLabel);
    #endif

    if (markerClass != lastTrackMarkerClass) {
      lastTrackMarkerClass = markerClass;
      #if DEBUG_COLOR_SENSOR
      DBG_COLOR_SENSOR(F("Track marker changed to: "));
      DBGLN_COLOR_SENSOR(markerLabel);
      #endif
      handleTrackMarkerAction(markerClass);
    }
  }

  // ================================================================================================
  // Distance sensor
  // ================================================================================================
  // Insert a sample into the median filter and return result.
  // distanceBuffer is a "ring buffer" (also called a circular buffer): bufferIndex always points to
  // the next slot to overwrite, and "(bufferIndex + 1) % AUTO_SAMPLES_FOR_MEDIAN" wraps it back to 0
  // once it reaches the end, so the array cycles through forever without ever needing to shift
  // existing elements. "bufferFilled" tracks whether the buffer has wrapped around at least once
  // (i.e., has AUTO_SAMPLES_FOR_MEDIAN real samples yet, vs. still filling up for the first time). A
  // temporary copy is sorted (via medianFromUnsortedSamples above) rather than sorting the ring
  // buffer itself, since sorting would scramble the FIFO order needed for the next overwrite.
  int pushDistanceSampleAndGetMedian(int raw) {
    distanceBuffer[bufferIndex] = raw;
    bufferIndex = (bufferIndex + 1) % AUTO_SAMPLES_FOR_MEDIAN;
    if (bufferIndex == 0) bufferFilled = true;

    uint8_t size = bufferFilled ? AUTO_SAMPLES_FOR_MEDIAN : bufferIndex;
    uint8_t temp[AUTO_SAMPLES_FOR_MEDIAN];
    for (uint8_t i = 0; i < size; i++) temp[i] = distanceBuffer[i];
    return medianFromUnsortedSamples(temp, size);
  }

  // Latch a distance-sensor fault, stop the train, and disable auto-distance mode.
  void handleDistanceSensorFault() {
    if (distanceTofFaultLatched) return;

    distanceTofFaultLatched = true;
    distanceTofDetected = false;
    distanceTofConsecutiveFailures = maxConsecutiveTofFailures;
    Distance = 0;
    pendingMotorStopReason = F("auto: no response from distance sensor");
    exitAutoDistanceMode();
    stopAndResetStepSelection();
    SetRGBColor(RgbColor::Red);
    playPattern(pattern_batteryWarn);
    DBGLN_DISTANCE_SENSOR(F("VL53L0X fault: auto-distance mode disabled until sensor is reinitialized"));
    #if ENABLE_EEPROM_LOGGING
    logBatteryEvent(EEPROM_EVENT_TOF_FAULT, batteryVoltage);
    #endif
  }

  // Return the latest filtered distance in cm.
  int getDistanceReading() {
    if (!distanceTofDetected) {
      handleDistanceSensorFault();
      return AUTO_DISTANCE_INVALID;
    }

    unsigned long now = millis();
    if (now - lastTofReadMs < tofReadEveryMs) {
      if (!bufferFilled && bufferIndex == 0) return AUTO_DISTANCE_INVALID;
      return Distance;
    }
    lastTofReadMs = now;

    uint16_t rawMm = distanceTof.readRangeContinuousMillimeters();
    if (!distanceTof.lastRangeReadValid()) {
      if (lastGoodTofReadMs == 0) {
        pendingMotorStopReason = F("auto: invalid distance sensor");
        return AUTO_DISTANCE_INVALID;
      }
      if ((now - lastGoodTofReadMs) <= tofFailureGraceMs) {
        return Distance;
      }
      handleDistanceSensorFault();
      return AUTO_DISTANCE_INVALID;
    }

    distanceTofConsecutiveFailures = 0;
    distanceTofFaultLatched = false;
    lastGoodTofReadMs = now;

    int raw = (int)(rawMm / 10U);
    if (raw <= 0) raw = AUTO_DISTANCE_MAX_SPEED;
    if (raw > AUTO_DISTANCE_MAX_SPEED) raw = AUTO_DISTANCE_MAX_SPEED;

    int median = pushDistanceSampleAndGetMedian(raw);
    DBG_DISTANCE_SENSOR(F("VL53L0X raw="));
    DBG_DISTANCE_SENSOR(raw);
    DBG_DISTANCE_SENSOR(F(" cm  |  median="));
    DBG_DISTANCE_SENSOR(median);
    DBGLN_DISTANCE_SENSOR(F(" cm"));

    Distance = median;
    return median;
  }

  // Initialize the VL53L0X distance sensor backend.
  void initDistanceSensorHardware() {
    pinMode(pinVL53L0X_XSHUT, OUTPUT);
    digitalWrite(pinVL53L0X_XSHUT, LOW);
    delay(10);
    digitalWrite(pinVL53L0X_XSHUT, HIGH);
    delay(10);
    startDistanceSensorRanging(true);
  }

  // (Re-)configure and start continuous VL53L0X ranging.
  bool startDistanceSensorRanging(bool reinitializeSensor) {
    distanceTof.setTimeout(distanceTofTimeoutMs);
    if (!distanceTof.setAddress(vl53l0xAddress)) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L0X address set failed"));
      return false;
    }

    if (reinitializeSensor && !distanceTof.init()) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L0X not detected on I2C"));
      return false;
    }

    if (!distanceTof.setMeasurementTimingBudget(distanceTofTimingBudgetUs)) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L0X timing budget rejected"));
      return false;
    }

    distanceTof.startContinuous(distanceTofContinuousPeriodMs);
    distanceTofDetected = true;
    distanceTofFaultLatched = false;
    distanceTofConsecutiveFailures = 0;
    lastGoodTofReadMs = 0;
    lastTofReadMs = 0;
    DBGLN_DISTANCE_SENSOR(F("VL53L0X ready"));
    return true;
  }

  // ================================================================================================
  // Tilt sensor
  // ================================================================================================
  void updateAccelerometerSafety() {
    const unsigned long now = millis();
    if (!accelerometerDetected || (now - lastAccelerometerReadMs) < mpu6050ReadEveryMs) return;
    lastAccelerometerReadMs = now;

    int16_t sampleX = 0;
    int16_t sampleY = 0;
    int16_t sampleZ = 0;
    if (!accelerometer.readAcceleration(&sampleX, &sampleY, &sampleZ)) {
      accelerometerDetected = false;
      accelerometerTiltTiming = false;
      DBGLN_ACCELEROMETER(F("MPU-6050 read failed; accelerometer safety disabled"));
      return;
    }

    const int32_t rawX = sampleX;
    const int32_t rawY = sampleY;
    const int32_t rawZ = sampleZ;
    const int32_t uprightZ = (int32_t)rawZ * mpu6050UprightZSign;
    const int64_t horizontalSquared =
      (int64_t)rawX * rawX + (int64_t)rawY * rawY;
    const int64_t uprightZSquared = (int64_t)uprightZ * uprightZ;
    const bool triggerTilt = uprightZ <= 0 ||
      horizontalSquared * 1000LL >=
        uprightZSquared * mpu6050TiltTriggerTanSquaredPermille;
    const bool recovered = uprightZ > 0 &&
      horizontalSquared * 1000LL <
        uprightZSquared * mpu6050TiltRecoveryTanSquaredPermille;

    if ((now - lastAccelerometerDebugMs) >= 1000UL) {
      lastAccelerometerDebugMs = now;
      DBG_ACCELEROMETER(F("MPU-6050 raw X/Y/Z: "));
      DBG_ACCELEROMETER(rawX);
      DBG_ACCELEROMETER(F("/"));
      DBG_ACCELEROMETER(rawY);
      DBG_ACCELEROMETER(F("/"));
      DBGLN_ACCELEROMETER(rawZ);
    }

    if (Speed == 0 || accelerometerCrashLatched) {
      accelerometerPreviousForwardSampleValid = false;
    } else if (MotorDirection == 1 || MotorDirection == 2) {
      const int32_t selectedAxisRaw =
        mpu6050ForwardAxis == 0 ? rawX : rawY;
      const int32_t signedForwardRaw =
        (int32_t)selectedAxisRaw * mpu6050ForwardAxisSign;
      const int32_t crashThresholdRaw =
        ((int32_t)mpu6050CrashDeltaMg * mpu6050AccelLsbPerG) / 1000L;

      if (accelerometerPreviousForwardSampleValid) {
        const int32_t deltaRaw =
          signedForwardRaw - accelerometerPreviousForwardRaw;
        const bool crash = MotorDirection == 1
          ? deltaRaw <= -crashThresholdRaw
          : deltaRaw >= crashThresholdRaw;

        if (crash) {
          accelerometerCrashLatched = true;
          stopAndResetStepSelection();
          exitAutoDistanceMode(false);
          cancelJog();
          SetRGBColor(RgbColor::Red);
          sirenActive = true;
          sirenTimer = millis();
          sirenStartMs = sirenTimer;
          DBG_ACCELEROMETER(F("MPU-6050 crash: delta="));
          DBG_ACCELEROMETER(deltaRaw);
          DBG_ACCELEROMETER(F(", threshold="));
          DBGLN_ACCELEROMETER(crashThresholdRaw);
        }
      }

      accelerometerPreviousForwardRaw = signedForwardRaw;
      accelerometerPreviousForwardSampleValid = !accelerometerCrashLatched;
    } else {
      accelerometerPreviousForwardSampleValid = false;
    }

    if (triggerTilt) {
      if (!accelerometerTiltStopLatched) {
        if (!accelerometerTiltTiming) {
          accelerometerTiltTiming = true;
          accelerometerTiltStartedMs = now;
        } else if ((now - accelerometerTiltStartedMs) >= TILT_STABLE_MS) {
          stopAndResetStepSelection();
          exitAutoDistanceMode(false);
          cancelJog();
          accelerometerTiltStopLatched = true;
          sirenActive = false;
          stopMelody();
          noTone(pinBuzzer);
          SetRGBColor(RgbColor::Red);
          playPattern(pattern_tiltBeep, true);
          DBGLN_ACCELEROMETER(F("MPU-6050 tilt: ACTIVE -> emergency stop"));
        }
      }
    } else if (recovered) {
      accelerometerTiltTiming = false;
      if (accelerometerTiltStopLatched) {
        accelerometerTiltStopLatched = false;
        refreshDriveLights();
        DBGLN_ACCELEROMETER(F("MPU-6050 tilt: IDLE -> clear latch, restore LEDs"));
      }
    } else {
      accelerometerTiltTiming = false;
    }
  }

  // Debounce tilt input and latch emergency stop.
  // "Debouncing" means waiting for a signal to settle before trusting it: a mechanical tilt switch
  // can flicker rapidly between HIGH/LOW for a few milliseconds while it's physically moving, so
  // reacting to every raw reading would cause false triggers. This function tracks the last raw
  // reading (tiltLastRead) and when it changes (tiltEdgeAt), then only accepts it as a real,
  // "stable" state change once it has held steady for TILT_STABLE_MS - and then enforces a further
  // "quiet period" (tiltQuietUntil) afterward before it will consider yet another change, to avoid
  // rapid re-triggering right at the edge of stability.
  // This alarm bypasses the battery-restriction sound gate (bypassBatteryGate = true), the same way
  // the denial beeps elsewhere in this project do: a physical tip-over is a safety event and must
  // stay audible even during battery Warning/Shutdown restrictions.
  void updateTiltSensor() {
    unsigned long now = millis();
    int reading = digitalRead(pinTiltSensor);

    if (reading != tiltLastRead) {
      tiltLastRead = reading;
      tiltEdgeAt = now;
    }

    if ((long)(now - tiltQuietUntil) >= 0 && reading != tiltStableState && (now - tiltEdgeAt) >= TILT_STABLE_MS) {

      tiltStableState = reading;
      tiltQuietUntil = now + TILT_QUIET_MS;

      if (tiltStableState == LOW) {
        DBGLN_TILT_SENSOR(F("TILT: ACTIVE -> emergency stop"));
        if (!tiltStopLatched) {
          stopAndResetStepSelection();
          exitAutoDistanceMode(false);  // optional: exit AUTO
          tiltStopLatched = true;
        }
        sirenActive = false;   // silence siren first so the tilt beep is audible
        stopMelody();
        noTone(pinBuzzer);
        SetRGBColor(RgbColor::Red);
        // Bypass the battery-restriction sound gate: this is a safety alarm, not a user sound.
        playPattern(pattern_tiltBeep, true);
      } else {
        DBGLN_TILT_SENSOR(F("TILT: IDLE -> clear latch, restore LEDs"));
        tiltStopLatched = false;
        refreshDriveLights();
      }
    }
    updateAccelerometerSafety();
  }
