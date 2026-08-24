// ================================================================================================
// File description
// ================================================================================================
// Retired VL53L0X distance-sensor driver, kept for reference only.
//
// The train now uses a VL53L1X time-of-flight sensor, driven by TrainDistanceSensorVL53L1X in
// 42-distance-sensor.ino. The two chips are not protocol compatible: the VL53L0X uses 8-bit register
// addresses while the VL53L1X uses 16-bit ones, so this driver cannot talk to the new hardware.
//
// This tab is excluded from compilation by ENABLE_VL53L0X_LEGACY_DRIVER in config.h. While that
// switch is 0 the preprocessor strips everything below, so this file costs zero flash and zero
// SRAM. Turning it back on only makes the struct compile again; nothing references it, and the
// call sites in 42-distance-sensor.ino would still have to be reconnected by hand.

#include "config.h"
#if ENABLE_VL53L0X_LEGACY_DRIVER

  // The live code renamed this constant to vl53l1xAddress, so this snapshot carries its own copy
  // to stay self-contained. It only exists while the legacy switch is on.
  constexpr uint8_t vl53l0xAddress = 0x2A;

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

    #if DEBUG_DISTANCE_SENSOR
    void logStartupI2cState(const __FlashStringHelper* checkpoint) {
      DBGLN_DISTANCE_SENSOR(checkpoint);
      logI2cAddress(0x20);
      logI2cAddress(DefaultAddress);
      logI2cAddress(vl53l0xAddress);
    }

    void logModelId() {
      uint8_t modelId = 0;
      DBG_DISTANCE_SENSOR(F("VL53L0X model ID at 0x"));
      printHexByte(address);
      DBG_DISTANCE_SENSOR(F(": "));

      if (!readModelIdAt(address, &modelId)) {
        DBGLN_DISTANCE_SENSOR(F("read failed"));
        return;
      }

      DBG_DISTANCE_SENSOR(F("0x"));
      printHexByte(modelId);
      DBGLN_DISTANCE_SENSOR(F(""));
    }
    #endif

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
    #if DEBUG_DISTANCE_SENSOR
    void logI2cAddress(uint8_t targetAddress) {
      DBG_DISTANCE_SENSOR(F("I2C 0x"));
      printHexByte(targetAddress);
      DBGLN_DISTANCE_SENSOR(probeAddress(targetAddress) ? F(": ACK") : F(": no ACK"));
    }

    bool probeAddress(uint8_t targetAddress) {
      Wire.beginTransmission(targetAddress);
      return Wire.endTransmission() == 0;
    }

    bool readModelIdAt(uint8_t targetAddress, uint8_t* modelId) {
      Wire.beginTransmission(targetAddress);
      Wire.write(RegisterIdentificationModelId);
      if (Wire.endTransmission(false) != 0) return false;
      if (Wire.requestFrom((int)targetAddress, 1) != 1) return false;
      *modelId = (uint8_t)Wire.read();
      return true;
    }

    void printHexByte(uint8_t value) {
      if (value < 0x10) DBG_DISTANCE_SENSOR(F("0"));
      DBG_DISTANCE_SENSOR(value, HEX);
    }
    #endif

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

#endif

