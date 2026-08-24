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

  // VL53L1X distance-sensor helper.
  // Like the TCS34725 helper above, this is a hand-written register-level driver, here for the
  // VL53L1X time-of-flight distance sensor, instead of using a ready-made library such as
  // Pololu's "VL53L1X" Arduino library or ST's official Ultra Lite Driver (ULD). Those include
  // calibration routines, multi-zone ROI scanning, distance thresholds and interrupt modes this
  // train doesn't need, all of which cost flash and RAM the sketch cannot spare alongside the
  // colour sensor, LED expander, accelerometer and IR remote code.
  //
  // The VL53L1X is NOT protocol compatible with the VL53L0X this train used previously: it uses
  // 16-bit register addresses instead of 8-bit ones. The old driver is kept for reference in
  // 90-legacy-vl53l0x.ino, switched out of the build.
  //
  // Almost every literal below is an undocumented "magic" value transcribed from ST's ULD; ST has
  // never published a full VL53L1X register map. Do not "tidy" them.
  //
  // If you wanted to use the Pololu library instead, the equivalent calls would be:
  //   VL53L1X sensor;                        // ~= this whole struct
  //   sensor.init();                          // ~= init() below
  //   sensor.setDistanceMode(VL53L1X::Short); // ~= setDistanceModeShort()
  //   sensor.setROISize(4, 4);                // ~= setRoi()
  //   sensor.startContinuous(50);             // ~= startContinuous()
  //   sensor.read();                          // ~= readRangeContinuousMillimeters()

  // ST's default configuration block, registers 0x2D..0x87 inclusive (91 bytes).
  // "PROGMEM" keeps this table in flash instead of copying it into SRAM at startup, which matters
  // on a Nano with 2 KB of RAM; pgm_read_byte() below is how you read it back.
  const uint8_t kVl53l1xDefaultConfig[] PROGMEM = {
    0x00, 0x01, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x05, 0x00,
    0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0xC7, 0xFF,
    0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00
  };

  struct TrainDistanceSensorVL53L1X {
    static const uint8_t DefaultAddress = 0x29;

    // The one-off calibration inside init() ranges using the default configuration block, which is
    // still in long distance mode with a ~100 ms timing budget at that point. The normal 50 ms
    // measurement timeout would abort a perfectly healthy sensor, so init() raises the timeout to
    // this value for the duration and puts it back afterwards.
    static const uint16_t InitTimeoutMs = 200;

    uint8_t address = DefaultAddress;
    uint16_t ioTimeoutMs = 0;
    bool didTimeout = false;
    bool lastReadValid = false;
    unsigned long timeoutStartMs = 0;

    void setTimeout(uint16_t timeoutMs) {
      ioTimeoutMs = timeoutMs;
    }

    // Assign the runtime I2C address.
    //
    // This is deliberately a WRITE-ONLY transaction and it matters: the TCS34725 colour sensor also
    // answers at 0x29, so a *read* there would have both chips driving the bus and return garbage.
    // A write is safe because on the wire this is [0x00, 0x01, 0x2A], and the TCS34725 discards it
    // since its own protocol requires the command bit 0x80 on the first byte (see
    // TrainColorSensorTCS34725::writeRegister above), which 0x00 does not have.
    //
    // The value written is the plain 7-bit address. (ST's ULD writes "new_address >> 1" only
    // because its device handle stores the 8-bit form.)
    bool setAddress(uint8_t newAddress) {
      if (!writeRegAt(address, 0x0001, (uint8_t)(newAddress & 0x7F))) {
        if (address == DefaultAddress) return false;
        if (!writeRegAt(DefaultAddress, 0x0001, (uint8_t)(newAddress & 0x7F))) return false;
      }
      address = newAddress;
      return true;
    }

    // Wait until the sensor firmware has finished booting.
    //
    // Only call this once the sensor is alone on its own address. It doubles as the check that
    // setAddress() actually worked: if the address write silently failed, the TCS34725 would have
    // acknowledged it anyway, and this poll at the new address is what catches that.
    // The sensor may not acknowledge at all while booting, so a failed read is retried rather than
    // treated as fatal.
    bool waitForBoot() {
      startTimeout();
      for (;;) {
        uint8_t state = 0;
        if (readRegs(0x00E5, &state, 1) && (state & 0x01) != 0) return true;
        if (hasTimedOut()) {
          didTimeout = true;
          return false;
        }
      }
    }

    // Identify and configure the sensor.
    // "io2v8" is accepted for call-site compatibility with the old VL53L1X driver and ignored: the
    // 2.8 V I/O setting is already part of the default configuration block (register 0x2E).
    //
    // Note there is deliberately NO software reset here. Writing SOFT_RESET (0x0000), which several
    // VL53L1X libraries do as their first step, would return the sensor to address 0x29 on top of
    // the TCS34725 and quietly recreate the address collision.
    bool init(bool io2v8 = true) {
      (void)io2v8;
      didTimeout = false;
      lastReadValid = false;

      if (readReg16(0x010F) != 0xEACC) return false;

      for (uint8_t i = 0; i < sizeof(kVl53l1xDefaultConfig); ++i) {
        if (!writeReg((uint16_t)(0x002D + i), pgm_read_byte(&kVl53l1xDefaultConfig[i]))) {
          return false;
        }
      }

      // Required one-off calibration. This is NOT part of the configuration block, and skipping it
      // leaves a sensor that acknowledges on I2C and passes the model-ID check while never
      // returning usable distances.
      const uint16_t savedTimeout = ioTimeoutMs;
      ioTimeoutMs = InitTimeoutMs;
      const bool calibrated = writeReg(0x0087, 0x40)   // start ranging
                              && waitForDataReady()
                              && clearInterrupt()
                              && writeReg(0x0087, 0x00) // stop ranging
                              && writeReg(0x0008, 0x09)
                              && writeReg(0x000B, 0x00);
      ioTimeoutMs = savedTimeout;
      if (!calibrated) return false;

      return setDistanceModeShort();
    }

    // Select short distance mode.
    // The default configuration block is long mode, so this is an explicit change. Short mode is
    // wanted here: it is far more robust against ambient light, and its roughly 1.3 m ceiling still
    // covers AUTO_DISTANCE_MAX_SPEED (50 cm), the longest distance the auto-drive logic uses.
    bool setDistanceModeShort() {
      if (!writeReg(0x004B, 0x14)) return false;
      if (!writeReg(0x0060, 0x07)) return false;
      if (!writeReg(0x0063, 0x05)) return false;
      if (!writeReg(0x0069, 0x38)) return false;
      if (!writeReg16(0x0078, 0x0705)) return false;
      return writeReg16(0x007A, 0x0606);
    }

    // Apply the measurement timing budget.
    // The VL53L1X only accepts a handful of discrete budgets, so an unsupported request snaps to
    // the nearest supported one rather than failing; that keeps a mistyped config.h from disabling
    // the distance sensor entirely. These are the short-mode register pairs. (The 33 ms and 20 ms
    // entries genuinely share the same B value - that is not a copy-paste slip.)
    bool setMeasurementTimingBudget(uint32_t budgetUs) {
      const uint16_t ms = (uint16_t)(budgetUs / 1000UL);
      uint16_t macropA;
      uint16_t macropB;
      if (ms < 18)       { macropA = 0x001D; macropB = 0x0027; }  // 15 ms
      else if (ms < 27)  { macropA = 0x0051; macropB = 0x006E; }  // 20 ms
      else if (ms < 42)  { macropA = 0x00D6; macropB = 0x006E; }  // 33 ms
      else if (ms < 75)  { macropA = 0x01AE; macropB = 0x01E8; }  // 50 ms
      else if (ms < 150) { macropA = 0x02E1; macropB = 0x0388; }  // 100 ms
      else if (ms < 350) { macropA = 0x03E1; macropB = 0x0496; }  // 200 ms
      else               { macropA = 0x0591; macropB = 0x05C1; }  // 500 ms

      if (!writeReg16(0x005E, macropA)) return false;
      return writeReg16(0x0061, macropB);
    }

    // Set the detection cone (region of interest) on the sensor's 16x16 SPAD array.
    // A smaller window narrows the cone, which is what keeps the floor and passing scenery from
    // registering as obstacles, at the cost of maximum range.
    // Width and height are clamped to the 4..16 the hardware accepts. The centre is NOT validated:
    // a centre far from 199 combined with a small window can push the ROI off the array, which the
    // sensor reports as range status 13 and which shows up here as readings that are never valid.
    bool setRoi(uint8_t width, uint8_t height, uint8_t centerSpad) {
      if (width < 4) width = 4;
      if (width > 16) width = 16;
      if (height < 4) height = 4;
      if (height > 16) height = 16;

      if (!writeReg(0x0080, (uint8_t)(((height - 1) << 4) | (width - 1)))) return false;
      return writeReg(0x007F, centerSpad);
    }

    // Begin continuous ranging.
    // The intermeasurement period is derived from the sensor's own oscillator calibration value.
    // ST's formula multiplies by 1.075; that is done here as "* 43 / 40" so the sketch does not
    // drag AVR floating-point support into flash for a single multiply. The widest intermediate is
    // 1023 * 50 * 43 = 2199450, comfortably inside a uint32_t.
    void startContinuous(uint32_t periodMs = 0) {
      didTimeout = false;
      lastReadValid = false;

      if (periodMs != 0) {
        uint8_t osc[2] = { 0, 0 };
        if (readRegs(0x00DE, osc, sizeof(osc))) {
          const uint32_t clockPll = (uint32_t)((((uint16_t)osc[0] << 8) | osc[1]) & 0x03FF);
          writeReg32(0x006C, clockPll * periodMs * 43UL / 40UL);
        }
      }

      writeReg(0x0087, 0x40);
    }

    // Read the latest distance in millimetres. Returns 65535 and leaves lastRangeReadValid() false
    // when the reading is unusable, matching what the previous driver did.
    uint16_t readRangeContinuousMillimeters() {
      lastReadValid = false;
      if (!waitForDataReady()) return 65535;

      uint8_t status = 0;
      uint8_t range[2] = { 0, 0 };
      const bool ok = readRegs(0x0089, &status, 1) && readRegs(0x0096, range, sizeof(range));

      // Mandatory: without clearing the interrupt the sensor never flags a new measurement and
      // ranging stalls after the first reading.
      clearInterrupt();
      if (!ok) return 65535;

      // The low five bits are an internal raw status, not the public status code. ST maps raw 9 to
      // "valid", so testing this register against 0 would reject every good measurement. The train
      // only needs valid-versus-invalid, so the full 24-entry translation table is omitted.
      if ((status & 0x1F) != 9) return 65535;

      lastReadValid = true;
      return ((uint16_t)range[0] << 8) | range[1];
    }

    bool timeoutOccurred() {
      bool timedOut = didTimeout;
      didTimeout = false;
      return timedOut;
    }

    bool lastRangeReadValid() const {
      return lastReadValid;
    }

    #if DEBUG_DISTANCE_SENSOR
    void logStartupI2cState(const __FlashStringHelper* checkpoint) {
      DBGLN_DISTANCE_SENSOR(checkpoint);
      logI2cAddress(mcp23008Address);
      logI2cAddress(DefaultAddress);
      logI2cAddress(vl53l1xAddress);
    }

    void logModelId() {
      DBG_DISTANCE_SENSOR(F("VL53L1X model ID at 0x"));
      printHexByte(address);
      DBG_DISTANCE_SENSOR(F(": 0x"));

      uint8_t id[2] = { 0, 0 };
      if (!readRegs(0x010F, id, sizeof(id))) {
        DBGLN_DISTANCE_SENSOR(F("read failed"));
        return;
      }

      printHexByte(id[0]);
      printHexByte(id[1]);
      DBGLN_DISTANCE_SENSOR(F(""));
    }
    #endif

   private:
    #if DEBUG_DISTANCE_SENSOR
    // Note that a probe of 0x29 is ambiguous by nature: the TCS34725 answers there too, so an ACK
    // does not prove the distance sensor is present.
    void logI2cAddress(uint8_t targetAddress) {
      DBG_DISTANCE_SENSOR(F("I2C 0x"));
      printHexByte(targetAddress);
      DBGLN_DISTANCE_SENSOR(probeAddress(targetAddress) ? F(": ACK") : F(": no ACK"));
    }

    bool probeAddress(uint8_t targetAddress) {
      Wire.beginTransmission(targetAddress);
      return Wire.endTransmission() == 0;
    }

    void printHexByte(uint8_t value) {
      if (value < 0x10) DBG_DISTANCE_SENSOR(F("0"));
      DBG_DISTANCE_SENSOR(value, HEX);
    }
    #endif

    bool clearInterrupt() {
      return writeReg(0x0086, 0x01);
    }

    // Poll until the sensor says a measurement is ready.
    // Watch the "!" in the polarity line: dropping it inverts the test, and the loop then either
    // spins until it times out or returns stale data.
    bool waitForDataReady() {
      uint8_t mux = 0;
      if (!readRegs(0x0030, &mux, 1)) {
        didTimeout = true;
        return false;
      }
      const uint8_t polarity = (uint8_t)(!((mux >> 4) & 0x01));

      startTimeout();
      for (;;) {
        uint8_t status = 0;
        if (!readRegs(0x0031, &status, 1)) {
          didTimeout = true;
          return false;
        }
        if ((status & 0x01) == polarity) return true;
        if (hasTimedOut()) {
          didTimeout = true;
          return false;
        }
      }
    }

    void startTimeout() {
      timeoutStartMs = millis();
    }

    // Falls back to a sane cap when no timeout has been configured, so no poll above can spin
    // forever on a dead bus.
    bool hasTimedOut() const {
      const uint16_t limit = (ioTimeoutMs > 0) ? ioTimeoutMs : 500;
      return (uint16_t)(millis() - timeoutStartMs) > limit;
    }

    // --- 16-bit-register access primitives -------------------------------------------------
    // Every VL53L1X transaction sends the register address as two bytes, high byte first. This is
    // the fundamental difference from the VL53L1X, whose registers were single bytes.
    bool writeRegAt(uint8_t targetAddress, uint16_t reg, uint8_t value) {
      Wire.beginTransmission(targetAddress);
      Wire.write((uint8_t)(reg >> 8));
      Wire.write((uint8_t)(reg & 0xFF));
      Wire.write(value);
      return Wire.endTransmission() == 0;
    }

    bool writeReg(uint16_t reg, uint8_t value) {
      return writeRegAt(address, reg, value);
    }

    bool writeReg16(uint16_t reg, uint16_t value) {
      Wire.beginTransmission(address);
      Wire.write((uint8_t)(reg >> 8));
      Wire.write((uint8_t)(reg & 0xFF));
      Wire.write((uint8_t)(value >> 8));
      Wire.write((uint8_t)(value & 0xFF));
      return Wire.endTransmission() == 0;
    }

    bool writeReg32(uint16_t reg, uint32_t value) {
      Wire.beginTransmission(address);
      Wire.write((uint8_t)(reg >> 8));
      Wire.write((uint8_t)(reg & 0xFF));
      for (int8_t shift = 24; shift >= 0; shift -= 8) {
        Wire.write((uint8_t)(value >> shift));
      }
      return Wire.endTransmission() == 0;
    }

    bool readRegsAt(uint8_t targetAddress, uint16_t reg, uint8_t* buffer, uint8_t length) {
      Wire.beginTransmission(targetAddress);
      Wire.write((uint8_t)(reg >> 8));
      Wire.write((uint8_t)(reg & 0xFF));
      // endTransmission(false) issues a repeated start, holding the bus so no other device can cut
      // in between "here's the register I want" and reading it back.
      if (Wire.endTransmission(false) != 0) return false;

      if (Wire.requestFrom((int)targetAddress, (int)length) != length) return false;
      for (uint8_t i = 0; i < length; ++i) {
        buffer[i] = (uint8_t)Wire.read();
      }
      return true;
    }

    bool readRegs(uint16_t reg, uint8_t* buffer, uint8_t length) {
      return readRegsAt(address, reg, buffer, length);
    }

    uint16_t readReg16(uint16_t reg) {
      uint8_t data[2] = { 0, 0 };
      if (!readRegs(reg, data, sizeof(data))) {
        didTimeout = true;
        return 0;
      }
      return ((uint16_t)data[0] << 8) | data[1];
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
  uint8_t lastTrackMarkerClass = 255;

  // Distance-sensor runtime state and filtering.
  uint8_t Distance = 0;
  uint8_t distanceBuffer[AUTO_SAMPLES_FOR_MEDIAN];
  uint8_t bufferIndex = 0;
  bool bufferFilled = false;
  TrainDistanceSensorVL53L1X distanceTof;
  bool distanceTofDetected = false;
  bool distanceTofFaultLatched = false;
  uint8_t distanceTofConsecutiveFailures = 0;
  unsigned long lastTofReadMs = 0;
  unsigned long lastGoodTofReadMs = 0;
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

    // Walk every known marker color in the PROGMEM table from config.h and
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
    DBGLN_DISTANCE_SENSOR(F("VL53L1X fault: auto-distance mode disabled until sensor is reinitialized"));
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
    DBG_DISTANCE_SENSOR(F("VL53L1X raw="));
    DBG_DISTANCE_SENSOR(raw);
    DBG_DISTANCE_SENSOR(F(" cm  |  median="));
    DBG_DISTANCE_SENSOR(median);
    DBGLN_DISTANCE_SENSOR(F(" cm"));

    Distance = median;
    return median;
  }

  // Initialize the VL53L1X distance sensor backend.
  void initDistanceSensorHardware() {
    pinMode(pinVL53L1X_XSHUT, OUTPUT);
    digitalWrite(pinVL53L1X_XSHUT, LOW);
    delay(10);
    #if DEBUG_DISTANCE_SENSOR
    distanceTof.logStartupI2cState(F("VL53L1X startup: XSHUT low"));
    #endif
    digitalWrite(pinVL53L1X_XSHUT, HIGH);
    // The datasheet allows roughly 1.2 ms for the firmware to boot, so 10 ms is a comfortable
    // margin. Do not trim it: the sensor is sitting on 0x29 alongside the TCS34725 right now, so
    // this delay is the only thing guaranteeing it is awake before the address write below, and
    // reading a status register here to check would be ambiguous with two chips on that address.
    delay(10);
    startDistanceSensorRanging(true);
  }

  // (Re-)configure and start continuous VL53L1X ranging.
  bool startDistanceSensorRanging(bool reinitializeSensor) {
    distanceTof.setTimeout(distanceTofTimeoutMs);

    // Ordering below is the correctness rule for the shared bus: move the sensor off 0x29 first,
    // then confirm it at its own address, and only then configure it. Never configure it while it
    // still shares 0x29 with the colour sensor.
    if (!distanceTof.setAddress(vl53l1xAddress)) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L1X address set failed"));
      return false;
    }

    // The TCS34725 would have acknowledged the address write regardless, so this boot poll at the
    // new address is what actually proves the reassignment took effect.
    if (!distanceTof.waitForBoot()) {
      distanceTofDetected = false;
      #if DEBUG_DISTANCE_SENSOR
      distanceTof.logStartupI2cState(F("VL53L1X startup: boot wait failed"));
      #endif
      DBGLN_DISTANCE_SENSOR(F("VL53L1X did not boot at its assigned address"));
      return false;
    }

    #if DEBUG_DISTANCE_SENSOR
    distanceTof.logStartupI2cState(F("VL53L1X startup: after address assignment"));
    distanceTof.logModelId();
    #endif

    if (reinitializeSensor && !distanceTof.init()) {
      distanceTofDetected = false;
      #if DEBUG_DISTANCE_SENSOR
      distanceTof.logStartupI2cState(F("VL53L1X startup: initialization failure"));
      distanceTof.logModelId();
      #endif
      DBGLN_DISTANCE_SENSOR(F("VL53L1X not detected on I2C"));
      return false;
    }

    if (!distanceTof.setMeasurementTimingBudget(distanceTofTimingBudgetUs)) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L1X timing budget rejected"));
      return false;
    }

    if (!distanceTof.setRoi(distanceTofRoiWidth, distanceTofRoiHeight, distanceTofRoiCenterSpad)) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L1X detection cone rejected"));
      return false;
    }

    distanceTof.startContinuous(distanceTofContinuousPeriodMs);
    distanceTofDetected = true;
    distanceTofFaultLatched = false;
    distanceTofConsecutiveFailures = 0;
    lastGoodTofReadMs = 0;
    lastTofReadMs = 0;
    DBGLN_DISTANCE_SENSOR(F("VL53L1X ready"));
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

      if (tiltStableState == HIGH) {
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
