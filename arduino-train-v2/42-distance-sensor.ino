  // ================================================================================================
  // File description
  // ================================================================================================
  // VL53L1X time-of-flight distance-sensor driver, median filtering, and obstacle-distance readout.
  // Split out of the former 40-sensors.ino; the color sensor lives in 41-color-sensor.ino and the
  // accelerometer in 43-accelerometer.ino.

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
