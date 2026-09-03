#if USE_VL53L1X_DISTANCE_SENSOR
// ================================================================================================
// VL53L1X distance-sensor module
// ================================================================================================

  // VL53L1X distance-sensor helper.
  // Like the TCS34725 helper above, this is a hand-written register-level driver, here for the
  // VL53L1X time-of-flight distance sensor, instead of using a ready-made library such as
  // Pololu's "VL53L1X" Arduino library or ST's official Ultra Lite Driver (ULD). Those include
  // calibration routines, multi-zone ROI scanning, distance thresholds and interrupt modes this
  // train doesn't need, all of which cost flash and RAM the sketch cannot spare alongside the
  // colour sensor, LED expander, accelerometer and IR remote code.
  //
  // This backend uses 16-bit register addresses.
  //
  // Almost every literal below is an undocumented "magic" value transcribed from ST's ULD; ST has
  // never published a full VL53L1X register map. Do not "tidy" them.
  //
  // If you wanted to use the Pololu library instead, the equivalent calls would be:
  //   VL53L1X sensor;                        // ~= this whole struct
  //   sensor.init();                          // ~= init() below
  //   sensor.setDistanceMode(VL53L1X::Short); // ~= setDistanceModeShort()
  //   sensor.setROISize(8, 4);                // ~= setRoi()
  //   sensor.startContinuous(50);             // ~= startContinuous()
  //   sensor.stopContinuous();                // ~= stopContinuous()
  //   sensor.dataReady();                     // ~= sampleReady()
  //   sensor.read(false);                     // ~= readRangeContinuousMillimeters() (non-blocking)

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
    // Cached data-ready polarity, filled in once by init(). The sensor's GPIO_HV_MUX__CTRL register
    // (0x0030) decides whether "measurement ready" reads as bit value 0 or 1 in register 0x0031.
    // Reading it once here (instead of on every poll, as ST's ULD does) halves the I2C traffic of
    // every readiness check. It only changes when the configuration block is rewritten, i.e. in
    // init(), so caching is safe.
    uint8_t dataReadyPolarity = 1;

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

      // Cache the data-ready polarity now that the configuration block (which fixes register
      // 0x0030) has been written. sampleReady() and waitForDataReady() below both depend on this.
      // Watch the "!" in the polarity line: dropping it inverts the test, and every readiness
      // check afterwards would either always time out or return stale data.
      {
        uint8_t mux = 0;
        if (!readRegs(0x0030, &mux, 1)) return false;
        dataReadyPolarity = (uint8_t)(!((mux >> 4) & 0x01));
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

    // Stop continuous ranging (Pololu library equivalent: sensor.stopContinuous()).
    // Used when auto-distance mode is switched off: the sensor keeps its configuration and address
    // and can be restarted instantly with startContinuous(), but stops measuring, which saves
    // power (~20 mA during ranging) and keeps the I2C bus quiet for the other devices.
    bool stopContinuous() {
      lastReadValid = false;
      return writeReg(0x0087, 0x00);
    }

    // Non-blocking check whether a new measurement is waiting to be read
    // (Pololu library equivalent: sensor.dataReady()).
    // Costs a single 1-byte register read (~50 us at 400 kHz) and returns immediately, unlike the
    // old blocking wait that could busy-spin for up to 50 ms inside loop(). The caller polls this
    // each loop pass and only calls readRangeContinuousMillimeters() once it returns true.
    bool sampleReady() {
      uint8_t status = 0;
      if (!readRegs(0x0031, &status, 1)) return false;  // I2C error counts as "nothing to read"
      return (status & 0x01) == dataReadyPolarity;
    }

    // Read the latest distance in millimetres. Returns 65535 and leaves lastRangeReadValid() false
    // when the reading is unusable, matching what the previous driver did.
    // (Pololu library equivalent: sensor.read(false) - the non-blocking form.)
    // IMPORTANT: only call this after sampleReady() has returned true; this function no longer
    // waits for a measurement itself, so calling it early would read stale or half-updated data.
    uint16_t readRangeContinuousMillimeters() {
      lastReadValid = false;

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

    bool lastRangeReadValid() const {
      return lastReadValid;
    }

    #if DEBUG_DISTANCE_SENSOR
    void logStartupI2cState(const __FlashStringHelper* checkpoint) {
      DBGLN_DISTANCE_SENSOR(checkpoint);
      logI2cAddress(mcp23008Address);
      logI2cAddress(DefaultAddress);
      logI2cAddress(distanceSensorAddress);
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
    // This BLOCKING wait is only used during the one-off calibration inside init(), where waiting
    // is correct and simple. Normal operation uses the non-blocking sampleReady() instead so
    // loop() is never stalled. Relies on dataReadyPolarity already being cached by init().
    bool waitForDataReady() {
      startTimeout();
      for (;;) {
        uint8_t status = 0;
        if (!readRegs(0x0031, &status, 1)) {
          didTimeout = true;
          return false;
        }
        if ((status & 0x01) == dataReadyPolarity) return true;
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
    // this driver uses two-byte register addresses, high byte first.
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

  TrainDistanceSensorVL53L1X distanceTof;

  void logDistanceSensorStatus(const __FlashStringHelper* status) {
    DBG_DISTANCE_SENSOR(F("VL53L1X"));
    DBGLN_DISTANCE_SENSOR(status);
  }
  // Distance-sensor runtime state and filtering.
  uint8_t Distance = 0;
  uint8_t distanceBuffer[AUTO_SAMPLES_FOR_MEDIAN];
  uint8_t bufferIndex = 0;
  bool bufferFilled = false;
  bool distanceTofDetected = false;
  bool distanceTofFaultLatched = false;
  unsigned long lastTofReadMs = 0;
  unsigned long lastGoodTofReadMs = 0;
  // When continuous ranging last (re)started. Used by the startup-grace check in
  // getDistanceReading(): if the sensor never produces a single valid reading within
  // tofStartupGraceMs of this moment, a fault is latched instead of waiting forever.
  unsigned long tofRangingStartedMs = 0;
  // Whether continuous ranging is currently running. Ranging is active only while auto-distance
  // mode is on (see setDistanceSensorRangingActive()); the sensor stays initialized but idle the
  // rest of the time to save power and I2C traffic.
  bool tofRangingActive = false;

  // Forget all buffered distance samples so the median filter starts fresh.
  // Called whenever ranging (re)starts: samples taken minutes ago (before auto mode was last
  // switched off) must never influence the first drive decision of a new auto session.
  void resetDistanceFilter() {
    bufferIndex = 0;
    bufferFilled = false;
    Distance = 0;
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
    distanceTof.stopContinuous();
    distanceTofDetected = false;
    tofRangingActive = false;
    Distance = 0;
    pendingMotorStopReason = F("auto: no response from distance sensor");
    exitAutoDistanceMode();
    stopAndResetStepSelection();
    SetRGBColor(RgbColor::Red);
    playPattern(pattern_batteryWarn);
    logDistanceSensorStatus(F(" fault: auto-distance mode disabled until sensor is reinitialized"));
    #if ENABLE_EEPROM_LOGGING
    logBatteryEvent(EEPROM_EVENT_TOF_FAULT, batteryVoltage);
    #endif
  }

  // Return the latest filtered distance in cm.
  // Non-blocking by design: instead of waiting inside the driver for a measurement (which used to
  // stall loop() for up to 50 ms), this asks the sensor "is a new sample ready?" (one fast I2C
  // read) and simply returns the previous filtered value when nothing new has arrived yet.
  // Failure policy, from mildest to most severe:
  //   1. No new sample yet, last good reading is recent (within tofFailureGraceMs) -> keep using it.
  //   2. No good reading for longer than the grace period -> latch a fault and stop the train.
  //   3. Never had a good reading and tofStartupGraceMs has passed since ranging started (sensor
  //      present on I2C but not measuring, e.g. optically blocked or miswired) -> latch a fault.
  int getDistanceReading() {
    if (!distanceTofDetected || !tofRangingActive) {
      handleDistanceSensorFault();
      return AUTO_DISTANCE_INVALID;
    }

    unsigned long now = millis();
    if (now - lastTofReadMs < tofReadEveryMs) {
      // Not time for a new sensor read yet; serve the cached value. An empty filter (fresh ranging
      // start, no samples yet) reports "invalid" so auto mode waits instead of driving blind.
      if (!bufferFilled && bufferIndex == 0) return AUTO_DISTANCE_INVALID;
      return Distance;
    }

    if (!distanceTof.sampleReady()) {
      // No new measurement waiting. This is normal for a poll or two (the sensor's 50 ms
      // measurement cycle drifts against our 50 ms read cycle), so only escalate when it persists.
      if (lastGoodTofReadMs == 0) {
        // Still waiting for the very first valid reading since ranging started.
        if (now - tofRangingStartedMs > tofStartupGraceMs) {
          handleDistanceSensorFault();  // sensor never delivered anything usable
          return AUTO_DISTANCE_INVALID;
        }
        pendingMotorStopReason = F("auto: invalid distance sensor");
        return AUTO_DISTANCE_INVALID;
      }
      if ((now - lastGoodTofReadMs) <= tofFailureGraceMs) {
        return Distance;  // recent good value still trustworthy
      }
      handleDistanceSensorFault();
      return AUTO_DISTANCE_INVALID;
    }
    lastTofReadMs = now;

    uint16_t rawMm = distanceTof.readRangeContinuousMillimeters();
    if (!distanceTof.lastRangeReadValid()) {
      // A sample WAS ready but its range status marked it unusable (target too weak/ambiguous).
      // Apply the same escalation ladder as the "no sample" branch above.
      if (lastGoodTofReadMs == 0) {
        if (now - tofRangingStartedMs > tofStartupGraceMs) {
          handleDistanceSensorFault();
          return AUTO_DISTANCE_INVALID;
        }
        pendingMotorStopReason = F("auto: invalid distance sensor");
        return AUTO_DISTANCE_INVALID;
      }
      if ((now - lastGoodTofReadMs) <= tofFailureGraceMs) {
        return Distance;
      }
      handleDistanceSensorFault();
      return AUTO_DISTANCE_INVALID;
    }

    distanceTofFaultLatched = false;
    lastGoodTofReadMs = now;

    // Convert mm -> cm, then clamp into the 1..AUTO_DISTANCE_MAX_SPEED range the auto logic uses.
    // SAFETY: a point-blank reading (0-9 mm becomes 0 cm) must clamp to the CLOSEST value (1 cm =
    // obstacle touching the sensor), never to the far end - an earlier version mapped it to
    // AUTO_DISTANCE_MAX_SPEED, which told the train "50 cm of clear track" while it was pressed
    // against an obstacle.
    int raw = (int)(rawMm / 10U);
    if (raw < 1) raw = 1;
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

  // Initialize the VL53L1X distance sensor.
  void initDistanceSensorHardware() {
    pinMode(pinDistanceSensorXSHUT, OUTPUT);
    digitalWrite(pinDistanceSensorXSHUT, LOW);
    delay(10);
    #if DEBUG_DISTANCE_SENSOR
    distanceTof.logStartupI2cState(F("VL53L1X startup: XSHUT low"));
    #endif
    digitalWrite(pinDistanceSensorXSHUT, HIGH);
    // The datasheet allows roughly 1.2 ms for the firmware to boot, so 10 ms is a comfortable
    // margin. Do not trim it: the sensor is sitting on 0x29 alongside the TCS34725 right now, so
    // this delay is the only thing guaranteeing it is awake before the address write below, and
    // reading a status register here to check would be ambiguous with two chips on that address.
    delay(10);
    startDistanceSensorRanging();
  }

  // (Re-)configure VL53L1X after power-up/XSHUT reset, validate it, and leave it idle unless auto-distance is enabled.
  bool startDistanceSensorRanging() {
    distanceTof.setTimeout(distanceTofTimeoutMs);

    // Move the sensor off the TCS34725's default address before any ToF read or configuration.
    if (!distanceTof.setAddress(distanceSensorAddress)) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L1X address set failed"));
      return false;
    }

    if (!distanceTof.waitForBoot()) {
      distanceTofDetected = false;
      #if DEBUG_DISTANCE_SENSOR
      distanceTof.logStartupI2cState(F("VL53L1X startup: boot wait failed"));
      #endif
      DBGLN_DISTANCE_SENSOR(F("VL53L1X did not boot at its assigned address"));
      return false;
    }

    // init() confirms the model ID at 0x2A before writing the configuration block.
    #if DEBUG_DISTANCE_SENSOR
    distanceTof.logStartupI2cState(F("VL53L1X startup: after address assignment"));
    distanceTof.logModelId();
    #endif
    if (!distanceTof.init()) {
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

    distanceTofDetected = true;
    distanceTofFaultLatched = false;
    resetDistanceFilter();
    lastGoodTofReadMs = 0;
    lastTofReadMs = 0;
    if (AutoDistanceOnOff) {
      distanceTof.startContinuous(distanceTofContinuousPeriodMs);
      tofRangingStartedMs = millis();
      tofRangingActive = true;
    } else {
      tofRangingActive = false;
    }
    logDistanceSensorStatus(F(" ready"));
    return true;
  }

  // Start or stop VL53L1X continuous ranging without a full re-initialization.
  void setDistanceSensorRangingActive(bool active) {
    if (!distanceTofDetected) return;  // nothing to control (never found, or fault-latched)
    if (active == tofRangingActive) return;  // already in the requested state
    if (active) {
      resetDistanceFilter();
      lastGoodTofReadMs = 0;
      lastTofReadMs = 0;
      distanceTof.startContinuous(distanceTofContinuousPeriodMs);
      tofRangingStartedMs = millis();
      tofRangingActive = true;
      logDistanceSensorStatus(F(" ranging started"));
    } else {
      distanceTof.stopContinuous();
      tofRangingActive = false;
      logDistanceSensorStatus(F(" ranging stopped"));
    }
  }
#endif
