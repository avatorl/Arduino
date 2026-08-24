  // ================================================================================================
  // File description
  // ================================================================================================
  // MPU-6050 accelerometer driver and the tilt/crash safety checks built on top of it.
  // Split out of the former 40-sensors.ino; the color sensor lives in 41-color-sensor.ino and the
  // distance sensor in 42-distance-sensor.ino. The mechanical tilt switch is handled by
  // updateTiltSensor() in arduino-train-v2.ino.

  // MPU-6050 accelerometer helper.
  // Like the other sensor helpers in this sketch, this is a minimal hand-written register-level
  // driver rather than a library, to keep flash and SRAM use down on the Nano.
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

  TrainAccelerometerMPU6050 accelerometer;
  bool accelerometerDetected = false;
  unsigned long lastAccelerometerReadMs = 0;
  unsigned long accelerometerTiltStartedMs = 0;
  bool accelerometerTiltTiming = false;
  unsigned long lastAccelerometerDebugMs = 0;
  bool accelerometerPreviousForwardSampleValid = false;
  int32_t accelerometerPreviousForwardRaw = 0;

  // Probe the MPU-6050 and report whether accelerometer-based safety can run.
  void initAccelerometerHardware() {
    accelerometerDetected = accelerometer.begin_I2C();
    if (accelerometerDetected) {
      DBGLN_ACCELEROMETER(F("MPU-6050 ready"));
    } else {
      DBGLN_ACCELEROMETER(F("MPU-6050 not detected; accelerometer safety disabled"));
    }
  }

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
