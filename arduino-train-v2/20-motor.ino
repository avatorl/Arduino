  // ================================================================================================
  // File description
  // ================================================================================================
  // Motor-driving and auto-distance speed control live here.
  // These settings mainly affect how the train accelerates, brakes, and reverses.

  constexpr int AUTO_DISTANCE_INVALID = -1;

  // Obstacle stop latch for auto-distance hysteresis (see motorVoltageFromDistance() below).
  // true  = the train stopped because an obstacle came closer than AUTO_DISTANCE_STOP and must stay
  //         stopped until the obstacle clears past AUTO_DISTANCE_RESTART.
  // false = normal driving; distance simply scales the speed.
  // The gap between STOP (8 cm) and RESTART (11 cm) prevents rapid stop/start oscillation when an
  // obstacle sits exactly at one threshold (sensor noise would otherwise flip the decision every
  // reading, jerking the train). Reset via resetAutoDistanceState() whenever auto mode is toggled.
  bool autoObstacleStopLatched = false;

  // Map obstacle distance to a target motor voltage for auto-distance mode.
  // This function turns a distance reading into a target voltage using simple linear interpolation
  // ("ramp up smoothly between two points") instead of a lookup table: at or below restartDistanceCm
  // -> crawl at minMotorMv (the stop decision itself is made by the caller's hysteresis latch, not
  // here); farther than maxSpeedDistanceCm -> the maximum allowed voltage; in between, the target
  // voltage rises in a straight line from minMotorMv up to maxMotorMv as the obstacle gets farther
  // away. All the math is done in integers (no floating point) since AVR chips are slow at
  // floating-point arithmetic: "spanV"/"spanD" are the total voltage/distance ranges, and
  // "(spanD / 2U)" added before dividing is a standard integer-rounding trick (rounds to the
  // nearest whole number instead of always rounding down).
  uint16_t motorVoltageFromDistanceMm(
    int distanceCm,
    int restartDistanceCm,
    int maxSpeedDistanceCm,
    uint16_t minMotorMv,
    uint16_t maxMotorMv
  ) {
    if (distanceCm <= restartDistanceCm) return minMotorMv;

    if (distanceCm < maxSpeedDistanceCm) {
      const uint16_t spanV = (uint16_t)(maxMotorMv - minMotorMv);
      const uint16_t spanD = (uint16_t)(maxSpeedDistanceCm - restartDistanceCm);
      const uint16_t rawV = (uint16_t)(
        minMotorMv + (((uint32_t)(distanceCm - restartDistanceCm) * spanV + (spanD / 2U)) / spanD)
      );
      return constrain(rawV, minMotorMv, maxMotorMv);
    }

    return maxMotorMv;
  }

  // Disable auto-distance mode and clear its indicator.
  // Also stops distance-sensor continuous ranging (the sensor is only needed while auto mode is active,
  // so this saves power and I2C traffic) and clears the obstacle stop latch so the next auto
  // session starts from a clean state.
  void exitAutoDistanceMode(bool clearIndicator) {
    AutoDistanceOnOff = false;
    resetAutoDistanceState();
    setDistanceSensorRangingActive(false);
    if (clearIndicator) SetGreenLightValue(0);
  }

  // Clear the obstacle stop latch used by motorVoltageFromDistance()'s hysteresis.
  // Called whenever auto-distance mode is entered or exited so a stale "stopped by obstacle"
  // decision from a previous session cannot leak into a new one.
  void resetAutoDistanceState() {
    autoObstacleStopLatched = false;
  }

  // Stop motor and reset manual speed step to 0.
  void stopAndResetStepSelection(bool resetDirection) {
    Stop();
    currentStep = 0;
    if (resetDirection) MotorDirection = 1;
  }

  // Clear the momentary jog active flag.
  void cancelJog() {
    momentaryActive = false;
  }

  // Return a beginner-friendly label for the current manual speed level.
  const __FlashStringHelper* motorLevelLabel(uint8_t step) {
    switch (step) {
      case 0: return F("0");
      case 1: return F("1");
      case 2: return F("2");
      case 3: return F("3");
      case BOOST_SPEED_STEP: return F("4 (BOOST)");
      default: return F("unknown");
    }
  }

  // Print a motor debug line that always includes both the speed level and PWM value.
  void logMotorLevelAndPwm(const __FlashStringHelper* prefix, uint8_t step, int pwm) {
    DBG_MOTOR(prefix);
    DBG_MOTOR(F(" level="));
    DBG_MOTOR(motorLevelLabel(step));
    DBG_MOTOR(F(", pwm="));
    DBGLN_MOTOR(pwm);
  }

  // Precompute the PWM values used by the manual speed-step buttons.
  void configureSpeedSteps() {
    const int stepCount = sizeof(pwmSteps) / sizeof(pwmSteps[0]);
    uint16_t restingVoltage = (batteryVoltage > 0) ? batteryVoltage : getBatteryVoltageDirect();
    bool pwmChanged = false;
    for (int i = 0; i < stepCount; i++) {
      uint8_t nextPwm = safePWMFromVoltage(voltageSteps[i], restingVoltage);
      if (nextPwm != pwmSteps[i]) pwmChanged = true;
      pwmSteps[i] = nextPwm;
    }
    if (!pwmChanged) return;

    DBGLN_MOTOR(F("Configured speed steps (PWM values):"));
    for (int i = 0; i < stepCount; i++) {
      DBG_MOTOR(F("Level "));
      DBG_MOTOR(motorLevelLabel(i));
      DBG_MOTOR(F(": "));
      DBG_MOTOR(voltageSteps[i]);
      DBG_MOTOR(F("mV -> PWM "));
      DBGLN_MOTOR(pwmSteps[i]);
    }
  }

  // Audible confirmation for the selected manual step.
  void playStepBeep(int step) {
    if (!areUserSoundsAllowed() || SoundOnOff != 1) return;  // respect mute and battery policy
    if (batterySignalActive || sirenActive) return;
    stopMelody();
    clearBuzzerPattern();
    int idx = 0;
    if (step == 0) {
      buzzerPattern[idx++] = 90;
    } else {
      for (int i = 0; i < step; i++) {
        buzzerPattern[idx++] = 150;  // ON
        buzzerPattern[idx++] = 150;  // OFF
      }
    }
    buzzerPattern[idx] = 0;
    buzzerIndex = 0;
    buzzerTimer = millis();
    if (buzzerPattern[0] > 0) tone(pinBuzzer, buzzerPatternToneHz);
  }

  // Apply the current manual step to the motor state.
  void applySpeedStep() {
    Speed = pwmSteps[currentStep];

    DBG_MOTOR(F("Selected level="));
    DBG_MOTOR(motorLevelLabel(currentStep));
    DBG_MOTOR(F(", target="));
    DBG_MOTOR(voltageSteps[currentStep]);
    DBG_MOTOR(F("mV, pwm="));
    DBGLN_MOTOR(Speed);

    playStepBeep(currentStep);

    if (Speed == 0) Stop();
    else if (MotorDirection == 1) GoForward();
    else if (MotorDirection == 2) GoBackward();
  }

  // CH- and CH+ adjust the current drive step through these thin wrappers.
  // Move between manual drive steps.
  void increaseStep() {
    if (currentStep < NORMAL_MAX_SPEED_STEP) {
      currentStep++;
    } else if (currentStep == NORMAL_MAX_SPEED_STEP) {
      if (!isBoostAllowed()) {
        // Boost is disabled by battery policy (Warning/Shutdown), independent of the cooldown timer.
        DBGLN_MOTOR(F("Ignored: boost disabled by battery policy"));
        SetRGBColor(RgbColor::Yellow);
        // Denial cue: bypass the battery-restriction sound gate so the user still hears it.
        playPattern(pattern_tiltBeep, true);
      } else if ((long)(millis() - boostCooldownEndsAt) >= 0) {
        currentStep = BOOST_SPEED_STEP;
        boostActive = true;
        boostEndsAt = millis() + BOOST_DURATION_MS;
        SetRGBColor(RgbColor::Magenta);
        playPattern(pattern_double);
      } else {
        unsigned long remainingMs = boostCooldownEndsAt - millis();
        unsigned long remainingSeconds = (remainingMs + 999UL) / 1000UL;
        DBG_MOTOR(F("Boost cooldown period: wait "));
        DBG_MOTOR(remainingSeconds);
        DBGLN_MOTOR(F(" s before boosting the motor again"));
        SetRGBColor(RgbColor::Yellow);
        // Denial cue: bypass the battery-restriction sound gate so the user still hears it.
        playPattern(pattern_tiltBeep, true);
      }
    }
    applySpeedStep();
  }
  // Move between manual drive steps.
  void decreaseStep() {
    if (boostActive) {
      boostActive = false;
      boostCooldownEndsAt = millis() + BOOST_COOLDOWN_MS;
      playPattern(pattern_descend);
    }
    if (currentStep > 0) currentStep--;
    applySpeedStep();
  }

  // ================================================================================================
  // Auto-distance speed control
  // ================================================================================================
  // Convert obstacle distance into a motor-voltage target with stop/restart hysteresis.
  // Map obstacle distance to a target motor voltage.
  // Hysteresis (the reason STOP and RESTART are two different distances):
  //   - While driving: an obstacle closer than AUTO_DISTANCE_STOP (8 cm) latches a full stop.
  //   - While stopped by the latch: the train stays stopped until the obstacle clears past
  //     AUTO_DISTANCE_RESTART (11 cm); readings inside the 8-11 cm band keep the previous decision.
  //   - While driving inside the 8-11 cm band (obstacle slowly approaching), the train crawls at
  //     the minimum voltage until the obstacle either clears or crosses the 8 cm stop line.
  // Without this band the train would oscillate stop/start when an obstacle sits near a single
  // threshold, because sensor noise flips consecutive readings above/below it.
  uint16_t motorVoltageFromDistance(int distance) {
    if (distance < AUTO_DISTANCE_STOP) autoObstacleStopLatched = true;         // too close -> latch stop
    else if (distance > AUTO_DISTANCE_RESTART) autoObstacleStopLatched = false; // clear -> release latch
    if (autoObstacleStopLatched) return 0;

    uint16_t minV = min(MAX_SAFE_MOTOR_MV, voltageSteps[1]);        // ≈3.5V
    uint16_t maxV = min(NORMAL_MAX_MOTOR_MV, voltageSteps[NORMAL_MAX_SPEED_STEP]);  // ≈6.0V
    return motorVoltageFromDistanceMm(
      distance,
      AUTO_DISTANCE_RESTART,
      AUTO_DISTANCE_MAX_SPEED,
      minV,
      maxV
    );
  }

  // Poll distance, compute target speed, and hand off ramping to updateMotorSpeed().
  // Run the automatic speed controller.
  void updateAutoDistanceSpeed() {
    if (!isAutoDistanceAllowed()) {
      exitAutoDistanceMode();
      return;
    }
    int distanceReading = getDistanceReading();

    if (distanceReading < 0) {
      pendingMotorStopReason = F("auto: invalid distance sensor");
      SetRGBColor(RgbColor::Red);
      DBGLN_DISTANCE_SENSOR(F("Auto: STOP (invalid distance sensor)"));
      updateMotorSpeed(0);
      return;
    }

    DBG_DISTANCE_SENSOR(F("Distance: "));
    DBG_DISTANCE_SENSOR(distanceReading);
    DBGLN_DISTANCE_SENSOR(F(" cm"));

    uint16_t targetV = motorVoltageFromDistance(distanceReading);
    int targetSpeed = safePWMFromVoltage(targetV, getLoadedBatteryVoltageForMotorControl());

    if (targetSpeed == 0) {
      SetRGBColor(RgbColor::Red);
      DBGLN_DISTANCE_SENSOR(F("Auto: STOP"));
    } else {
      SetRGBColor(RgbColor::White);
      DBG_DISTANCE_SENSOR(F("Auto target speed = "));
      DBG_DISTANCE_SENSOR(targetSpeed);
      DBG_DISTANCE_SENSOR(F(" (~ "));
      DBG_DISTANCE_SENSOR(targetV);
      DBGLN_DISTANCE_SENSOR(F(" mV)"));
    }

    updateMotorSpeed(targetSpeed);
  }

  static unsigned long lastRamp = 0;

  // Smooth large speed changes so auto mode does not jerk the drivetrain.
  // Ramp the motor toward a requested speed.
  // SAFETY: a stop request (targetSpeed == 0) is executed IMMEDIATELY, before the ramp-delay gate
  // below. The gate exists only to pace gradual speed changes (one rampStep every rampDelay ms);
  // letting it delay an emergency stop by up to rampDelay (80 ms) would add several centimetres of
  // travel toward an obstacle at full speed.
  void updateMotorSpeed(int targetSpeed) {
    if (targetSpeed == 0) {
      // Skip the repeated Stop() side effects (debug spam, boost bookkeeping) once already stopped.
      if (Speed != 0 || motorDrivePending) {
        if (AutoDistanceOnOff && pendingMotorStopReason == nullptr) {
          pendingMotorStopReason = F("auto: obstacle too close");
        }
        Speed = 0;
        Stop();
      }
      return;
    }

    unsigned long now = millis();
    if (now - lastRamp < rampDelay) return;
    lastRamp = now;

    if (Speed < targetSpeed) Speed = min(Speed + rampStep, targetSpeed);
    else if (Speed > targetSpeed) Speed = max(Speed - rampStep, targetSpeed);

    if (Speed == 0) Stop();
    else GoForward();  // auto always drives through the same direction-change guard as manual mode
  }

  // nFAULT is a binary driver-protection signal, not an analog current measurement.
  // Once latched, the user must issue a fresh motor command after the fault clears.
  // Overwrites the fault_count byte for the current boot slot (capped to max 16 writes per power-on cycle).
  // Only increments the fault counter if motor drive was actually attempted/started since the previous fault.
  // The DRV8833 motor driver chip pulls its nFAULT output pin LOW when it detects a problem such as
  // overcurrent, a short circuit, or overheating (the leading "n" is a common hardware naming
  // convention meaning "active-low": LOW = fault, HIGH = OK). pinMotorFault is wired with
  // INPUT_PULLUP, so digitalRead() reads HIGH normally and LOW only when the chip actively pulls it
  // down - this function runs every loop() (see updateMotorFault() call in loop()) to catch that
  // transition quickly and immediately stop driving until the user explicitly re-arms it.
  // Latch, report, and EEPROM-log DRV8833 fault conditions.
  void updateMotorFault() {
    if (motorFaultLatched || digitalRead(pinMotorFault) == HIGH) return;

    Stop();  // Coast the motor before disabling the driver.
    digitalWrite(pinMotorSleep, LOW);
    exitAutoDistanceMode();
    cancelJog();
    currentStep = 0;
    motorFaultLatched = true;

    #if ENABLE_EEPROM_LOGGING
    // Log the fault event by updating the single fault counter byte (max 16 times per power-on).
    if (motorDriveAttemptedSinceFault) {
      motorDriveAttemptedSinceFault = false;
      if (bootFaultCount < MAX_FAULTS_PER_BOOT
          && currentBootSlot < EEPROM_BOOT_LOG_SIZE
          && reserveRuntimeEepromWrite()) {
        bootFaultCount++;
        uint16_t faultAddr = EEPROM_ADDR_LOG_BASE + (uint16_t)currentBootSlot * EEPROM_ENTRY_SIZE + 4;
        EEPROM.update(faultAddr, bootFaultCount);
        DBG_MOTOR(F("DRV8833 fault #"));
        DBG_MOTOR(bootFaultCount);
        DBGLN_MOTOR(F(" logged to EEPROM"));
      }
    }
    #endif

    sirenActive = false;
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    SetGreenLightValue(0);
    SetRGBLightColor(RgbColor::Red);  // Bypass siren suppression for a safety indication.
    playPattern(pattern_batteryWarn);
    DBGLN_MOTOR(F("DRV8833 fault: motor disabled until a motor command re-arms it"));
  }

  // Re-enable the driver after a cleared fault.
  bool rearmMotorDriver() {
    if (criticalOvervoltageLatched) {
      analogWrite(pinMotor_IN1, 0);
      analogWrite(pinMotor_IN2, 0);
      digitalWrite(pinMotorSleep, LOW);
      applyCriticalOvervoltageOutputs();
      return false;
    }

    digitalWrite(pinDistanceSensorXSHUT, HIGH); // Wake up distance sensor
    delay(10); // boot time
    startDistanceSensorRanging();

    digitalWrite(pinMotorSleep, HIGH);
    delay(1);  // Allow nSLEEP to release and nFAULT to report the current condition.
    if (digitalRead(pinMotorFault) == LOW) {
      digitalWrite(pinMotorSleep, LOW);
      DBGLN_MOTOR(F("DRV8833 fault is still active"));
      return false;
    }

    motorFaultLatched = false;
    DBGLN_MOTOR(F("DRV8833 fault cleared; motor command accepted"));
    return true;
  }

  // ================================================================================================
  // Motor Control
  // ================================================================================================
  // Low-level H-bridge writer used by all higher-level movement commands.
  // Low-level DRV8833 direction and PWM output.
  // "analogWrite(pin, value)" doesn't output a real analog voltage on most Arduino pins - instead it
  // uses PWM (Pulse Width Modulation): the pin is switched fully HIGH and LOW very rapidly, and
  // "value" (0-255) controls what fraction of each cycle is HIGH ("duty cycle"). Averaged over time,
  // this behaves like a lower voltage, which is how the DRV8833 driver gets variable motor speed
  // from a digital-only pin. The DRV8833 is an H-bridge chip: driving IN1 with PWM and IN2 LOW spins
  // the motor one way, and swapping which pin gets the PWM (IN2 PWM / IN1 LOW) reverses it - IN1 and
  // IN2 must never both be driven HIGH/PWM at the same time, so the code always sets one of them to
  // exactly 0.
  void setMotor(Dir dir, int speed) {
    if (criticalOvervoltageLatched) {
      analogWrite(pinMotor_IN1, 0);
      analogWrite(pinMotor_IN2, 0);
      digitalWrite(pinMotorSleep, LOW);
      applyCriticalOvervoltageOutputs();
      return;
    }

    int safeSpeed = constrain(speed, 0, 255);
    switch (dir) {
      case Dir::Forward:
        analogWrite(pinMotor_IN1, safeSpeed);
        analogWrite(pinMotor_IN2, 0);
        MotorDirection = 1;
        if (safeSpeed > 0) motorDriveAttemptedSinceFault = true;
        logMotorLevelAndPwm(F("Driving Forward >>>"), currentStep, safeSpeed);
        break;
      case Dir::Backward:
        analogWrite(pinMotor_IN1, 0);
        analogWrite(pinMotor_IN2, safeSpeed);
        MotorDirection = 2;
        if (safeSpeed > 0) motorDriveAttemptedSinceFault = true;
        logMotorLevelAndPwm(F("Driving Backward >>"), currentStep, safeSpeed);
        break;
      default:  // Stop
        analogWrite(pinMotor_IN1, 0);
        analogWrite(pinMotor_IN2, 0);
        Speed = 0;
        if (pendingMotorStopReason != nullptr) {
          DBG_MOTOR(F("Motor OFF ("));
          DBG_MOTOR(pendingMotorStopReason);
          DBGLN_MOTOR(F(")"));
          pendingMotorStopReason = nullptr;
        } else {
          logMotorLevelAndPwm(F("Motor OFF"), currentStep, 0);
        }
        break;
    }
  }

  // Direction-safe drive entry points.
  // Drive the motor forward only after any required reverse-direction cooldown.
  // Reversing a spinning motor's direction instantly can spike current and stress the gears/driver,
  // so this "direction cooldown" state machine inserts a brief coast-and-settle pause whenever the
  // requested direction differs from the direction the motor was actually last driven in. Instead of
  // blocking with delay() (which would freeze the whole sketch, including the IR receiver and
  // safety checks), it records a future timestamp (motorReverseReadyAt = now + DIR_DELAY) and a
  // "motorDrivePending" flag; updateMotorReverseCooldown() (called every loop()) then finishes the
  // job once enough time has passed. Comparing timestamps as "(long)(millis() - target) >= 0"
  // instead of "millis() >= target" is a standard Arduino idiom that keeps working correctly even
  // when the millis() counter wraps around back to 0 after about 49 days of uptime.
  void GoForward() {
    if (MotorDirection == 2 && Speed > 0) {
      DBGLN_MOTOR(F("Ignored: cannot switch to FORWARD while moving"));
      return;
    }
    if (MotorDirection == 2 && Speed == 0) {
      Stop();
      motorReverseReadyAt = millis() + DIR_DELAY;  // non-blocking settle before reverse drive
      SetRGBColor(RgbColor::Yellow);
    }
    MotorDirection = 1;
    if (Speed > 0) {
      if ((long)(millis() - motorReverseReadyAt) >= 0) setMotor(Dir::Forward, Speed);
      else motorDrivePending = true;  // drive once the cooldown elapses
    }
  }

  // Direction-safe drive entry points.
  // Drive the motor backward only after any required reverse-direction cooldown.
  void GoBackward() {
    if (MotorDirection == 1 && Speed > 0) {
      DBGLN_MOTOR(F("Ignored: cannot switch to BACKWARD while moving"));
      return;
    }
    if (MotorDirection == 1 && Speed == 0) {
      Stop();
      motorReverseReadyAt = millis() + DIR_DELAY;  // non-blocking settle before reverse drive
      SetRGBColor(RgbColor::Yellow);
    }
    MotorDirection = 2;
    if (Speed > 0) {
      if ((long)(millis() - motorReverseReadyAt) >= 0) {
        setMotor(Dir::Backward, Speed);
        DBGLN_MOTOR(F("Driving Backward >>"));
      } else {
        motorDrivePending = true;  // drive once the cooldown elapses
      }
    }
  }

  // Coast the motor to an idle state.
  void Stop() {
    if (boostActive) {
      boostActive = false;
      boostCooldownEndsAt = millis() + BOOST_COOLDOWN_MS;
    }
    setMotor(Dir::Stop, 0);
    motorDrivePending = false;  // cancel any deferred reverse drive
  }

  // Apply a reverse drive that was deferred during the non-blocking direction cooldown.
  // Apply deferred motor drive after direction-change delay.
  void updateMotorReverseCooldown() {
    if (!motorDrivePending) return;
    if ((long)(millis() - motorReverseReadyAt) < 0) return;
    motorDrivePending = false;
    if (Speed <= 0) return;
    if (MotorDirection == 1) setMotor(Dir::Forward, Speed);
    else if (MotorDirection == 2) setMotor(Dir::Backward, Speed);
    refreshDriveLights();
  }

  // Jog motor briefly without changing the stored direction/speed state machine.
  // Temporary hold-to-run movement helper.
  // Callers (10-ir-remote.ino buttonBackward/buttonForward) only invoke this while the train is
  // fully stationary (Speed == 0, auto-distance mode off), so this never has to arbitrate against
  // an already-running manual or auto drive. Direction is whatever the caller passes in (fixed by
  // which button was pressed); speed is always the fixed step-1 PWM value (pwmSteps[1]), never the
  // last-selected manual step, and MotorDirection/Speed are intentionally left untouched since the
  // jog is momentary and should not persist once the button is released.
  void JogDrive(Dir dir) {
    if (criticalOvervoltageLatched) {
      analogWrite(pinMotor_IN1, 0);
      analogWrite(pinMotor_IN2, 0);
      digitalWrite(pinMotorSleep, LOW);
      applyCriticalOvervoltageOutputs();
      return;
    }

    int jogPWM = pwmSteps[1];
    if (dir == Dir::Forward) {
      analogWrite(pinMotor_IN1, jogPWM);
      analogWrite(pinMotor_IN2, 0);
      if (jogPWM > 0) motorDriveAttemptedSinceFault = true;
    } else if (dir == Dir::Backward) {
      analogWrite(pinMotor_IN1, 0);
      analogWrite(pinMotor_IN2, jogPWM);
      if (jogPWM > 0) motorDriveAttemptedSinceFault = true;
    } else {
      analogWrite(pinMotor_IN1, 0);
      analogWrite(pinMotor_IN2, 0);
    }
  }
