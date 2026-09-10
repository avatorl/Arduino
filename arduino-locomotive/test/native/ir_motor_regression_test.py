"""Focused fallback regressions; run by run-tests.ps1 on both compiler paths.

Timing cases mirror only the jog/reversal core of 20-motor.ino, with AVR uint32
millis arithmetic and constants read from config.h. Source contracts check the
production wiring, not C++ execution. No IR decoder, AUTO controller, battery
policy, or LED engine is reimplemented here. Hardware and scheduler execution
still require the firmware; source contracts may need updating after refactors.
"""

from pathlib import Path
import re
import unittest

from train_logic_test import safe_pwm


PROJECT = Path(__file__).resolve().parents[2]


def source(name):
    text = (PROJECT / name).read_text(encoding="utf-8-sig")
    # Preserve quoted strings while removing comments, including commented-out code.
    return re.sub(r'"(?:\\.|[^"\\])*"|//[^\n]*|/\*.*?\*/',
                  lambda match: match[0] if match[0].startswith('"') else "",
                  text, flags=re.S)


CONFIG = source("config.h")
MOTOR = source("20-motor.ino")
IR = source("10-ir-remote.ino")
MAIN = source("arduino-locomotive.ino")
LIGHTS = source("30-lights-and-sounds.ino")
POWER = source("50-power-management.ino")
DISTANCE = source("42-distance-sensor-vl53l0x.ino")


def constant(name):
    match = re.search(rf"\b{re.escape(name)}\s*=\s*(\d+)(?:UL|U|L)?\s*;", CONFIG)
    if not match:
        raise AssertionError(f"Missing numeric production constant: {name}")
    return int(match[1])


def block(text, marker):
    """Extract one balanced block; deliberately not a C++ parser."""
    start = text.index("{", text.index(marker)) + 1
    depth = 1
    for token in re.finditer(r'"(?:\\.|[^"\\])*"|[{}]', text[start:]):
        if token[0] == "{":
            depth += 1
        elif token[0] == "}":
            depth -= 1
        if depth == 0:
            return text[start:start + token.start()]
    raise AssertionError(f"Unclosed production block: {marker}")


def function(text, name):
    match = re.search(rf"\b{re.escape(name)}\s*\([^;{{}}]*\)\s*\{{", text)
    if not match:
        raise AssertionError(f"Missing production function: {name}")
    return block(text, match[0])


def compact(text):
    return re.sub(r"\s+", "", text)


TIMEOUT = constant("momentaryTimeout")
COAST = constant("DIR_DELAY")
RAMP = constant("MOMENTARY_RAMP_DURATION_MS")
VOLTAGES = [int(v) for v in re.search(r"voltageSteps\[\]\s*=\s*\{([^}]+)", CONFIG)[1].split(",")]
PWM = [safe_pwm(v, 8400, constant("MAX_SAFE_MOTOR_MV")) for v in VOLTAGES]
NORMAL_PWM = PWM[constant("NORMAL_MAX_SPEED_STEP")]
MASK = 0xFFFFFFFF


def elapsed(now, then):
    return (now - then) & MASK


class JogCore:
    """Small mirror of setMotor/requestMotorDrive/Stop and the jog update helpers."""

    def __init__(self, now=10000):
        self.now = now
        self.started = self.seen = self.stopped = 0
        self.active = self.ramping = False
        self.direction = self.last_direction = 0
        self.manual_direction = 1
        self.speed = 0  # Production jog PWM is separate from manual Speed.
        self.pwm = (0, 0)
        self.writes = []
        self.pending = None
        self.blocked = False

    def advance(self, milliseconds):
        self.now = (self.now + milliseconds) & MASK

    def write(self, direction, pwm):
        if direction and pwm > 0:
            if self.active and not any(self.pwm):
                self.started = self.now
            self.last_direction = direction
        elif any(self.pwm):
            self.stopped = self.now
        self.pwm = (pwm, 0) if direction == 1 else (0, pwm) if direction == 2 else (0, 0)
        self.writes.append(self.pwm)
        if not direction:
            self.speed = 0
        elif not self.active:
            self.manual_direction = direction

    def stop(self, reset_selection=False):
        self.write(0, 0)
        self.pending = None
        self.active = self.ramping = False
        self.direction = 0
        if reset_selection:
            self.manual_direction = 1

    def request(self, direction, pwm):
        if self.blocked or pwm <= 0 or not direction:
            self.stop()
            return
        if self.last_direction and direction != self.last_direction:
            if any(self.pwm):
                self.write(0, 0)
                if not self.active:
                    self.speed = pwm
            if elapsed(self.now, self.stopped) < COAST:
                self.pending = (direction, pwm)
                return
        self.pending = None
        self.write(direction, pwm)

    def start(self, direction, ramp=True):
        self.active, self.ramping, self.direction = True, ramp, direction
        self.started = self.seen = self.now
        self.request(direction, PWM[1])

    def heartbeat(self):
        if self.active:
            self.seen = self.now

    def watchdog(self):
        if self.active and elapsed(self.now, self.seen) >= TIMEOUT:
            self.stop()

    def ramp(self):
        if self.active and self.ramping and any(self.pwm):
            duration = min(elapsed(self.now, self.started), RAMP)
            self.request(self.direction, PWM[1] + (NORMAL_PWM - PWM[1]) * duration // RAMP)

    def cooldown(self):
        if self.pending:
            self.watchdog()
            if self.pending and elapsed(self.now, self.stopped) >= COAST:
                self.request(*self.pending)

    def tick(self):
        self.watchdog()
        self.ramp()
        self.cooldown()


class JogTimingTests(unittest.TestCase):
    def assert_disarmed(self, drive):
        self.assertEqual(drive.pwm, (0, 0))
        self.assertFalse(drive.active)
        self.assertFalse(drive.ramping)
        self.assertIsNone(drive.pending)
        drive.advance(COAST + RAMP)
        drive.heartbeat()  # A stale repeat cannot re-arm a stopped jog.
        drive.tick()
        self.assertEqual(drive.pwm, (0, 0))

    def test_production_limits(self):
        self.assertEqual((TIMEOUT, COAST, RAMP), (200, 1000, 2000))
        self.assertEqual(VOLTAGES[constant("NORMAL_MAX_SPEED_STEP")], 6000)
        self.assertEqual(constant("NORMAL_MAX_MOTOR_MV"), 6000)
        self.assertLess(NORMAL_PWM, PWM[constant("BOOST_SPEED_STEP")])

    def test_held_plus_minus_ramp_without_heartbeat_pwm_writes(self):
        for direction in (1, 2):
            for start in (10000, MASK - 127):
                with self.subTest(direction=direction, start=start):
                    drive = JogCore(start)
                    drive.start(direction)
                    self.assertEqual(max(drive.pwm), PWM[1])
                    for duration in range(100, RAMP + 501, 100):
                        drive.advance(100)
                        before = (drive.started, drive.pwm, len(drive.writes))
                        drive.heartbeat()
                        self.assertEqual((drive.started, drive.pwm, len(drive.writes)), before)
                        drive.tick()
                        expected = PWM[1] + (NORMAL_PWM - PWM[1]) * min(duration, RAMP) // RAMP
                        self.assertEqual(max(drive.pwm), expected)
                        self.assertEqual(drive.started, start)
                        self.assertEqual(drive.speed, 0)
                    self.assertEqual(max(drive.pwm), NORMAL_PWM)

    def test_release_boundary_active_and_pending_all_jog_buttons(self):
        for direction in (1, 2):
            for ramp in (False, True):
                for pending in (False, True):
                    for timeout in (200, 201, 1200):
                        with self.subTest(direction=direction, ramp=ramp, pending=pending, timeout=timeout):
                            drive = JogCore(MASK - 100)
                            if pending:
                                drive.request(3 - direction, PWM[1])
                                drive.stop(reset_selection=True)
                            drive.start(direction, ramp)
                            drive.advance(199)
                            drive.tick()
                            self.assertTrue(drive.active)
                            drive.advance(timeout - 199)
                            # Also exercise the watchdog inside the deferred-drive helper itself.
                            drive.cooldown() if pending else drive.watchdog()
                            self.assert_disarmed(drive)

    def test_stop_or_safety_gate_disarms_active_and_pending_ramps(self):
        for pending in (False, True):
            for blocked_request in (False, True):
                with self.subTest(pending=pending, blocked_request=blocked_request):
                    drive = JogCore()
                    if pending:
                        drive.request(2, PWM[1])
                        drive.stop()
                    drive.start(1)
                    if blocked_request:
                        drive.blocked = True
                        drive.request(1, PWM[1])
                    else:
                        drive.stop()
                    drive.blocked = False
                    self.assert_disarmed(drive)

    def test_reversal_uses_physical_direction_and_does_not_restart_coast(self):
        for previous in (1, 2):
            for stopped_first in (False, True):
                for start in (10000, MASK - 50):
                    with self.subTest(previous=previous, stopped_first=stopped_first, start=start):
                        drive = JogCore(start)
                        drive.request(previous, PWM[1])
                        if stopped_first:
                            drive.stop(reset_selection=True)
                            self.assertEqual(drive.manual_direction, 1)
                        drive.start(3 - previous)
                        coast_start = drive.stopped
                        for _ in range(9):
                            drive.advance(100)
                            drive.heartbeat()
                            drive.request(3 - previous, PWM[1])
                            drive.tick()
                            self.assertEqual(drive.stopped, coast_start)
                            self.assertEqual(drive.pwm, (0, 0))
                            self.assertIsNotNone(drive.pending)
                        drive.advance(99)
                        drive.tick()
                        self.assertEqual(drive.pwm, (0, 0))
                        drive.advance(1)
                        drive.tick()
                        self.assertIsNone(drive.pending)
                        self.assertEqual(drive.last_direction, 3 - previous)
                        self.assertEqual(max(drive.pwm), PWM[1])
                        self.assertEqual(drive.started, drive.now)
                        drive.advance(100)
                        drive.heartbeat()
                        drive.tick()
                        self.assertGreater(max(drive.pwm), PWM[1])

    def test_same_direction_needs_no_coast(self):
        for direction in (1, 2):
            drive = JogCore()
            drive.request(direction, PWM[1])
            drive.stop(reset_selection=True)
            drive.start(direction, ramp=False)
            self.assertIsNone(drive.pending)
            self.assertEqual(max(drive.pwm), PWM[1])


class ProductionContractTests(unittest.TestCase):
    """Structural checks against live production functions, not runtime coverage."""

    def contains(self, text, *fragments):
        for fragment in fragments:
            self.assertIn(compact(fragment), compact(text), fragment)

    def ordered(self, text, *fragments):
        cursor = 0
        for fragment in fragments:
            index = compact(text).find(compact(fragment), cursor)
            self.assertNotEqual(index, -1, f"Missing/out-of-order production code: {fragment}")
            cursor = index + len(compact(fragment))

    def test_heartbeat_is_only_activity_not_a_new_jog(self):
        router = function(IR, "translateIR")
        heartbeat = block(router, "if (momentaryActive && code == momentaryButton)")
        self.contains(heartbeat, "momentaryLastSeen = millis();", "lastActive = momentaryLastSeen;",
                      "idleSleepWarningIssued = false;", "return;")
        assignments = set(re.findall(r"\b(\w+)\s*=(?!=)", heartbeat))
        self.assertEqual(assignments, {"momentaryLastSeen", "lastActive", "idleSleepWarningIssued"})
        self.assertEqual(re.findall(r"\b(\w+)\s*\(", heartbeat), ["millis"])
        self.ordered(router, "if (lastWasRepeat && (!momentaryActive || code != momentaryButton)) return;",
                     "if (momentaryActive && code == momentaryButton)", "GreenLEDBlink();")
        for button, direction, ramp in (("buttonPlus", "Forward", "true"), ("buttonMinus", "Backward", "true"),
                                        ("buttonForward", "Forward", "false"), ("buttonBackward", "Backward", "false")):
            self.contains(block(router, f"case {button}:"),
                          f"startJog(Dir::{direction}, {button}, {ramp});")

    def test_mirrored_ramp_and_unsigned_watchdog_match_production(self):
        self.contains(function(MOTOR, "startJog"), "momentaryStartedAt = millis();",
                      "momentaryLastSeen = momentaryStartedAt;", "JogDrive(dir, pwmSteps[1]);")
        self.contains(function(MOTOR, "updateJogDriveSpeed"),
                      "if (!momentaryActive || !momentaryRampActive || !motorOutputActive) return;",
                      "const unsigned long elapsedMs = millis() - momentaryStartedAt;",
                      "elapsedMs < MOMENTARY_RAMP_DURATION_MS ? elapsedMs : MOMENTARY_RAMP_DURATION_MS",
                      "const int minimumPwm = pwmSteps[1];",
                      "const int maximumPwm = pwmSteps[NORMAL_MAX_SPEED_STEP];",
                      "((uint32_t)(maximumPwm - minimumPwm) * cappedElapsedMs) / MOMENTARY_RAMP_DURATION_MS")
        self.contains(function(MOTOR, "updateJogWatchdog"),
                      "momentaryActive && millis() - momentaryLastSeen >= momentaryTimeout", "Stop();")
        self.contains(MAIN, "unsigned long momentaryStartedAt", "unsigned long momentaryLastSeen",
                      "unsigned long motorStoppedAt")
        self.ordered(function(MAIN, "loop"), "updateJogWatchdog();", "updateJogDriveSpeed();")

    def test_stop_tilt_and_battery_disarm_before_any_future_ramp(self):
        stop = function(MOTOR, "Stop")
        self.ordered(stop, "setMotor(Dir::Stop, 0);", "motorDrivePending = false;", "cancelJog();")
        self.contains(stop, "pendingMotorDirection = Dir::Stop;", "pendingMotorPwm = 0;")
        self.assertNotRegex(stop, r"\b(?:lastMotorDriveDirection|motorStoppedAt)\s*=")
        self.contains(function(MOTOR, "cancelJog"), "momentaryActive = false;",
                      "momentaryRampActive = false;", "momentaryButton = 0;")
        self.ordered(function(MOTOR, "stopAndResetStepSelection"), "Stop();", "if (resetDirection) MotorDirection = 1;")
        self.contains(function(MAIN, "updateTiltSensor"), "stopAndResetStepSelection();", "tiltStopLatched = true;")
        self.ordered(function(POWER, "applyBatteryRestrictions"),
                     "if (boostActive || momentaryActive) Stop();", "cancelJog();")
        guard = block(function(MOTOR, "requestMotorDrive"), "if (criticalOvervoltageLatched")
        self.contains(function(MOTOR, "requestMotorDrive"), "motorFaultLatched",
                      "batteryState == BatteryState::Shutdown", "tiltStopLatched", "accelerometerTiltStopLatched")
        self.ordered(guard, "Stop();", "return;")

    def test_vin_shutdown_window_is_independent_of_vcc_shutdown(self):
        self.contains(
            CONFIG,
            "#define ENABLE_VIN_BATTERY_SHUTDOWN 1",
            "#define ENABLE_VCC_POWER_SHUTDOWN 0",
            "#define ENABLE_OVERVOLTAGE_POWER_SHUTDOWN 0",
            "VIN_BATTERY_SHUTDOWN_MIN_MV = 5500",
            "VIN_BATTERY_SHUTDOWN_MAX_MV = 6500",
        )
        self.contains(
            function(POWER, "isVinBatteryShutdownEligible"),
            "voltageMv > VIN_BATTERY_SHUTDOWN_MIN_MV",
            "voltageMv < VIN_BATTERY_SHUTDOWN_MAX_MV",
        )
        self.contains(
            function(MAIN, "setup"),
            "if (isVinBatteryShutdownEligible(batteryVoltage))",
            "if (isVinBatteryShutdownEligible(retry))",
        )
        self.contains(
            function(POWER, "updateBatteryGuard"),
            "else if (isVinBatteryShutdownEligible(v))",
            "#if ENABLE_VIN_BATTERY_SHUTDOWN",
            "enterBatteryShutdown();",
        )
        self.contains(
            function(POWER, "updateVccGuard"),
            "#if ENABLE_VCC_POWER_SHUTDOWN",
            "enterBatteryShutdown(false, ShutdownCause::LowVcc);",
        )

    def test_reversal_and_deferred_release_use_physical_elapsed_time(self):
        request = function(MOTOR, "requestMotorDrive")
        self.contains(request, "lastMotorDriveDirection != Dir::Stop && dir != lastMotorDriveDirection",
                      "if (motorOutputActive)", "if (millis() - motorStoppedAt < DIR_DELAY)",
                      "pendingMotorDirection = dir;", "pendingMotorPwm = constrain(speed, 0, 255);")
        self.assertNotRegex(request, r"\bmotorStoppedAt\s*=")
        writer = function(MOTOR, "setMotor")
        self.contains(writer, "if (momentaryActive && !motorOutputActive) momentaryStartedAt = millis();",
                      "lastMotorDriveDirection = dir;")
        self.contains(block(writer, "else if (motorOutputActive)"),
                      "motorStoppedAt = millis();", "motorOutputActive = false;")
        self.ordered(function(MOTOR, "updateMotorReverseCooldown"), "updateJogWatchdog();",
                     "if (!motorDrivePending || millis() - motorStoppedAt < DIR_DELAY) return;",
                     "requestMotorDrive(pendingMotorDirection, pendingMotorPwm);")

    def test_auto_handoff_preserves_pwm_ends_boost_and_starts_cooldown(self):
        toggle = block(function(IR, "translateIR"), "case buttonPlayPause:")
        boost = block(toggle, "if (boostActive)")
        enable = block(toggle, "if (AutoDistanceOnOff)")
        self.contains(boost, "boostActive = false;",
                      "boostCooldownEndsAt = millis() + BOOST_COOLDOWN_MS;",
                      "currentStep = NORMAL_MAX_SPEED_STEP;")
        self.contains(enable, "resetAutoDistanceState();", "setDistanceSensorRangingActive(true);")
        for section in (boost, enable):
            self.assertNotRegex(section, r"\b(?:Stop|stopAndResetStepSelection|applySpeedStep|setMotor|GoForward)\s*\(")
            self.assertNotRegex(section, r"\b(?:Speed|motorDrivePending|lastMotorDriveDirection)\s*=")
        auto = function(MOTOR, "updateAutoDistanceSpeed")
        self.ordered(auto, "if (distanceReading == AUTO_DISTANCE_PENDING) return;",
                     "if (distanceReading < 0)", "updateMotorSpeed(0, 0);", "motorVoltageFromDistance(distanceReading)")
        self.contains(block(auto, "if (distanceReading < 0)"), "updateMotorSpeed(0, 0);", "return;")
        self.ordered(function(MOTOR, "updateMotorSpeed"), "if (targetSpeed == 0)", "Stop();", "now - lastRamp < rampDelay")

    def test_unknown_commands_and_their_repeats_cannot_refresh_jog_or_activity(self):
        router = function(IR, "translateIR")
        self.ordered(router, "if (!isKnownIRCommand(code)) return;", "if (momentaryActive && code != 0",
                     "momentaryLastSeen = millis();", "lastActive = millis();", "GreenLEDBlink();")
        self.contains(function(IR, "irReceive"),
                      "command <= UINT8_MAX && isKnownIRCommand((uint8_t)command) ? (uint8_t)command : 0;",
                      "lastIRCommand = received;")
        self.contains(function(IR, "isKnownIRCommand"), "default: return false;")

    def test_boost_denial_and_siren_off_restore_drive_colors(self):
        refresh = function(LIGHTS, "refreshDriveLights")
        self.ordered(refresh, "if (sirenActive) return;", "RgbColor driveColor = RgbColor::Red;",
                     "if (motorDrivePending) driveColor = RgbColor::Yellow;",
                     "else if (boostActive) driveColor = RgbColor::Magenta;",
                     "else if (motorOutputActive)")
        self.contains(refresh, "lastMotorDriveDirection == Dir::Backward ? RgbColor::Blue : RgbColor::White;",
                      "SetRGBLightColor(RgbColor::Cyan, 1);", "SetRGBLightColor(driveColor, 2);",
                      "SetRGBColor(driveColor);")
        self.contains(block(function(IR, "translateIR"), "if (!sirenActive)"), "refreshDriveLights();")
        increase = function(MOTOR, "increaseStep")
        policy_denial = block(increase, "if (!isBoostAllowed())")
        cooldown_denial = block(increase, "} else {")
        for denial in (policy_denial, cooldown_denial):
            self.ordered(denial, "SetRGBColor(RgbColor::Yellow);", "playPattern(pattern_tiltBeep, true);", "return;")
            self.assertNotRegex(denial, r"\b(?:applySpeedStep|refreshDriveLights)\s*\(")
        self.contains(increase, "currentStep = BOOST_SPEED_STEP;", "boostActive = true;", "applySpeedStep();")
        self.contains(function(MOTOR, "applySpeedStep"), "refreshDriveLights();")

    def test_idle_wake_recreates_sensor_driver_before_reinitializing(self):
        recovery = function(DISTANCE, "recoverDistanceSensorAfterXshut")
        self.ordered(
            recovery,
            "distanceTof = VL53L0X();",
            "return startDistanceSensorRanging();",
        )
        self.assertEqual(
            compact(recovery),
            compact("distanceTof = VL53L0X(); return startDistanceSensorRanging();"),
        )

        idle = function(POWER, "goToIdle")
        self.ordered(
            idle,
            "digitalWrite(pinDistanceSensorXSHUT, HIGH);",
            "delay(10);",
            "recoverDistanceSensorAfterXshut();",
        )
        self.assertNotIn(
            compact("startDistanceSensorRanging();"),
            compact(block(idle, "digitalWrite(pinDistanceSensorXSHUT, HIGH);")),
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
