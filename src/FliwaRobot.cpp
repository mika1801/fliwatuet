#include "FliwaRobot.h"
#include <cmath>

namespace fliwa {

const char* robotStateName(RobotState s) {
    switch (s) {
        case RobotState::INIT:    return "INIT";
        case RobotState::EXPLORE: return "EXPLORE";
        case RobotState::RECOVER: return "RECOVER";
        case RobotState::ESTOP:   return "ESTOP";
        default:                  return "UNKNOWN";
    }
}

FliwaRobot::FliwaRobot()
    : _state(RobotState::INIT), _stateEntryMs(0),
      _stuckCheckMs(0), _recoveryTargetHeading(0.0f),
      _recoveryScanStartMs(0), _recoveryScanDone(false),
      _lastNavMs(0), _lastPidMs(0),
      _lidarOk(false), _loggerOk(false) {}

// ============================================================================
//  begin() — Full hardware initialization
// ============================================================================
bool FliwaRobot::begin(const char* wifiSsid, const char* wifiPassword,
                        const char* logTargetIP) {
    Serial.println(F("[ROBOT] Initializing..."));

    // Motors
    _motor.begin();
    Serial.println(F("[ROBOT] Motors OK"));

    // Odometry (starts encoder Ticker)
    _odom.begin();
    Serial.println(F("[ROBOT] Odometry OK"));

    // LiDAR
    _lidarOk = _lidar.begin(Serial1);
    if (_lidarOk) {
        Serial.println(F("[ROBOT] LiDAR OK"));
    } else {
        Serial.println(F("[ROBOT] LiDAR FAILED — running without LiDAR!"));
    }

    // WiFi logger (optional)
    if (wifiSsid && wifiPassword && logTargetIP) {
        _loggerOk = _logger.begin(wifiSsid, wifiPassword, logTargetIP);
        if (_loggerOk) {
            Serial.println(F("[ROBOT] WiFi Logger OK"));
        } else {
            Serial.println(F("[ROBOT] WiFi Logger FAILED — running without logging"));
        }
    }

    // Initialize state
    _state        = RobotState::INIT;
    _stateEntryMs = millis();
    _lastNavMs    = millis();
    _lastPidMs    = millis();

    _stuckCheckPose = _odom.pose();
    _stuckCheckMs   = millis();

    Serial.println(F("[ROBOT] Init complete, entering INIT state"));
    return _lidarOk;
}

// ============================================================================
//  update() — Main tick, called every loop() iteration
// ============================================================================
void FliwaRobot::update() {
    uint32_t now = millis();

    // Always read lidar data (non-blocking, ~10ms time-slice)
    if (_lidarOk) {
        _lidar.update();
    }

    // Always update odometry
    float odomDt = _odom.update();

    // PID motor control at 50 Hz
    if (now - _lastPidMs >= PID_INTERVAL_MS) {
        float pidDt = (float)(now - _lastPidMs) / 1000.0f;
        _motor.updatePID(_odom.leftWheelRadS(), _odom.rightWheelRadS(), pidDt);
        _lastPidMs = now;
    }

    // Navigation state machine at 20 Hz
    if (now - _lastNavMs < NAV_LOOP_INTERVAL_MS) return;
    float navDt = (float)(now - _lastNavMs) / 1000.0f;
    _lastNavMs = now;

    switch (_state) {
        case RobotState::INIT:
            stateInit();
            break;
        case RobotState::EXPLORE:
            stateExplore(navDt);
            break;
        case RobotState::RECOVER:
            stateRecover(navDt);
            break;
        case RobotState::ESTOP:
            stateEstop();
            break;
    }

    // Update telemetry for WiFi logger
    if (_loggerOk) {
        updateTelemetry();
    }
}

// ============================================================================
//  emergencyStop() / resume()
// ============================================================================
void FliwaRobot::emergencyStop() {
    _motor.emergencyStop();
    _state = RobotState::ESTOP;
    _stateEntryMs = millis();
    Serial.println(F("[ROBOT] EMERGENCY STOP"));
}

void FliwaRobot::resume() {
    if (_state == RobotState::ESTOP) {
        _state = RobotState::EXPLORE;
        _stateEntryMs = millis();
        _stuckCheckPose = _odom.pose();
        _stuckCheckMs   = millis();
        Serial.println(F("[ROBOT] Resuming → EXPLORE"));
    }
}

// ============================================================================
//  INIT state — wait for lidar to produce data, then transition to EXPLORE
// ============================================================================
void FliwaRobot::stateInit() {
    uint32_t elapsed = millis() - _stateEntryMs;

    // Wait up to 3 seconds for lidar data, then transition regardless
    if (_lidarOk && _lidar.nearestObstacleMm(1000) < 1e5f) {
        // Lidar is producing valid data
        _state = RobotState::EXPLORE;
        _stateEntryMs   = millis();
        _stuckCheckPose = _odom.pose();
        _stuckCheckMs   = millis();
        Serial.println(F("[ROBOT] LiDAR data received → EXPLORE"));
        return;
    }

    if (elapsed > 3000) {
        _state = RobotState::EXPLORE;
        _stateEntryMs   = millis();
        _stuckCheckPose = _odom.pose();
        _stuckCheckMs   = millis();
        Serial.println(F("[ROBOT] Init timeout → EXPLORE (blind mode)"));
    }
}

// ============================================================================
//  EXPLORE state — main autonomous navigation
//
//  1. Determine exploration heading (largest gap direction).
//  2. Run DWA to compute optimal (v, omega).
//  3. Convert to wheel velocities and command motors.
//  4. Check for stuck condition → transition to RECOVER.
// ============================================================================
void FliwaRobot::stateExplore(float dt) {
    // Compute exploration goal heading
    float goalHeading = computeExplorationHeading();
    _dwa.setGoalHeading(goalHeading);

    // Run DWA planner
    float curV = _odom.linearVelocity();
    float curW = _odom.angularVelocity();
    float curTheta = _odom.pose().theta;

    VelocityCommand cmd = _dwa.compute(curV, curW, curTheta, _lidar, dt);

    // Convert to wheel velocities
    float wL, wR;
    FliwaDWA::toWheelVelocities(cmd.v, cmd.omega, wL, wR);
    _motor.setWheelVelocities(wL, wR);

    // Stuck detection
    if (isStuck()) {
        Serial.println(F("[ROBOT] Stuck detected → RECOVER"));
        _motor.stop();
        _state = RobotState::RECOVER;
        _stateEntryMs       = millis();
        _recoveryScanDone   = false;
        _recoveryScanStartMs = millis();
    }
}

// ============================================================================
//  RECOVER state — escape from dead ends / corners
//
//  Phase 1 (RECOVERY_SCAN_MS): Stop and let lidar complete a full rotation
//          for a clean 360° scan.
//  Phase 2: Find the largest gap in the scan.
//  Phase 3: Rotate in place toward the gap center.
//  Phase 4: Drive straight briefly, then transition back to EXPLORE.
// ============================================================================
void FliwaRobot::stateRecover(float dt) {
    uint32_t elapsed = millis() - _stateEntryMs;

    // Phase 1: Scan pause
    if (!_recoveryScanDone) {
        _motor.stop();
        if (millis() - _recoveryScanStartMs >= RECOVERY_SCAN_MS) {
            // Analyze the scan for the largest gap
            float gapWidth = 0.0f;
            float gapCenter = _lidar.findLargestGap(
                ROBOT_FRONT_X_MM + DWA_SAFETY_MARGIN_MM + 100.0f,
                gapWidth, 600);

            if (gapWidth < RECOVERY_MIN_GAP_DEG) {
                // Very tight — try backing up
                Serial.println(F("[RECOVER] No gap found, backing up"));
                _motor.setWheelVelocities(-3.0f, -3.0f);  // slow reverse
                _recoveryScanDone = true;
                _stateEntryMs = millis();
                return;
            }

            // Convert gap center (lidar degrees) to robot heading (radians)
            // Lidar 0° = forward, CW positive → negate for math CCW convention
            float gapRad = -gapCenter * ((float)M_PI / 180.0f);
            _recoveryTargetHeading = _odom.pose().theta + gapRad;

            // Normalize
            while (_recoveryTargetHeading >  (float)M_PI) _recoveryTargetHeading -= 2.0f * (float)M_PI;
            while (_recoveryTargetHeading < -(float)M_PI) _recoveryTargetHeading += 2.0f * (float)M_PI;

            Serial.print(F("[RECOVER] Gap found: "));
            Serial.print(gapWidth, 0);
            Serial.print(F("° at lidar "));
            Serial.print(gapCenter, 0);
            Serial.println(F("°"));

            _recoveryScanDone = true;
            _stateEntryMs = millis();
        }
        return;
    }

    // Phase 2: Rotate toward gap
    float headingError = _recoveryTargetHeading - _odom.pose().theta;
    while (headingError >  (float)M_PI) headingError -= 2.0f * (float)M_PI;
    while (headingError < -(float)M_PI) headingError += 2.0f * (float)M_PI;

    if (fabsf(headingError) > 0.15f && elapsed < 4000) {
        // Proportional rotation
        float omega = 2.0f * headingError;
        if (omega >  OP_MAX_W_RS) omega =  OP_MAX_W_RS;
        if (omega < -OP_MAX_W_RS) omega = -OP_MAX_W_RS;

        float wL, wR;
        FliwaDWA::toWheelVelocities(0.0f, omega, wL, wR);
        _motor.setWheelVelocities(wL, wR);
        return;
    }

    // Phase 3: Brief forward burst
    if (elapsed < 5500) {
        float wL, wR;
        FliwaDWA::toWheelVelocities(OP_MAX_V_MS * 0.5f, 0.0f, wL, wR);
        _motor.setWheelVelocities(wL, wR);
        return;
    }

    // Done → back to EXPLORE
    _stuckCheckPose = _odom.pose();
    _stuckCheckMs   = millis();
    _state = RobotState::EXPLORE;
    _stateEntryMs = millis();
    Serial.println(F("[ROBOT] Recovery complete → EXPLORE"));
}

// ============================================================================
//  ESTOP state — motors braked, do nothing until resume()
// ============================================================================
void FliwaRobot::stateEstop() {
    // Motors already stopped via emergencyStop()
    // Just wait for resume()
}

// ============================================================================
//  isStuck() — returns true if robot hasn't moved significantly
// ============================================================================
bool FliwaRobot::isStuck() const {
    uint32_t elapsed = millis() - _stuckCheckMs;
    if (elapsed < STUCK_TIMEOUT_MS) return false;

    Pose2D cur = _odom.pose();
    float dx = cur.x - _stuckCheckPose.x;
    float dy = cur.y - _stuckCheckPose.y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < STUCK_THRESHOLD_M) {
        return true;
    }

    // Not stuck — reset the check window
    // (const_cast needed because this is a const method but we want to
    //  reset the monitoring window on success)
    FliwaRobot* self = const_cast<FliwaRobot*>(this);
    self->_stuckCheckPose = cur;
    self->_stuckCheckMs   = millis();
    return false;
}

// ============================================================================
//  computeExplorationHeading()
//
//  STRATEGY: Go straight whenever possible!
//
//  1. Check if the forward cone (±FORWARD_HALF_CONE_DEG) is clear of obstacles
//     closer than FORWARD_CLEAR_DIST_MM.
//  2. If clear → return current heading (keep going straight).
//  3. If blocked → find the largest gap and steer toward it, but choose the
//     gap closest to "straight ahead" if multiple large gaps exist.
// ============================================================================
float FliwaRobot::computeExplorationHeading() {
    if (!_lidarOk) {
        return _odom.pose().theta;
    }

    // --- Check if forward path is clear ---
    bool forwardBlocked = false;
    for (int deg = 0; deg < 360; ++deg) {
        // Lidar convention: 0° = forward, CW positive.
        // Check if this degree is within the forward cone.
        float d = (float)deg;
        if (d > 180.0f) d -= 360.0f;  // normalize to -180..+180
        if (fabsf(d) > FORWARD_HALF_CONE_DEG) continue;

        const ScanPoint& sp = _lidar.bin(deg);
        uint32_t age = millis() - sp.stamp_ms;
        if (sp.valid && age < 400 && sp.dist_mm > 0.0f
            && sp.dist_mm < FORWARD_CLEAR_DIST_MM) {
            forwardBlocked = true;
            break;
        }
    }

    // Forward is clear → keep current heading, go straight!
    if (!forwardBlocked) {
        return _odom.pose().theta;
    }

    // --- Forward blocked: find best gap ---
    float gapWidth = 0.0f;
    float gapCenterDeg = _lidar.findLargestGap(
        ROBOT_FRONT_X_MM + DWA_SAFETY_MARGIN_MM,
        gapWidth, 400);

    // Convert lidar-frame gap center to relative angle.
    // Lidar 0° = forward, CW positive. Normalize to -180..+180.
    float relDeg = gapCenterDeg;
    if (relDeg > 180.0f) relDeg -= 360.0f;

    // Negate because lidar CW → our convention CCW
    float gapRelRad = -relDeg * ((float)M_PI / 180.0f);

    return _odom.pose().theta + gapRelRad;
}

// ============================================================================
//  updateTelemetry() — fill snapshot and pass to logger thread
// ============================================================================
void FliwaRobot::updateTelemetry() {
    TelemetryData td;
    Pose2D p = _odom.pose();

    td.x_m        = p.x;
    td.y_m        = p.y;
    td.theta_rad  = p.theta;
    td.v_ms       = _odom.linearVelocity();
    td.omega_rs   = _odom.angularVelocity();
    td.pwm_left   = _motor.lastPwmLeft();
    td.pwm_right  = _motor.lastPwmRight();
    td.target_v   = _motor.targetLeftRadS();   // approximate
    td.target_w   = _motor.targetRightRadS();
    td.nearest_mm = _lidar.nearestObstacleMm(400);
    td.state      = (uint8_t)_state;
    td.dwa_score  = _dwa.lastBestScore();
    td.uptime_ms  = millis();

    _logger.updateTelemetry(td);
}

} // namespace fliwa
