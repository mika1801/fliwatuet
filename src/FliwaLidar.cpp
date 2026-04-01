#include "FliwaLidar.h"
#include <cmath>

namespace fliwa {

FliwaLidar::FliwaLidar() {
    for (int i = 0; i < NUM_BINS; ++i) {
        _bins[i] = ScanPoint{};
    }
}

bool FliwaLidar::begin(HardwareSerial& serial) {
    _lidar.begin(serial, LIDAR_BAUDRATE, PIN_LIDAR_MOTOR_CTRL, LIDAR_MOTOR_ACTIVE_HIGH);
    _lidar.setMotorPwm(LIDAR_MOTOR_PWM);
    _lidar.motorOn();
    delay(200);

    RPLidarA1Giga::DeviceInfo info;
    if (!_lidar.getDeviceInfo(info, 300)) return false;

    RPLidarA1Giga::Health health;
    if (!_lidar.getHealth(health, 300)) return false;
    if (health.status == 2) return false;  // ERROR state

    if (!_lidar.startScan(false, 400)) return false;

    return true;
}

int FliwaLidar::update() {
    int count = 0;
    const uint32_t sliceStartUs = micros();

    // Read for up to 10ms per call to avoid blocking the nav loop
    while ((micros() - sliceStartUs) < 10000UL) {
        RPLidarA1Giga::Measurement m;
        if (!_lidar.readMeasurement(m, LIDAR_READ_TIMEOUT_MS)) continue;

        if (!m.valid || m.distanceMm == 0 || m.quality < 4) continue;

        // Bin index from angle
        int deg = (int)lroundf(m.angleDeg);
        if (deg < 0)   deg += 360;
        if (deg >= 360) deg -= 360;

        // Transform to robot body frame
        float rx, ry;
        transformToRobotFrame(m.angleDeg, (float)m.distanceMm, rx, ry);

        ScanPoint& sp = _bins[deg];
        sp.x_mm      = rx;
        sp.y_mm      = ry;
        sp.dist_mm   = (float)m.distanceMm;
        sp.angle_deg = m.angleDeg;
        sp.quality   = m.quality;
        sp.stamp_ms  = millis();
        sp.valid     = true;

        ++count;
    }

    return count;
}

void FliwaLidar::stop() {
    _lidar.stop(20);
    _lidar.motorOff();
}

const ScanPoint& FliwaLidar::bin(int degree) const {
    degree = ((degree % 360) + 360) % 360;
    return _bins[degree];
}

int FliwaLidar::getValidPoints(ScanPoint* out, int outCapacity, uint32_t maxAgeMs) const {
    uint32_t now = millis();
    int n = 0;
    for (int i = 0; i < NUM_BINS && n < outCapacity; ++i) {
        if (_bins[i].valid && (now - _bins[i].stamp_ms) <= maxAgeMs) {
            out[n++] = _bins[i];
        }
    }
    return n;
}

float FliwaLidar::findLargestGap(float minDistMm, float& gapWidthDeg, uint32_t maxAgeMs) const {
    uint32_t now = millis();

    // Build a boolean occupancy ring: true = obstacle present
    bool occupied[NUM_BINS];
    for (int i = 0; i < NUM_BINS; ++i) {
        const ScanPoint& sp = _bins[i];
        if (sp.valid && (now - sp.stamp_ms) <= maxAgeMs && sp.dist_mm < minDistMm) {
            occupied[i] = true;
        } else {
            occupied[i] = false;
        }
    }

    // Find the largest contiguous run of free bins
    int bestStart = 0;
    int bestLen   = 0;
    int curStart  = -1;
    int curLen    = 0;

    // Traverse twice to handle wrap-around
    for (int i = 0; i < 720; ++i) {
        int idx = i % 360;
        if (!occupied[idx]) {
            if (curLen == 0) curStart = idx;
            ++curLen;
            if (curLen > 360) curLen = 360;  // full circle free
        } else {
            if (curLen > bestLen) {
                bestLen   = curLen;
                bestStart = curStart;
            }
            curLen = 0;
        }
    }
    if (curLen > bestLen) {
        bestLen   = curLen;
        bestStart = curStart;
    }

    gapWidthDeg = (float)bestLen;

    // Return center of the gap
    float centerDeg = (float)bestStart + (float)bestLen / 2.0f;
    if (centerDeg >= 360.0f) centerDeg -= 360.0f;
    return centerDeg;
}

float FliwaLidar::nearestObstacleMm(uint32_t maxAgeMs) const {
    uint32_t now = millis();
    float nearest = 1e6f;
    for (int i = 0; i < NUM_BINS; ++i) {
        if (_bins[i].valid && (now - _bins[i].stamp_ms) <= maxAgeMs) {
            if (_bins[i].dist_mm > 0.0f && _bins[i].dist_mm < nearest) {
                nearest = _bins[i].dist_mm;
            }
        }
    }
    return nearest;
}

// ============================================================================
//  transformToRobotFrame()
//
//  The RPLidar reports angles in its own frame (0° = forward from sensor).
//  The sensor sits at (LIDAR_OFFSET_X_MM, LIDAR_OFFSET_Y_MM) in robot frame.
//
//  Steps:
//   1. Convert polar (angleDeg, distMm) to Cartesian in sensor-local frame.
//      RPLidar convention: 0° = forward, CW positive when viewed from above.
//      Our convention: 0° = forward (x+), CCW positive (y+ = left).
//      Therefore: lidar angle θ_lidar maps to mathematical angle -θ_lidar.
//
//   2. Translate by the sensor offset to get robot-body coordinates.
// ============================================================================
void FliwaLidar::transformToRobotFrame(float angleDeg, float distMm,
                                        float& outX, float& outY) const {
    // Convert RPLidar CW angle to math CCW angle
    float angleRad = -angleDeg * ((float)M_PI / 180.0f);

    // Point in sensor-local Cartesian
    float sx = distMm * cosf(angleRad);
    float sy = distMm * sinf(angleRad);

    // Translate to robot body frame
    outX = sx + LIDAR_OFFSET_X_MM;
    outY = sy + LIDAR_OFFSET_Y_MM;
}

} // namespace fliwa
