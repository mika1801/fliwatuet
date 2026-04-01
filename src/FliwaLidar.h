#pragma once
// ============================================================================
//  FliwaLidar.h — RPLidar A1M8 scan manager with coordinate transformation
//
//  Wraps RPLidarA1Giga and maintains a 360-bin distance map.
//  All points are transformed from lidar-local to robot-body coordinates
//  using the sensor offset (LIDAR_OFFSET_X_MM, LIDAR_OFFSET_Y_MM).
// ============================================================================

#include <Arduino.h>
#include "FliwaConfig.h"
#include "RPLidarA1Giga.h"

namespace fliwa {

// ============================================================================
//  A single scan point in robot-body coordinates (mm)
// ============================================================================
struct ScanPoint {
    float x_mm    = 0.0f;   // robot frame x (forward)
    float y_mm    = 0.0f;   // robot frame y (left)
    float dist_mm = 0.0f;   // raw distance from sensor
    float angle_deg = 0.0f; // lidar angle in degrees
    uint8_t quality = 0;
    uint32_t stamp_ms = 0;
    bool valid = false;
};

// ============================================================================
//  FliwaLidar — manages lidar scanning and provides obstacle data
// ============================================================================
class FliwaLidar {
public:
    static constexpr int NUM_BINS = 360;

    FliwaLidar();

    /// Initialize lidar on the given serial port.
    /// Returns false if device info or scan start fails.
    bool begin(HardwareSerial& serial);

    /// Read available measurements and update the scan map.
    /// Call frequently (every nav loop iteration).
    /// Returns number of points ingested this call.
    int update();

    /// Stop scanning and motor.
    void stop();

    /// Access the current scan bin at a given degree (0..359).
    const ScanPoint& bin(int degree) const;

    /// Get all valid scan points newer than maxAgeMs as a flat array.
    /// Returns the number of valid points written into `out`.
    /// `outCapacity` should be >= NUM_BINS.
    int getValidPoints(ScanPoint* out, int outCapacity, uint32_t maxAgeMs = 400) const;

    /// Find the largest angular gap (in degrees) with no obstacles closer than minDistMm.
    /// Returns the center angle (degrees, lidar convention) and writes the gap width.
    float findLargestGap(float minDistMm, float& gapWidthDeg, uint32_t maxAgeMs = 400) const;

    /// Nearest obstacle distance in mm (from entire 360° scan).
    float nearestObstacleMm(uint32_t maxAgeMs = 400) const;

    bool isRunning() const { return _lidar.isScanning(); }

private:
    /// Transform a lidar measurement (polar, sensor-local) into robot-body XY.
    void transformToRobotFrame(float angleDeg, float distMm,
                               float& outX, float& outY) const;

    RPLidarA1Giga _lidar;
    ScanPoint     _bins[NUM_BINS];
};

} // namespace fliwa
