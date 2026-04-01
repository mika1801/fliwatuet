#pragma once
// ============================================================================
//  FliwaOdometry.h — Encoder polling via mbed::Ticker + differential odometry
//
//  Encoder sampling runs in ISR context (Ticker at 25 kHz).
//  Odometry integration runs from the navigation loop via update().
// ============================================================================

#include <Arduino.h>
#include <mbed.h>
#include "FliwaConfig.h"

namespace fliwa {

// ============================================================================
//  Pose2D — robot pose in world frame
// ============================================================================
struct Pose2D {
    float x     = 0.0f;   // meters
    float y     = 0.0f;   // meters
    float theta = 0.0f;   // radians, CCW positive
};

// ============================================================================
//  FliwaOdometry
// ============================================================================
class FliwaOdometry {
public:
    FliwaOdometry();

    /// Initialize encoder pins and start the Ticker.
    void begin();

    /// Call periodically (e.g., 50 Hz) to integrate odometry.
    /// Returns elapsed dt in seconds.
    float update();

    /// Current pose estimate in world frame (meters, radians).
    Pose2D pose() const;

    /// Reset pose to a given value (default origin).
    void resetPose(float x = 0.0f, float y = 0.0f, float theta = 0.0f);

    /// Current wheel angular velocities in rad/s (filtered).
    float leftWheelRadS()  const { return _leftRadS; }
    float rightWheelRadS() const { return _rightRadS; }

    /// Robot-level velocities derived from wheel speeds.
    float linearVelocity()  const;   // m/s
    float angularVelocity() const;   // rad/s

    /// Raw encoder counts (atomic snapshot).
    void encoderCounts(long& left, long& right) const;

private:
    // Ticker callback — runs in ISR context, must be static
    static void encoderISR();

    // Quadrature lookup table
    static const int8_t QEM[16];

    // Volatile state written by ISR
    static volatile long    _encL;
    static volatile long    _encR;
    static volatile uint8_t _encLState;
    static volatile uint8_t _encRState;

    mbed::Ticker _ticker;

    // Odometry integration state
    Pose2D   _pose;
    long     _prevEncL;
    long     _prevEncR;
    uint32_t _prevTimeUs;

    // Filtered wheel velocities
    float _leftRadS;
    float _rightRadS;
};

} // namespace fliwa
