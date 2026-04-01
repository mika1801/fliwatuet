#pragma once
// ============================================================================
//  FliwaMotor.h — Motor driver + PID velocity controller
//  Drives two DC motors via Arduino Motor Shield Rev3.
//  Each wheel has an independent PID loop controlling wheel velocity (rad/s).
// ============================================================================

#include <Arduino.h>
#include "FliwaConfig.h"

namespace fliwa {

// ============================================================================
//  PID controller for a single wheel
// ============================================================================
class PIDController {
public:
    PIDController(float kp, float ki, float kd, float integralLimit);

    /// Compute control output given setpoint and measurement (both in rad/s).
    /// dt is the elapsed time in seconds since last call.
    float compute(float setpoint, float measurement, float dt);

    void reset();

private:
    float _kp, _ki, _kd;
    float _integralLimit;
    float _integral;
    float _prevError;
    bool  _firstCall;
};

// ============================================================================
//  Single motor channel (direction + PWM + brake)
// ============================================================================
struct MotorChannel {
    uint8_t pinDir;
    uint8_t pinPwm;
    uint8_t pinBrake;
    bool    invert;
};

// ============================================================================
//  FliwaMotor — manages both drive motors with PID velocity control
// ============================================================================
class FliwaMotor {
public:
    FliwaMotor();

    void begin();

    /// Set raw PWM for a motor (-255..+255, negative = reverse).
    /// Use only for testing; prefer setWheelVelocities() for navigation.
    void setRawPWM(int leftPwm, int rightPwm);

    /// Set desired wheel angular velocities in rad/s.
    /// Call updatePID() periodically to close the loop.
    void setWheelVelocities(float leftRadS, float rightRadS);

    /// Update PID controllers using measured wheel velocities.
    /// measuredLeftRadS / measuredRightRadS come from FliwaOdometry.
    void updatePID(float measuredLeftRadS, float measuredRightRadS, float dt);

    /// Hard brake both motors immediately.
    void emergencyStop();

    /// Soft stop (set target velocities to zero, let PID ramp down).
    void stop();

    float targetLeftRadS()  const { return _targetL; }
    float targetRightRadS() const { return _targetR; }
    int   lastPwmLeft()     const { return _lastPwmL; }
    int   lastPwmRight()    const { return _lastPwmR; }

private:
    void applyMotor(const MotorChannel& ch, int pwm);
    void brakeMotor(const MotorChannel& ch);

    MotorChannel _left;
    MotorChannel _right;
    PIDController _pidL;
    PIDController _pidR;

    float _targetL;   // desired rad/s
    float _targetR;
    int   _lastPwmL;
    int   _lastPwmR;
    bool  _emergencyStopped;
};

} // namespace fliwa
