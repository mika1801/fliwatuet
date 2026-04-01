#include "FliwaMotor.h"

namespace fliwa {

// ============================================================================
//  PIDController
// ============================================================================

PIDController::PIDController(float kp, float ki, float kd, float integralLimit)
    : _kp(kp), _ki(ki), _kd(kd), _integralLimit(integralLimit),
      _integral(0.0f), _prevError(0.0f), _firstCall(true) {}

float PIDController::compute(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    // Integral with anti-windup
    _integral += error * dt;
    if (_integral >  _integralLimit) _integral =  _integralLimit;
    if (_integral < -_integralLimit) _integral = -_integralLimit;

    // Derivative (skip on first call to avoid spike)
    float derivative = 0.0f;
    if (!_firstCall) {
        derivative = (error - _prevError) / dt;
    }
    _firstCall = false;
    _prevError = error;

    return _kp * error + _ki * _integral + _kd * derivative;
}

void PIDController::reset() {
    _integral  = 0.0f;
    _prevError = 0.0f;
    _firstCall = true;
}

// ============================================================================
//  FliwaMotor
// ============================================================================

FliwaMotor::FliwaMotor()
    : _pidL(PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT),
      _pidR(PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT),
      _targetL(0.0f), _targetR(0.0f),
      _lastPwmL(0), _lastPwmR(0),
      _emergencyStopped(false) {

    _left  = { PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, PIN_MOTOR_L_BRAKE, MOTOR_L_INVERT };
    _right = { PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, PIN_MOTOR_R_BRAKE, MOTOR_R_INVERT };
}

void FliwaMotor::begin() {
    // Configure motor output pins
    pinMode(_left.pinDir,   OUTPUT);
    pinMode(_left.pinPwm,   OUTPUT);
    pinMode(_left.pinBrake, OUTPUT);
    pinMode(_right.pinDir,  OUTPUT);
    pinMode(_right.pinPwm,  OUTPUT);
    pinMode(_right.pinBrake,OUTPUT);

    // Start in braked state
    brakeMotor(_left);
    brakeMotor(_right);

    _pidL.reset();
    _pidR.reset();
    _emergencyStopped = false;
}

void FliwaMotor::setRawPWM(int leftPwm, int rightPwm) {
    _emergencyStopped = false;
    applyMotor(_left,  leftPwm);
    applyMotor(_right, rightPwm);
    _lastPwmL = leftPwm;
    _lastPwmR = rightPwm;
}

void FliwaMotor::setWheelVelocities(float leftRadS, float rightRadS) {
    _targetL = leftRadS;
    _targetR = rightRadS;
    _emergencyStopped = false;
}

void FliwaMotor::updatePID(float measuredLeftRadS, float measuredRightRadS, float dt) {
    if (_emergencyStopped) return;

    float outL = _pidL.compute(_targetL, measuredLeftRadS, dt);
    float outR = _pidR.compute(_targetR, measuredRightRadS, dt);

    // Convert PID output to PWM (-255..+255)
    // PID output is roughly in rad/s units; scale to PWM.
    // MAX_WHEEL_RAD_S maps to ~255 PWM.
    float scale = 255.0f / MAX_WHEEL_RAD_S;
    int pwmL = (int)constrain(outL * scale, -255.0f, 255.0f);
    int pwmR = (int)constrain(outR * scale, -255.0f, 255.0f);

    // Dead-band: if target is zero and we're close, stop completely
    if (fabsf(_targetL) < 0.05f && fabsf(measuredLeftRadS) < 0.1f) {
        pwmL = 0;
        _pidL.reset();
    }
    if (fabsf(_targetR) < 0.05f && fabsf(measuredRightRadS) < 0.1f) {
        pwmR = 0;
        _pidR.reset();
    }

    applyMotor(_left,  pwmL);
    applyMotor(_right, pwmR);
    _lastPwmL = pwmL;
    _lastPwmR = pwmR;
}

void FliwaMotor::emergencyStop() {
    brakeMotor(_left);
    brakeMotor(_right);
    _targetL = 0.0f;
    _targetR = 0.0f;
    _lastPwmL = 0;
    _lastPwmR = 0;
    _pidL.reset();
    _pidR.reset();
    _emergencyStopped = true;
}

void FliwaMotor::stop() {
    _targetL = 0.0f;
    _targetR = 0.0f;
}

// ---- Private helpers ----

void FliwaMotor::applyMotor(const MotorChannel& ch, int pwm) {
    if (pwm == 0) {
        brakeMotor(ch);
        return;
    }

    bool forward = (pwm > 0);
    if (ch.invert) forward = !forward;

    uint8_t absPwm = (uint8_t)min(abs(pwm), 255);

    digitalWrite(ch.pinBrake, LOW);
    digitalWrite(ch.pinDir,   forward ? HIGH : LOW);
    analogWrite(ch.pinPwm,    absPwm);
}

void FliwaMotor::brakeMotor(const MotorChannel& ch) {
    analogWrite(ch.pinPwm, 0);
    digitalWrite(ch.pinBrake, HIGH);
}

} // namespace fliwa
