#include "FliwaOdometry.h"

namespace fliwa {

// ============================================================================
//  Quadrature Encoder state Machine lookup table
//  index = (prevState << 2) | newState
//  Forward: 00->10->11->01->00 = +1 per step
// ============================================================================
const int8_t FliwaOdometry::QEM[16] = {
//  prev\new:  00   01   10   11
/* 00 */        0,  -1,  +1,   0,
/* 01 */       +1,   0,   0,  -1,
/* 10 */       -1,   0,   0,  +1,
/* 11 */        0,  +1,  -1,   0
};

// Static volatile encoder state
volatile long    FliwaOdometry::_encL      = 0;
volatile long    FliwaOdometry::_encR      = 0;
volatile uint8_t FliwaOdometry::_encLState = 0;
volatile uint8_t FliwaOdometry::_encRState = 0;

// ============================================================================
//  Ticker ISR — runs every ENC_POLL_INTERVAL_US microseconds
//  Reads encoder pins and updates counts via quadrature decode.
//  Must be short: no heap, no Serial, no delay.
// ============================================================================
void FliwaOdometry::encoderISR() {
    // Left encoder
    uint8_t la    = digitalRead(PIN_ENC_L_A) ? 1u : 0u;
    uint8_t lb    = digitalRead(PIN_ENC_L_B) ? 1u : 0u;
    uint8_t newLS = (la << 1) | lb;
    int8_t  dL    = QEM[(_encLState << 2) | newLS];
    _encLState    = newLS;
    _encL        += ENC_L_INVERT ? -dL : dL;

    // Right encoder
    uint8_t ra    = digitalRead(PIN_ENC_R_A) ? 1u : 0u;
    uint8_t rb    = digitalRead(PIN_ENC_R_B) ? 1u : 0u;
    uint8_t newRS = (ra << 1) | rb;
    int8_t  dR    = QEM[(_encRState << 2) | newRS];
    _encRState    = newRS;
    _encR        += ENC_R_INVERT ? -dR : dR;
}

// ============================================================================
//  Constructor
// ============================================================================
FliwaOdometry::FliwaOdometry()
    : _prevEncL(0), _prevEncR(0), _prevTimeUs(0),
      _leftRadS(0.0f), _rightRadS(0.0f) {}

// ============================================================================
//  begin() — set up pins and start Ticker
// ============================================================================
void FliwaOdometry::begin() {
    // Encoder input pins
    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_L_B, INPUT_PULLUP);
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_B, INPUT_PULLUP);

    // Seed state machines from current pin levels
    _encLState = ((digitalRead(PIN_ENC_L_A) ? 1u : 0u) << 1)
               |  (digitalRead(PIN_ENC_L_B) ? 1u : 0u);
    _encRState = ((digitalRead(PIN_ENC_R_A) ? 1u : 0u) << 1)
               |  (digitalRead(PIN_ENC_R_B) ? 1u : 0u);
    _encL = 0;
    _encR = 0;

    _prevEncL = 0;
    _prevEncR = 0;
    _prevTimeUs = micros();

    _pose = Pose2D{};
    _leftRadS  = 0.0f;
    _rightRadS = 0.0f;

    // Start encoder polling Ticker (mbed 6 API)
    _ticker.attach(encoderISR, std::chrono::microseconds(ENC_POLL_INTERVAL_US));
}

// ============================================================================
//  update() — integrate odometry from encoder deltas
//  Call at ~50 Hz from the navigation loop.
//  Returns dt in seconds.
// ============================================================================
float FliwaOdometry::update() {
    // Atomic snapshot of encoder counts
    long encL, encR;
    noInterrupts();
    encL = _encL;
    encR = _encR;
    interrupts();

    uint32_t nowUs = micros();
    float dt = (float)(nowUs - _prevTimeUs) / 1e6f;
    if (dt < 1e-6f) dt = 1e-6f;  // guard against zero

    // Encoder deltas (counts since last update)
    long dL = encL - _prevEncL;
    long dR = encR - _prevEncR;
    _prevEncL = encL;
    _prevEncR = encR;
    _prevTimeUs = nowUs;

    // Convert counts to wheel angular displacement (radians)
    float dThetaL = (float)dL * (2.0f * (float)M_PI / ENC_CPR_OUTPUT);
    float dThetaR = (float)dR * (2.0f * (float)M_PI / ENC_CPR_OUTPUT);

    // Wheel angular velocities (rad/s) — simple low-pass filter
    const float alpha = 0.3f;  // smoothing factor
    _leftRadS  = alpha * (dThetaL / dt) + (1.0f - alpha) * _leftRadS;
    _rightRadS = alpha * (dThetaR / dt) + (1.0f - alpha) * _rightRadS;

    // Wheel linear displacements (meters)
    float dSL = dThetaL * WHEEL_RADIUS_M;
    float dSR = dThetaR * WHEEL_RADIUS_M;

    // Differential drive odometry (mid-point integration)
    float dS     = (dSR + dSL) / 2.0f;
    float dTheta = (dSR - dSL) / WHEEL_BASE_M;

    float midTheta = _pose.theta + dTheta / 2.0f;
    _pose.x     += dS * cosf(midTheta);
    _pose.y     += dS * sinf(midTheta);
    _pose.theta += dTheta;

    // Normalize theta to [-PI, PI]
    while (_pose.theta >  (float)M_PI) _pose.theta -= 2.0f * (float)M_PI;
    while (_pose.theta < -(float)M_PI) _pose.theta += 2.0f * (float)M_PI;

    return dt;
}

// ============================================================================
//  Accessors
// ============================================================================

Pose2D FliwaOdometry::pose() const {
    return _pose;
}

void FliwaOdometry::resetPose(float x, float y, float theta) {
    _pose.x     = x;
    _pose.y     = y;
    _pose.theta = theta;
}

float FliwaOdometry::linearVelocity() const {
    return (_leftRadS + _rightRadS) * WHEEL_RADIUS_M / 2.0f;
}

float FliwaOdometry::angularVelocity() const {
    return (_rightRadS - _leftRadS) * WHEEL_RADIUS_M / WHEEL_BASE_M;
}

void FliwaOdometry::encoderCounts(long& left, long& right) const {
    noInterrupts();
    left  = _encL;
    right = _encR;
    interrupts();
}

} // namespace fliwa
