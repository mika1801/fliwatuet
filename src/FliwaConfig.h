#pragma once
// ============================================================================
//  FliwaConfig.h — Central configuration for the Fliwatuet autonomous robot
//  Board : Arduino GIGA R1 WiFi (STM32H747, Dual-Core M7+M4, Mbed OS)
//  Shield: Arduino Motor Shield Rev3
//  LiDAR : RPLidar A1M8 (360° 2D)
//  Motors: 2x Pololu 4751 (19:1 Metal Gearmotor 12V 37Dx68L, 64 CPR encoder)
// ============================================================================

#include <cstdint>
#include <cmath>

namespace fliwa {

// ============================================================================
//  Motor Shield Rev3 — Pin definitions
//  Channel A = Left motor, Channel B = Right motor
// ============================================================================
constexpr uint8_t PIN_MOTOR_L_DIR   = 12;   // Channel A direction
constexpr uint8_t PIN_MOTOR_L_PWM   =  3;   // Channel A PWM
constexpr uint8_t PIN_MOTOR_L_BRAKE =  9;   // Channel A brake

constexpr uint8_t PIN_MOTOR_R_DIR   = 13;   // Channel B direction
constexpr uint8_t PIN_MOTOR_R_PWM   = 11;   // Channel B PWM
constexpr uint8_t PIN_MOTOR_R_BRAKE =  8;   // Channel B brake

// Software inversion flags (from MotorEncoderTest calibration)
constexpr bool MOTOR_L_INVERT = true;
constexpr bool MOTOR_R_INVERT = false;

// ============================================================================
//  Encoder pins & parameters
// ============================================================================
constexpr uint8_t PIN_ENC_L_A = 4;
constexpr uint8_t PIN_ENC_L_B = 5;
constexpr uint8_t PIN_ENC_R_A = 6;
constexpr uint8_t PIN_ENC_R_B = 7;

constexpr bool ENC_L_INVERT = true;
constexpr bool ENC_R_INVERT = false;

constexpr uint32_t ENC_POLL_INTERVAL_US = 40;       // 25 kHz polling via Ticker
constexpr float    ENC_CPR_MOTOR        = 64.0f;    // counts per motor shaft revolution
constexpr float    ENC_GEAR_RATIO       = 19.0f;    // gearbox ratio
constexpr float    ENC_CPR_OUTPUT       = ENC_CPR_MOTOR * ENC_GEAR_RATIO;  // 1216

// ============================================================================
//  RPLidar A1M8 — Pin definitions
// ============================================================================
constexpr uint8_t PIN_LIDAR_MOTOR_CTRL = 2;
constexpr uint32_t LIDAR_BAUDRATE      = 115200;
constexpr uint8_t  LIDAR_MOTOR_PWM     = 160;
constexpr bool     LIDAR_MOTOR_ACTIVE_HIGH = true;
// Serial1 is used (D0=RX, D1=TX) — set in code, not as a constexpr

// ============================================================================
//  Robot geometry — all dimensions in mm
//
//  Origin (0,0) = center of drive axle
//  x = forward (Fahrtrichtung), y = left, z = up
//
//       y+113
//         |
//  x-23.81 +------+------+ x+178.38
//         |  (0,0)       |
//  x-23.81 +------+------+ x+178.38
//         |
//       y-113
// ============================================================================
constexpr float ROBOT_LENGTH_MM    = 202.19f;
constexpr float ROBOT_WIDTH_MM     = 226.0f;

constexpr float ROBOT_FRONT_X_MM  = 178.38f;   // front edge x
constexpr float ROBOT_BACK_X_MM   = -23.81f;   // rear edge x
constexpr float ROBOT_LEFT_Y_MM   = 113.0f;    // left edge y
constexpr float ROBOT_RIGHT_Y_MM  = -113.0f;   // right edge y

// LiDAR sensor center offset relative to robot origin
constexpr float LIDAR_OFFSET_X_MM = 80.86f;
constexpr float LIDAR_OFFSET_Y_MM = 0.0f;

// ============================================================================
//  Wheel geometry
// ============================================================================
constexpr float WHEEL_DIAMETER_MM  = 70.0f;     // Pololu 70mm wheel
constexpr float WHEEL_RADIUS_MM    = WHEEL_DIAMETER_MM / 2.0f;
constexpr float WHEEL_BASE_MM      = 210.0f;    // distance between wheel centers

// Meters for kinematic calculations
constexpr float WHEEL_RADIUS_M    = WHEEL_RADIUS_MM / 1000.0f;
constexpr float WHEEL_BASE_M      = WHEEL_BASE_MM / 1000.0f;

// ============================================================================
//  Kinematic limits
// ============================================================================
constexpr float MAX_WHEEL_RPM      = 530.0f;    // Pololu 4751 no-load at 12V
constexpr float MAX_WHEEL_RAD_S    = MAX_WHEEL_RPM * 2.0f * (float)M_PI / 60.0f;
constexpr float MAX_LINEAR_V_MS    = MAX_WHEEL_RAD_S * WHEEL_RADIUS_M;  // ~1.94 m/s
constexpr float MAX_ANGULAR_W_RS   = 2.0f * MAX_LINEAR_V_MS / WHEEL_BASE_M;

// Practical operating limits (conservative for indoor use)
constexpr float OP_MAX_V_MS        = 0.40f;     // max forward speed
constexpr float OP_MIN_V_MS        = -0.15f;    // max reverse speed
constexpr float OP_MAX_W_RS        = 2.0f;      // max angular velocity
constexpr float OP_MAX_ACC_MS2     = 0.8f;      // max linear acceleration
constexpr float OP_MAX_ANG_ACC_RS2 = 3.0f;      // max angular acceleration

// ============================================================================
//  PID controller gains (velocity control per wheel)
// ============================================================================
constexpr float PID_KP = 2.5f;
constexpr float PID_KI = 8.0f;
constexpr float PID_KD = 0.05f;
constexpr float PID_INTEGRAL_LIMIT = 150.0f;    // anti-windup clamp
constexpr uint32_t PID_INTERVAL_MS = 20;         // 50 Hz control loop

// ============================================================================
//  DWA planner parameters
// ============================================================================
constexpr float DWA_DT_S           = 0.1f;      // simulation time step
constexpr float DWA_HORIZON_S      = 1.5f;      // prediction horizon
constexpr int   DWA_V_SAMPLES      = 11;         // velocity samples
constexpr int   DWA_W_SAMPLES      = 21;         // angular velocity samples
constexpr float DWA_SAFETY_MARGIN_MM = 50.0f;    // clearance around robot hull

// Cost function weights — velocity is king for straight-line exploration
constexpr float DWA_WEIGHT_HEADING    = 0.6f;   // alignment to goal direction
constexpr float DWA_WEIGHT_VELOCITY   = 1.8f;   // strongly prefer high forward speed
constexpr float DWA_WEIGHT_OBSTACLE   = 1.5f;   // obstacle clearance
constexpr float DWA_WEIGHT_SMOOTHNESS = 0.4f;   // prefer smooth transitions

constexpr float DWA_OBSTACLE_COST_RADIUS_MM = 800.0f;  // range within which obstacles contribute cost

// ============================================================================
//  Exploration & Recovery
// ============================================================================
constexpr float STUCK_THRESHOLD_M     = 0.03f;  // moved less than 30mm
constexpr uint32_t STUCK_TIMEOUT_MS   = 4000;    // over 4 seconds = stuck
constexpr uint32_t RECOVERY_SCAN_MS   = 500;     // pause for full lidar rotation
constexpr float RECOVERY_MIN_GAP_DEG  = 30.0f;   // minimum gap width to attempt escape

// Forward-clear detection: if no obstacle in a ±HALF_CONE_DEG cone
// closer than FORWARD_CLEAR_DIST_MM, the path ahead is "open" → go straight.
constexpr float FORWARD_CLEAR_DIST_MM  = 600.0f;  // 60 cm
constexpr float FORWARD_HALF_CONE_DEG  = 35.0f;   // ±35° from dead ahead

// ============================================================================
//  WiFi UDP Logging
// ============================================================================
constexpr uint16_t LOG_UDP_PORT       = 9000;
constexpr uint32_t LOG_INTERVAL_MS    = 100;     // 10 Hz telemetry
constexpr size_t   LOG_BUFFER_SIZE    = 512;

// ============================================================================
//  Timing
// ============================================================================
constexpr uint32_t NAV_LOOP_INTERVAL_MS = 50;    // 20 Hz navigation loop
constexpr uint32_t LIDAR_READ_TIMEOUT_MS = 5;

} // namespace fliwa
