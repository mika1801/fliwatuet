#pragma once
// ============================================================================
//  FliwaRobot.h — Top-level state machine orchestrator
//
//  States:
//    INIT     — hardware bringup, wait for lidar + encoders
//    EXPLORE  — normal autonomous navigation via DWA
//    RECOVER  — stuck/cornered, perform recovery maneuver
//    ESTOP    — emergency stop (manual trigger or critical fault)
// ============================================================================

#include <Arduino.h>
#include "FliwaConfig.h"
#include "FliwaMotor.h"
#include "FliwaOdometry.h"
#include "FliwaLidar.h"
#include "FliwaDWA.h"
#include "FliwaLogger.h"

namespace fliwa {

enum class RobotState : uint8_t {
    INIT    = 0,
    EXPLORE = 1,
    RECOVER = 2,
    ESTOP   = 3,
};

const char* robotStateName(RobotState s);

// ============================================================================
//  FliwaRobot
// ============================================================================
class FliwaRobot {
public:
    FliwaRobot();

    /// Full initialization: motors, encoders, lidar, optional WiFi logging.
    /// Call once in setup().
    bool begin(const char* wifiSsid     = nullptr,
               const char* wifiPassword = nullptr,
               const char* logTargetIP  = nullptr);

    /// Main navigation tick — call in loop().
    /// Reads sensors, runs DWA, drives motors, updates telemetry.
    void update();

    /// Trigger emergency stop from external code.
    void emergencyStop();

    /// Resume from ESTOP → EXPLORE.
    void resume();

    // Accessors for diagnostics
    RobotState      state()    const { return _state; }
    const Pose2D&   pose()     const { return _odom.pose(); }
    FliwaMotor&     motor()          { return _motor; }
    FliwaOdometry&  odometry()       { return _odom; }
    FliwaLidar&     lidar()          { return _lidar; }
    FliwaDWA&       planner()        { return _dwa; }
    FliwaLogger&    logger()         { return _logger; }

private:
    void stateInit();
    void stateExplore(float dt);
    void stateRecover(float dt);
    void stateEstop();

    /// Check if robot is stuck (not making progress).
    bool isStuck() const;

    /// Compute exploration goal heading from lidar gap analysis.
    float computeExplorationHeading();

    /// Fill and send telemetry snapshot.
    void updateTelemetry();

    // Subsystems
    FliwaMotor    _motor;
    FliwaOdometry _odom;
    FliwaLidar    _lidar;
    FliwaDWA      _dwa;
    FliwaLogger   _logger;

    // State machine
    RobotState _state;
    uint32_t   _stateEntryMs;

    // Stuck detection
    Pose2D   _stuckCheckPose;
    uint32_t _stuckCheckMs;

    // Recovery state
    float    _recoveryTargetHeading;
    uint32_t _recoveryScanStartMs;
    bool     _recoveryScanDone;

    // Timing
    uint32_t _lastNavMs;
    uint32_t _lastPidMs;

    bool _lidarOk;
    bool _loggerOk;
};

} // namespace fliwa
