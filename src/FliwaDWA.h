#pragma once
// ============================================================================
//  FliwaDWA.h — Dynamic Window Approach local planner
//
//  The DWA evaluates a grid of (v, omega) candidates within the kinematically
//  feasible "dynamic window" around the current velocity. For each candidate
//  it simulates a short trajectory, checks collision against the rectangular
//  robot footprint using transformed lidar points, and scores the candidate
//  via a weighted cost function (heading, velocity, obstacle clearance, smoothness).
//
//  Collision checking explicitly models the robot as a rectangle (not a circle!)
//  and transforms each lidar scan point into the robot's predicted future frame
//  to test whether the point falls inside the inflated hull.
// ============================================================================

#include <Arduino.h>
#include "FliwaConfig.h"
#include "FliwaLidar.h"

namespace fliwa {

// ============================================================================
//  DWA output command
// ============================================================================
struct VelocityCommand {
    float v     = 0.0f;   // linear velocity (m/s)
    float omega = 0.0f;   // angular velocity (rad/s)
};

// ============================================================================
//  FliwaDWA
// ============================================================================
class FliwaDWA {
public:
    FliwaDWA();

    /// Set the desired heading angle for the robot in world frame (radians).
    /// In exploration mode this is typically the direction of the largest gap.
    void setGoalHeading(float headingRad);

    /// Compute the best (v, omega) command given current state and scan data.
    /// currentV / currentOmega: current robot velocity from odometry.
    /// currentTheta: current heading in world frame (radians).
    /// lidar: reference to the scan manager for obstacle data.
    /// dt: time since last call (seconds).
    VelocityCommand compute(float currentV, float currentOmega,
                            float currentTheta,
                            const FliwaLidar& lidar, float dt);

    /// Convert a (v, omega) command to left/right wheel angular velocities (rad/s).
    static void toWheelVelocities(float v, float omega,
                                  float& leftRadS, float& rightRadS);

    /// Last computed best score (for diagnostics).
    float lastBestScore() const { return _lastBestScore; }

private:
    // ========================================================================
    //  Rectangular collision check
    //
    //  Tests whether a scan point (in current robot frame, mm) would collide
    //  with the robot's rectangular footprint after the robot has moved along
    //  a circular arc defined by (v, omega) for time t.
    //
    //  The robot footprint is inflated by DWA_SAFETY_MARGIN_MM on all sides.
    //
    //  Returns the minimum clearance (mm) from the hull for the closest point.
    //  Returns 0 if any point is inside the hull (collision).
    // ========================================================================
    float checkTrajectoryCollision(float v, float omega, float horizonS,
                                   const ScanPoint* points, int numPoints) const;

    /// Check if a single point (robot-frame mm) is inside the inflated rectangle.
    /// Returns distance to nearest edge (negative = inside).
    static float pointToRectDistance(float px_mm, float py_mm,
                                     float frontX, float backX,
                                     float leftY, float rightY);

    /// Score a candidate trajectory.
    float scoreCandidate(float v, float omega,
                         float currentTheta, float goalHeading,
                         float clearanceMm) const;

    float _goalHeading;
    float _lastBestScore;
    float _prevBestV;
    float _prevBestOmega;
};

} // namespace fliwa
