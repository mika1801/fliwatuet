#include "FliwaDWA.h"
#include <cmath>
#include <cfloat>

namespace fliwa {

FliwaDWA::FliwaDWA()
    : _goalHeading(0.0f), _lastBestScore(-1.0f),
      _prevBestV(0.0f), _prevBestOmega(0.0f) {}

void FliwaDWA::setGoalHeading(float headingRad) {
    _goalHeading = headingRad;
}

// ============================================================================
//  compute() — DWA main loop
//
//  1. Define the dynamic window around current (v, omega) bounded by
//     acceleration limits and hardware limits.
//  2. Sample the window in a grid.
//  3. For each (v_cand, omega_cand):
//     a. Simulate forward trajectory for DWA_HORIZON_S.
//     b. Check rectangular collision against all valid lidar points.
//     c. Score the trajectory.
//  4. Return the (v, omega) with the highest score.
// ============================================================================
VelocityCommand FliwaDWA::compute(float currentV, float currentOmega,
                                   float currentTheta,
                                   const FliwaLidar& lidar, float dt) {
    // --- Dynamic window bounds ---
    float vMin = fmaxf(OP_MIN_V_MS, currentV - OP_MAX_ACC_MS2 * dt);
    float vMax = fminf(OP_MAX_V_MS, currentV + OP_MAX_ACC_MS2 * dt);
    float wMin = fmaxf(-OP_MAX_W_RS, currentOmega - OP_MAX_ANG_ACC_RS2 * dt);
    float wMax = fminf( OP_MAX_W_RS, currentOmega + OP_MAX_ANG_ACC_RS2 * dt);

    // --- Collect valid scan points ---
    ScanPoint scanBuf[FliwaLidar::NUM_BINS];
    int numPoints = lidar.getValidPoints(scanBuf, FliwaLidar::NUM_BINS, 400);

    // --- Grid search ---
    float bestScore = -FLT_MAX;
    float bestV     = 0.0f;
    float bestW     = 0.0f;

    float vStep = (DWA_V_SAMPLES > 1) ? (vMax - vMin) / (float)(DWA_V_SAMPLES - 1) : 0.0f;
    float wStep = (DWA_W_SAMPLES > 1) ? (wMax - wMin) / (float)(DWA_W_SAMPLES - 1) : 0.0f;

    for (int iv = 0; iv < DWA_V_SAMPLES; ++iv) {
        float vCand = vMin + (float)iv * vStep;

        for (int iw = 0; iw < DWA_W_SAMPLES; ++iw) {
            float wCand = wMin + (float)iw * wStep;

            // --- Collision check along trajectory ---
            float clearance = checkTrajectoryCollision(
                vCand, wCand, DWA_HORIZON_S, scanBuf, numPoints);

            if (clearance <= 0.0f) continue;  // collision — skip

            // --- Score ---
            float score = scoreCandidate(vCand, wCand, currentTheta,
                                          _goalHeading, clearance);

            if (score > bestScore) {
                bestScore = score;
                bestV     = vCand;
                bestW     = wCand;
            }
        }
    }

    _lastBestScore = bestScore;
    _prevBestV     = bestV;
    _prevBestOmega = bestW;

    VelocityCommand cmd;
    cmd.v     = bestV;
    cmd.omega = bestW;
    return cmd;
}

// ============================================================================
//  toWheelVelocities() — differential drive inverse kinematics
//  v     = (R * wL + R * wR) / 2
//  omega = (R * wR - R * wL) / L
//  => wL = (v - omega * L/2) / R
//  => wR = (v + omega * L/2) / R
// ============================================================================
void FliwaDWA::toWheelVelocities(float v, float omega,
                                  float& leftRadS, float& rightRadS) {
    leftRadS  = (v - omega * WHEEL_BASE_M / 2.0f) / WHEEL_RADIUS_M;
    rightRadS = (v + omega * WHEEL_BASE_M / 2.0f) / WHEEL_RADIUS_M;
}

// ============================================================================
//  checkTrajectoryCollision()
//
//  Simulates the robot moving along a circular arc at (v, omega) for horizonS.
//  At each time step, transforms all scan points from the CURRENT robot frame
//  into the PREDICTED robot frame and checks against the inflated rectangle.
//
//  The predicted pose at time t relative to current pose:
//    If omega ≈ 0 (straight line):
//      dx = v * t,  dy = 0,  dθ = 0
//    Else (circular arc):
//      dθ = omega * t
//      dx = (v / omega) * sin(dθ)
//      dy = (v / omega) * (1 - cos(dθ))
//
//  A scan point P in the current frame is transformed to predicted frame:
//    P' = R(-dθ) * (P - [dx, dy])
//
//  Then we check if P' is inside the inflated robot rectangle.
//
//  Returns minimum clearance (mm) across all time steps and points.
//  Returns 0 if collision detected.
// ============================================================================
float FliwaDWA::checkTrajectoryCollision(float v, float omega, float horizonS,
                                          const ScanPoint* points, int numPoints) const {
    if (numPoints == 0) return DWA_OBSTACLE_COST_RADIUS_MM;  // no data = assume free

    // Inflated robot hull edges
    const float frontX = ROBOT_FRONT_X_MM + DWA_SAFETY_MARGIN_MM;
    const float backX  = ROBOT_BACK_X_MM  - DWA_SAFETY_MARGIN_MM;
    const float leftY  = ROBOT_LEFT_Y_MM  + DWA_SAFETY_MARGIN_MM;
    const float rightY = ROBOT_RIGHT_Y_MM - DWA_SAFETY_MARGIN_MM;

    float minClearance = FLT_MAX;

    const int numSteps = (int)(horizonS / DWA_DT_S);

    for (int step = 1; step <= numSteps; ++step) {
        float t = (float)step * DWA_DT_S;

        // Predicted displacement from current pose
        float dx, dy, dTheta;
        if (fabsf(omega) < 1e-4f) {
            // Straight line
            dx     = v * t;
            dy     = 0.0f;
            dTheta = 0.0f;
        } else {
            // Circular arc
            dTheta = omega * t;
            dx     = (v / omega) * sinf(dTheta);
            dy     = (v / omega) * (1.0f - cosf(dTheta));
        }

        // Convert displacement to mm
        float dx_mm = dx * 1000.0f;
        float dy_mm = dy * 1000.0f;

        float cosNeg = cosf(-dTheta);
        float sinNeg = sinf(-dTheta);

        // Check every scan point against the predicted robot pose
        for (int i = 0; i < numPoints; ++i) {
            // Point in current robot frame (mm)
            float px = points[i].x_mm;
            float py = points[i].y_mm;

            // Transform to predicted robot frame:
            //   P' = R(-dTheta) * (P - displacement)
            float relX = px - dx_mm;
            float relY = py - dy_mm;
            float predX = cosNeg * relX - sinNeg * relY;
            float predY = sinNeg * relX + cosNeg * relY;

            // Distance to inflated rectangle edge
            float dist = pointToRectDistance(predX, predY,
                                             frontX, backX, leftY, rightY);

            if (dist <= 0.0f) return 0.0f;  // collision!

            if (dist < minClearance) {
                minClearance = dist;
            }
        }
    }

    return (minClearance < FLT_MAX) ? minClearance : DWA_OBSTACLE_COST_RADIUS_MM;
}

// ============================================================================
//  pointToRectDistance()
//
//  Computes the signed distance from point (px, py) to the nearest edge
//  of an axis-aligned rectangle defined by [backX..frontX] x [rightY..leftY].
//
//  Returns:
//    > 0 : point is OUTSIDE the rectangle, value = distance to nearest edge
//    ≤ 0 : point is INSIDE the rectangle (collision)
// ============================================================================
float FliwaDWA::pointToRectDistance(float px_mm, float py_mm,
                                    float frontX, float backX,
                                    float leftY, float rightY) {
    // Distances to each edge (positive = outside that edge, negative = inside)
    float dFront = px_mm - frontX;   // positive if in front of front edge
    float dBack  = backX - px_mm;    // positive if behind back edge
    float dLeft  = py_mm - leftY;    // positive if left of left edge
    float dRight = rightY - py_mm;   // positive if right of right edge

    // If point is inside the rectangle, all four are negative
    bool insideX = (dFront <= 0.0f) && (dBack <= 0.0f);
    bool insideY = (dLeft  <= 0.0f) && (dRight <= 0.0f);

    if (insideX && insideY) {
        // Point is inside — return negative penetration depth
        float maxPen = fmaxf(fmaxf(dFront, dBack), fmaxf(dLeft, dRight));
        return maxPen;  // negative value
    }

    // Point is outside — compute distance to nearest edge/corner
    float clampedX = px_mm;
    if (clampedX > frontX) clampedX = frontX;
    if (clampedX < backX)  clampedX = backX;

    float clampedY = py_mm;
    if (clampedY > leftY)  clampedY = leftY;
    if (clampedY < rightY) clampedY = rightY;

    float ex = px_mm - clampedX;
    float ey = py_mm - clampedY;
    return sqrtf(ex * ex + ey * ey);
}

// ============================================================================
//  scoreCandidate()
//
//  Cost function with four weighted components:
//
//  1. HEADING:    How well the predicted heading aligns with the goal direction.
//                 Score = 1.0 - |angle_diff| / PI  (normalized 0..1)
//
//  2. VELOCITY:   Prefer higher forward speed for efficient exploration.
//                 Score = v / OP_MAX_V_MS  (normalized 0..1)
//
//  3. OBSTACLE:   Prefer trajectories with more clearance from obstacles.
//                 Score = min(clearance, DWA_OBSTACLE_COST_RADIUS_MM)
//                         / DWA_OBSTACLE_COST_RADIUS_MM  (normalized 0..1)
//
//  4. SMOOTHNESS: Penalize large changes from previous best command.
//                 Score = 1.0 - |omega - prevOmega| / (2*OP_MAX_W_RS)
// ============================================================================
float FliwaDWA::scoreCandidate(float v, float omega,
                                float currentTheta, float goalHeading,
                                float clearanceMm) const {
    // 1. Heading alignment
    float predictedTheta = currentTheta + omega * DWA_HORIZON_S;
    float headingDiff = predictedTheta - goalHeading;
    // Normalize to [-PI, PI]
    while (headingDiff >  (float)M_PI) headingDiff -= 2.0f * (float)M_PI;
    while (headingDiff < -(float)M_PI) headingDiff += 2.0f * (float)M_PI;
    float headingScore = 1.0f - fabsf(headingDiff) / (float)M_PI;

    // 2. Velocity preference
    float velScore = 0.0f;
    if (OP_MAX_V_MS > 0.0f) {
        velScore = fmaxf(v, 0.0f) / OP_MAX_V_MS;
    }

    // 3. Obstacle clearance
    float clampedClearance = fminf(clearanceMm, DWA_OBSTACLE_COST_RADIUS_MM);
    float obstScore = clampedClearance / DWA_OBSTACLE_COST_RADIUS_MM;

    // 4. Smoothness
    float omegaDiff = fabsf(omega - _prevBestOmega);
    float smoothScore = 1.0f - omegaDiff / (2.0f * OP_MAX_W_RS);
    if (smoothScore < 0.0f) smoothScore = 0.0f;

    return DWA_WEIGHT_HEADING    * headingScore
         + DWA_WEIGHT_VELOCITY   * velScore
         + DWA_WEIGHT_OBSTACLE   * obstScore
         + DWA_WEIGHT_SMOOTHNESS * smoothScore;
}

} // namespace fliwa
