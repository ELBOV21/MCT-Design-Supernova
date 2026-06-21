#ifndef TRAJECTORY_PLANNING_H
#define TRAJECTORY_PLANNING_H

#include <math.h>
#include "Config/Config.h"

// ---------------------------------------------------------------------------
// Pose definitions
// ---------------------------------------------------------------------------

/** @brief A snapshot of all joint angles at a single configuration. */
typedef struct {
    float angles[CFG_NUM_JOINTS];  ///< One entry per joint (no gripper)
} Pose;

/**
 * @brief Named pose identifiers.
 *
 * The numerical value is used as an index into the poses[] table defined in
 * TrajectoryPlanning.cpp.  Add new poses here AND to that table.
 */
typedef enum {
    HOME = 0,
    PICK,
    GRIP_POSE,
    RIGHT,
    RIGHT_UP,
    RIGHT_PLACE,
    LEFT,
    LEFT_UP,
    LEFT_PLACE,
    DROP,
    MIDDLE,
    MIDDLE_UP,
    MIDDLE_PLACE,
    CAPTURE,
    PLACEONPEDESTAL,
    NUM_POSES   ///used for table-size assertions
} PoseID;

/** @brief A single step in a motion sequence (from → to, with duration). */
typedef struct {
    PoseID from;
    PoseID to;
    float  duration;  ///< Motion time in seconds
} MotionStep;

// ---------------------------------------------------------------------------
// Pose table (defined in TrajectoryPlanning.cpp)
// ---------------------------------------------------------------------------
extern Pose poses[NUM_POSES];

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief Quintic (5th-order) polynomial trajectory interpolation.
 *
 * Produces a smooth position profile with zero velocity and acceleration at
 * both endpoints.
 *
 * @param t   Elapsed time (s)
 * @param T   Total motion duration (s)
 * @param q0  Start angle (degrees)
 * @param qf  End angle (degrees)
 * @return    Interpolated angle (degrees)
 */
float quinticTrajectory(float t, float T, float q0, float qf);

/**
 * @brief Move all joints from a named start pose to a named end pose.
 *
 * @param from  Starting pose ID (must be an element of poses[])
 * @param to    Target pose ID
 * @param T     Motion duration in seconds
 */
void movePose(PoseID from, PoseID to, float T);

/**
 * @brief Move all joints from the current live position to a named pose.
 *
 * Reads the live joint_angles[] array as the starting configuration, so
 * the motion always begins exactly where the arm currently is.
 *
 * @param to  Target pose ID
 * @param T   Motion duration in seconds
 */
void moveFromCurrentPoseToTargetPose(PoseID to, float T);


#endif