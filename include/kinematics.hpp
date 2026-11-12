#pragma once

#include <Eigen/Core>
#include <vector>

namespace revolute {
enum class IKStatus {
    success,
    unreachable
};

struct Point2D {
    double x;
    double y;
};

struct ArmPose {
    Point2D base;
    Point2D elbow;
    Point2D hand;
};

struct JointAngles {
    double shoulder;
    double elbow;
};

struct JointLimits {
    JointAngles minimum;
    JointAngles maximum;
};

struct IKResult {
    IKStatus status;
    std::vector<JointAngles> solutions;
};

// Calculate the 2×2 hand-position Jacobian, with x/y rows and shoulder/elbow columns, in meters per radian.
Eigen::Matrix2d jacobian(double upper_arm_length, double forearm_length, double shoulder_angle, double elbow_angle);

// Calculate base, elbow, and hand positions in meters from link lengths and joint angles in radians.
ArmPose forward_kinematics(double upper_arm_length, double forearm_length, double shoulder_angle, double elbow_angle);

// Check whether the target lies within the arm's geometric reach, independent of joint limits.
bool reachable(double upper_arm_length, double forearm_length, Point2D target);

// Check whether both joint angles lie within their inclusive minimum and maximum bounds in radians.
bool within_joint_limits(JointAngles angles, JointLimits limits);

// Find candidate joint angles in radians for a target in meters, without enforcing joint limits.
IKResult inverse_kinematics(double upper_arm_length, double forearm_length, Point2D target);
} // namespace revolute
