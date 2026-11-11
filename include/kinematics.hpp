#pragma once

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

ArmPose forward_kinematics(double upper_arm_length, double forearm_length, double shoulder_angle, double elbow_angle);

bool reachable(double upper_arm_length, double forearm_length, Point2D target);

bool within_joint_limits(JointAngles angles, JointLimits limits);

IKResult inverse_kinematics(double upper_arm_length, double forearm_length, Point2D target);
} // namespace revolute
