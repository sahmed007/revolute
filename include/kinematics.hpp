#pragma once

namespace revolute {
struct Point2D {
    double x;
    double y;
};

struct ArmPose {
    Point2D base;
    Point2D elbow;
    Point2D hand;
};

ArmPose forward_kinematics(double upper_arm_length, double forearm_length, double shoulder_angle, double elbow_angle);

bool reachable(double upper_arm_length, double forearm_length, Point2D target);
} // namespace revolute
