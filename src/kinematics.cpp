#include "kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace revolute {

namespace {
Point2D calculate_endpoint(double link_length, double radians) {
    // to obtain endpoint coordinates for a given length, do:
    // x = L * cos(angle) and y = L * sin(angle)
    double x = link_length * std::cos(radians);
    double y = link_length * std::sin(radians);
    return Point2D{x, y};
}
} // namespace

ArmPose forward_kinematics(double upper_arm_length, double forearm_length, double shoulder_angle, double elbow_angle) {
    if (upper_arm_length <= 0 || forearm_length <= 0) {
        throw std::invalid_argument("Lengths must be greater than zero");
    }

    if (!std::isfinite(upper_arm_length) || !std::isfinite(forearm_length)) {
        throw std::invalid_argument("Lengths must be finite");
    }

    if (!std::isfinite(shoulder_angle) || !std::isfinite(elbow_angle)) {
        throw std::invalid_argument("Angles must be finite");
    }

    Point2D elbow = calculate_endpoint(upper_arm_length, shoulder_angle);
    Point2D forearm_offset = calculate_endpoint(forearm_length, shoulder_angle + elbow_angle);
    Point2D hand = {elbow.x + forearm_offset.x, elbow.y + forearm_offset.y};
    return ArmPose{{0, 0}, elbow, hand};
}

bool reachable(double upper_arm_length, double forearm_length, Point2D target) {
    // To calculate reachability, do the following:
    // abs(L1 - L2) <= r <= L1 + L2, where r = sqrt(x**2 + y**2) and assuming non-negative link lengths
    if (upper_arm_length <= 0 || forearm_length <= 0) {
        throw std::invalid_argument("Lengths must be greater than zero");
    }

    if (!std::isfinite(upper_arm_length) || !std::isfinite(forearm_length)) {
        throw std::invalid_argument("Lengths must be finite");
    }

    if (!std::isfinite(target.x) || !std::isfinite(target.y)) {
        throw std::invalid_argument("Target coordinates must be finite");
    }

    double target_distance = std::sqrt(pow(target.x, 2) + pow(target.y, 2));
    double minimum = std::abs(upper_arm_length - forearm_length);
    double maximum = upper_arm_length + forearm_length;
    return target_distance == std::clamp(target_distance, minimum, maximum);
}
} // namespace revolute
