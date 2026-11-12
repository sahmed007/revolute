#include "kinematics.hpp"

#include "Eigen/src/Core/Matrix.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <stdexcept>

namespace revolute {

namespace {

void validate_arm_configuration(
    double upper_arm_length,
    double forearm_length,
    double shoulder_angle,
    double elbow_angle
) {
    if (upper_arm_length <= 0 || forearm_length <= 0) {
        throw std::invalid_argument("Lengths must be greater than zero");
    }

    if (!std::isfinite(upper_arm_length) || !std::isfinite(forearm_length)) {
        throw std::invalid_argument("Lengths must be finite");
    }

    if (!std::isfinite(shoulder_angle) || !std::isfinite(elbow_angle)) {
        throw std::invalid_argument("Angles must be finite");
    }
}

double calculate_shoulder_angle(double upper_arm_length, double forearm_length, Point2D target, double elbow_angle) {
    // hand position relative to the base when the shoulder angle is zero
    double local_x = upper_arm_length + forearm_length * std::cos(elbow_angle);
    double local_y = forearm_length * std::sin(elbow_angle);

    // rotate the bent arm from its local direction toward the target
    double target_direction = std::atan2(target.y, target.x);
    double local_direction = std::atan2(local_y, local_x);
    return target_direction - local_direction;
}

double normalize_angle(double angle) {
    return std::remainder(angle, 2.0 * std::numbers::pi);
}

Point2D calculate_endpoint(double link_length, double radians) {
    // to obtain endpoint coordinates for a given length, do:
    // x = L * cos(angle) and y = L * sin(angle)
    double x = link_length * std::cos(radians);
    double y = link_length * std::sin(radians);
    return Point2D{x, y};
}
} // namespace

Eigen::Matrix2d jacobian(double upper_arm_length, double forearm_length, double shoulder_angle, double elbow_angle) {
    validate_arm_configuration(upper_arm_length, forearm_length, shoulder_angle, elbow_angle);

    Eigen::Matrix2d j = Eigen::Matrix2d::Zero();
    j(0, 0) = -upper_arm_length * std::sin(shoulder_angle) - forearm_length * std::sin(shoulder_angle + elbow_angle);
    j(0, 1) = -forearm_length * std::sin(shoulder_angle + elbow_angle);
    j(1, 0) = upper_arm_length * std::cos(shoulder_angle) + forearm_length * std::cos(shoulder_angle + elbow_angle);
    j(1, 1) = forearm_length * std::cos(shoulder_angle + elbow_angle);
    return j;
}

ArmPose forward_kinematics(double upper_arm_length, double forearm_length, double shoulder_angle, double elbow_angle) {
    validate_arm_configuration(upper_arm_length, forearm_length, shoulder_angle, elbow_angle);

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

bool within_joint_limits(JointAngles angles, JointLimits limits) {
    return angles.shoulder >= limits.minimum.shoulder
        && angles.shoulder <= limits.maximum.shoulder
        && angles.elbow >= limits.minimum.elbow
        && angles.elbow <= limits.maximum.elbow;
}

IKResult inverse_kinematics(double upper_arm_length, double forearm_length, Point2D target) {
    if (!reachable(upper_arm_length, forearm_length, target)) {
        return {IKStatus::unreachable, {}};
    }

    // equal links folded at the origin allow any shoulder angle, choose zero as a representative
    if (upper_arm_length == forearm_length && target.x == 0.0 && target.y == 0.0) {
        return {IKStatus::success, {{0.0, std::numbers::pi}}};
    }

    double target_distance_squared = target.x * target.x + target.y * target.y;
    double upper_arm_length_squared = upper_arm_length * upper_arm_length;
    double forearm_length_squared = forearm_length * forearm_length;
    double numerator = target_distance_squared - upper_arm_length_squared - forearm_length_squared;
    double denominator = 2 * upper_arm_length * forearm_length;
    double cos_elbow = numerator / denominator;

    constexpr double cosine_tolerance = 1e-12;
    if (cos_elbow < -1.0 - cosine_tolerance || cos_elbow > 1.0 + cosine_tolerance) {
        return {IKStatus::unreachable, {}};
    }

    cos_elbow = std::clamp(cos_elbow, -1.0, 1.0);
    double elbow_magnitude = std::acos(cos_elbow);
    IKResult result{IKStatus::success, {}};

    for (double elbow_angle : {elbow_magnitude, -elbow_magnitude}) {
        double shoulder_angle = calculate_shoulder_angle(upper_arm_length, forearm_length, target, elbow_angle);
        double normalized_angle = normalize_angle(shoulder_angle);
        result.solutions.push_back({normalized_angle, elbow_angle});

        // at extension or folding, both branches describe the same pose
        if (cos_elbow == 1.0 || cos_elbow == -1.0) {
            break;
        }
    }
    return result;
}
} // namespace revolute
