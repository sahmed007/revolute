#include "revolute/core/kinematics.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
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

double calculate_shoulder_angle(
    double upper_arm_length,
    double forearm_length,
    Point2D target,
    double elbow_angle
) {
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

void validate_numerical_ik_inputs(
    double upper_arm_length,
    double forearm_length,
    Point2D target,
    JointAngles initial_angles,
    double position_tolerance,
    int max_iterations,
    double max_step,
    std::optional<JointLimits> limits,
    double damping
) {
    validate_arm_configuration(
        upper_arm_length,
        forearm_length,
        initial_angles.shoulder,
        initial_angles.elbow
    );

    if (!std::isfinite(damping) || damping <= 0.0) {
        throw std::invalid_argument("damping must be finite and greater than zero");
    }

    if (!std::isfinite(max_step) || max_step <= 0.0) {
        throw std::invalid_argument("max_step must be finite and greater than zero");
    }

    if (!std::isfinite(position_tolerance) || position_tolerance <= 0.0) {
        throw std::invalid_argument("position_tolerance must be finite and greater than zero");
    }

    if (max_iterations < 0) {
        throw std::invalid_argument("max_iterations must be greater than or equal to zero");
    }

    if (!std::isfinite(target.x) || !std::isfinite(target.y)) {
        throw std::invalid_argument("Target coordinates must be finite");
    }

    if (limits) {
        if (!std::isfinite(limits->minimum.shoulder)
            || !std::isfinite(limits->minimum.elbow)
            || !std::isfinite(limits->maximum.shoulder)
            || !std::isfinite(limits->maximum.elbow)
            || limits->minimum.shoulder > limits->maximum.shoulder
            || limits->minimum.elbow > limits->maximum.elbow) {
            throw std::invalid_argument("Joint limits must be finite and ordered");
        }
        if (!within_joint_limits(initial_angles, *limits)) {
            throw std::invalid_argument("Initial angles must be within joint limits");
        }
    }
}

std::optional<Eigen::Vector2d> bounded_joint_correction(
    const Eigen::Matrix2d& J,
    const Eigen::Vector2d& error,
    double max_step,
    double damping
) {
    if (!J.allFinite()) {
        return std::nullopt;
    }

    Eigen::Matrix2d system = J.transpose() * J + damping * damping * Eigen::Matrix2d::Identity();
    Eigen::Vector2d rhs = J.transpose() * error;
    if (!system.allFinite() || !rhs.allFinite()) {
        return std::nullopt;
    }

    Eigen::LDLT<Eigen::Matrix2d> decomposition(system);
    if (decomposition.info() != Eigen::Success
        || !decomposition.isPositive()
        || (decomposition.vectorD().array() <= 0.0).any()) {
        return std::nullopt;
    }

    Eigen::Vector2d correction = decomposition.solve(rhs);
    double correction_length = correction.stableNorm();

    if (!correction.allFinite() || !std::isfinite(correction_length)) {
        return std::nullopt;
    }
    if (correction_length > max_step) {
        correction *= max_step / correction_length;
    }
    return correction;
}

std::optional<NumericalIKResult> check_termination(
    JointAngles angles,
    double position_error,
    double position_tolerance,
    int iterations,
    int max_iterations
) {
    if (!std::isfinite(position_error)) {
        return NumericalIKResult{
            angles,
            NumericalIKStatus::numerical_failure,
            iterations,
            std::numeric_limits<double>::infinity()
        };
    }
    if (position_error <= position_tolerance) {
        return NumericalIKResult{angles, NumericalIKStatus::converged, iterations, position_error};
    }
    if (iterations >= max_iterations) {
        return NumericalIKResult{
            angles,
            NumericalIKStatus::iteration_limit,
            iterations,
            position_error
        };
    }
    return std::nullopt;
}

std::optional<JointAngles> next_joint_angles(
    double upper_arm_length,
    double forearm_length,
    JointAngles angles,
    const Eigen::Vector2d& error,
    double max_step,
    double damping
) {
    const auto J = jacobian(upper_arm_length, forearm_length, angles.shoulder, angles.elbow);
    const auto correction = bounded_joint_correction(J, error, max_step, damping);
    if (!correction) {
        return std::nullopt;
    }

    JointAngles candidate{angles.shoulder + (*correction)(0), angles.elbow + (*correction)(1)};
    if (!std::isfinite(candidate.shoulder)
        || !std::isfinite(candidate.elbow)
        || !std::isfinite(candidate.shoulder + candidate.elbow)) {
        return std::nullopt;
    }
    return candidate;
}

} // namespace

Eigen::Matrix2d jacobian(
    double upper_arm_length,
    double forearm_length,
    double shoulder_angle,
    double elbow_angle
) {
    validate_arm_configuration(upper_arm_length, forearm_length, shoulder_angle, elbow_angle);
    Eigen::Matrix2d j = Eigen::Matrix2d::Zero();
    j(0, 0) = -upper_arm_length * std::sin(shoulder_angle)
        - forearm_length * std::sin(shoulder_angle + elbow_angle);
    j(0, 1) = -forearm_length * std::sin(shoulder_angle + elbow_angle);
    j(1, 0) = upper_arm_length * std::cos(shoulder_angle)
        + forearm_length * std::cos(shoulder_angle + elbow_angle);
    j(1, 1) = forearm_length * std::cos(shoulder_angle + elbow_angle);
    return j;
}

ArmPose forward_kinematics(
    double upper_arm_length,
    double forearm_length,
    double shoulder_angle,
    double elbow_angle
) {
    validate_arm_configuration(upper_arm_length, forearm_length, shoulder_angle, elbow_angle);
    Point2D elbow = calculate_endpoint(upper_arm_length, shoulder_angle);
    Point2D forearm_offset = calculate_endpoint(forearm_length, shoulder_angle + elbow_angle);
    Point2D hand = {elbow.x + forearm_offset.x, elbow.y + forearm_offset.y};
    return ArmPose{{0, 0}, elbow, hand};
}

bool reachable(double upper_arm_length, double forearm_length, Point2D target) {
    // To calculate reachability, do the following:
    // abs(L1 - L2) <= r <= L1 + L2, where r = sqrt(x**2 + y**2) and assuming non-negative link
    // lengths
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
        double shoulder_angle =
            calculate_shoulder_angle(upper_arm_length, forearm_length, target, elbow_angle);
        double normalized_angle = normalize_angle(shoulder_angle);
        result.solutions.push_back({normalized_angle, elbow_angle});

        // at extension or folding, both branches describe the same pose
        if (cos_elbow == 1.0 || cos_elbow == -1.0) {
            break;
        }
    }
    return result;
}

NumericalIKResult num_inverse_kinematics(
    double upper_arm_length,
    double forearm_length,
    Point2D target,
    JointAngles initial_angles,
    double position_tolerance,
    int max_iterations,
    double max_step,
    std::optional<JointLimits> limits,
    double damping
) {
    validate_numerical_ik_inputs(
        upper_arm_length,
        forearm_length,
        target,
        initial_angles,
        position_tolerance,
        max_iterations,
        max_step,
        limits,
        damping
    );
    JointAngles angles = initial_angles;
    int iterations = 0;

    while (true) {
        ArmPose pose =
            forward_kinematics(upper_arm_length, forearm_length, angles.shoulder, angles.elbow);
        Eigen::Vector2d error{target.x - pose.hand.x, target.y - pose.hand.y};
        double position_error = std::hypot(error(0), error(1));

        if (auto result = check_termination(
                angles,
                position_error,
                position_tolerance,
                iterations,
                max_iterations
            )) {
            return *result;
        }

        auto candidate =
            next_joint_angles(upper_arm_length, forearm_length, angles, error, max_step, damping);
        if (!candidate) {
            return {angles, NumericalIKStatus::numerical_failure, iterations, position_error};
        }

        if (limits && !within_joint_limits(*candidate, *limits)) {
            return {angles, NumericalIKStatus::joint_limit, iterations, position_error};
        }

        angles = *candidate;
        ++iterations;
    }
}
} // namespace revolute
