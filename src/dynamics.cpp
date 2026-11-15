#include "dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace revolute {

namespace {
void validate_joint_configurations(
    JointState state,
    const JointParameters& parameters,
    double commanded_torque,
    double dt
) {
    if (parameters.inertia <= 0 || !std::isfinite(parameters.inertia)) {
        throw std::invalid_argument("Inertia must be finite and greater than zero");
    }

    if (dt <= 0 || !std::isfinite(dt)) {
        throw std::invalid_argument("Timestep must be finite and greater than zero");
    }

    if (parameters.torque_limit < 0 || !std::isfinite(parameters.torque_limit)) {
        throw std::invalid_argument("Torque limit must be finite and non-negative");
    }

    if (parameters.damping < 0 || !std::isfinite(parameters.damping)) {
        throw std::invalid_argument("Damping must be finite and non-negative");
    }

    if (!std::isfinite(state.angle)
        || !std::isfinite(state.angular_velocity)
        || !std::isfinite(commanded_torque)) {
        throw std::invalid_argument("Angle, angular velocity, and commanded torque must be finite");
    }
}
} // namespace

JointState step_joint(
    JointState state,
    const JointParameters& parameters,
    double commanded_torque,
    double dt
) {
    validate_joint_configurations(state, parameters, commanded_torque, dt);

    double applied_torque =
        std::clamp(commanded_torque, -parameters.torque_limit, parameters.torque_limit);
    double angular_acceleration =
        (applied_torque - (parameters.damping * state.angular_velocity)) / parameters.inertia;

    state.angular_velocity += angular_acceleration * dt;
    state.angle += state.angular_velocity * dt;
    return state;
}
} // namespace revolute
