#include "revolute/core/dynamics.hpp"

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

int advance_simulation_clock(SimulationClock& clock, double elapsed, double dt, int max_updates) {
    if (!std::isfinite(elapsed)
        || elapsed < 0
        || !std::isfinite(dt)
        || dt <= 0
        || max_updates <= 0) {
        throw std::invalid_argument("Invalid simulation timing inputs");
    }
    if (!std::isfinite(clock.accumulator)
        || clock.accumulator < 0
        || !std::isfinite(clock.dropped_time)
        || clock.dropped_time < 0) {
        throw std::invalid_argument("Invalid simulation clock state");
    }
    double accumulated = clock.accumulator + elapsed;
    if (!std::isfinite(accumulated)) {
        throw std::overflow_error("Accumulated simulation time overflowed");
    }

    // Treat differences below one trillionth of a step as rounding noise.
    // Otherwise 30 FPS and 60 FPS can disagree by one update at a step boundary.
    const double tolerance = dt * 1e-12;
    int updates = 0;
    while (accumulated >= dt || dt - accumulated <= tolerance) {
        if (updates == max_updates) {
            break;
        }
        accumulated = std::max(0.0, accumulated - dt);
        ++updates;
    }

    double dropped = 0.0;
    if (accumulated >= dt || dt - accumulated <= tolerance) {
        double remainder = std::fmod(accumulated, dt);
        if (dt - remainder <= tolerance) {
            remainder = 0.0;
        }
        dropped = accumulated - remainder;
        accumulated = remainder;
    }
    if (!std::isfinite(clock.dropped_time + dropped)) {
        throw std::overflow_error("Dropped simulation time overflowed");
    }
    clock.accumulator = accumulated;
    clock.dropped_time += dropped;
    return updates;
}

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
