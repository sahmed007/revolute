#include "revolute/core/control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace revolute {

namespace {
void validate_pid_inputs(
    const ControllerState& state,
    const PIDGains& gains,
    double target_angle,
    double current_angle,
    double angular_velocity,
    double dt,
    double torque_limit
) {
    if (!std::isfinite(target_angle)
        || !std::isfinite(current_angle)
        || !std::isfinite(angular_velocity)
        || !std::isfinite(state.integral_error)) {
        throw std::invalid_argument("Angles, angular velocity, and integral error must be finite");
    }
    if (!std::isfinite(gains.kp)
        || gains.kp < 0.0
        || !std::isfinite(gains.ki)
        || gains.ki < 0.0
        || !std::isfinite(gains.kd)
        || gains.kd < 0.0) {
        throw std::invalid_argument("PID gains must be finite and nonnegative");
    }
    if (!std::isfinite(torque_limit) || torque_limit < 0.0) {
        throw std::invalid_argument("Torque limit must be finite and nonnegative");
    }
    if (!std::isfinite(dt) || dt <= 0.0) {
        throw std::invalid_argument("Timestep must be finite and greater than zero");
    }
}
} // namespace

double proportional_torque(double target_angle, double current_angle, double gain) {
    if (!std::isfinite(target_angle) || !std::isfinite(current_angle)) {
        throw std::invalid_argument("Target and current angles must be finite");
    }
    if (!std::isfinite(gain) || gain < 0.0) {
        throw std::invalid_argument("Gain must be finite and nonnegative");
    }
    return gain * (target_angle - current_angle);
}

double pd_torque(
    double target_angle,
    double current_angle,
    double angular_velocity,
    double kp,
    double kd
) {
    const double proportional = proportional_torque(target_angle, current_angle, kp);
    if (!std::isfinite(angular_velocity)) {
        throw std::invalid_argument("Angular velocity must be finite");
    }
    if (!std::isfinite(kd) || kd < 0.0) {
        throw std::invalid_argument("Derivative gain must be finite and nonnegative");
    }
    return proportional - kd * angular_velocity;
}

double pid_torque(
    ControllerState& state,
    const PIDGains& gains,
    double target_angle,
    double current_angle,
    double angular_velocity,
    double dt,
    double torque_limit
) {
    validate_pid_inputs(
        state,
        gains,
        target_angle,
        current_angle,
        angular_velocity,
        dt,
        torque_limit
    );
    double error = target_angle - current_angle;
    double candidate_integral = state.integral_error + error * dt;

    double candidate_torque =
        gains.kp * error + gains.ki * candidate_integral - gains.kd * angular_velocity;

    bool blocks_integration = (candidate_torque > torque_limit && error > 0)
        || (candidate_torque < -torque_limit && error < 0);

    if (!blocks_integration) {
        state.integral_error = candidate_integral;
    }

    double accepted_torque =
        gains.kp * error + gains.ki * state.integral_error - gains.kd * angular_velocity;

    return std::clamp(accepted_torque, -torque_limit, torque_limit);
}

} // namespace revolute
