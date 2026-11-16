#pragma once

namespace revolute {

struct ControllerState {
    double integral_error = 0.0;
};

struct PIDGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
};

// Return gain * (target_angle - current_angle) as commanded torque in N·m.
double proportional_torque(double target_angle, double current_angle, double gain);

// Return kp * (target_angle - current_angle) - kd * angular_velocity in N·m.
double pd_torque(
    double target_angle,
    double current_angle,
    double angular_velocity,
    double kp,
    double kd
);

// Update integral_error (rad·s) using dt, then return torque clamped to ±torque_limit (N·m).
double pid_torque(
    ControllerState& state,
    const PIDGains& gains,
    double target_angle,
    double current_angle,
    double angular_velocity,
    double dt,
    double torque_limit
);

} // namespace revolute
