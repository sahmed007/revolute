#pragma once

namespace revolute {

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

} // namespace revolute
