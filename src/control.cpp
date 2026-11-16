#include "control.hpp"

#include <cmath>
#include <stdexcept>

namespace revolute {

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

} // namespace revolute
