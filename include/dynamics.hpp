#pragma once

namespace revolute {
struct JointState {
    double angle;
    double angular_velocity;
};

struct JointParameters {
    double inertia;
    double damping;
    double torque_limit;
};

// Advance one timestep using semi-implicit Euler.
JointState step_joint(
    JointState state,
    const JointParameters& parameters,
    double commanded_torque,
    double dt
);
} // namespace revolute
