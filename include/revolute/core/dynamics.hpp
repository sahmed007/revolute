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

struct SimulationClock {
    double accumulator = 0.0;
    double dropped_time = 0.0;
};

// Consume elapsed seconds and return the number of fixed updates to run this frame.
int advance_simulation_clock(SimulationClock& clock, double elapsed, double dt, int max_updates);

// Advance one timestep using semi-implicit Euler.
JointState step_joint(
    JointState state,
    const JointParameters& parameters,
    double commanded_torque,
    double dt
);
} // namespace revolute
