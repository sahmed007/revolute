#pragma once

#include "app_state.hpp"

namespace revolute {

// Solve the target into the editable pose without moving the simulated joints.
void solve_target(AppState& state);

// Advance both joints and controllers by TIME_STEP, updating torque readouts and time.
void advance_simulation(AppState& state);

// Apply the editable pose with zero joint velocities; caller enforces paused mode.
void apply_pose(AppState& state);

// Restore defaults and clear accumulated and dropped simulation time.
void reset_simulation(AppState& state, SimulationClock& clock);

} // namespace revolute
