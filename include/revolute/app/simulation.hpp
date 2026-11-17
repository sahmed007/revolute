#pragma once

#include "revolute/app/app_state.hpp"

namespace revolute::app {

// Invalidate feedback when target inputs or solver options change.
void mark_target_pending(AppState& state);

// Resolve and commit the draft atomically; failed IK preserves the active target.
bool commit_target(AppState& state);

// Start or single-step after validating the selected task's commands.
bool start_simulation(AppState& state);
bool step_simulation(AppState& state);

// Pause and clear motion/commands while retaining the current pose and setup.
void switch_task(AppState& state, SimulationClock& clock, AppState::Task task);

// Advance both joints and controllers by TIME_STEP, updating torque readouts and time.
void advance_simulation(AppState& state);

// Resolve and apply the draft only in paused Pose mode; retain pose on failure.
bool apply_pose(AppState& state);

// Restore the starting pose and clear motion/history, preserving editable setup.
void reset_simulation(AppState& state, SimulationClock& clock);

} // namespace revolute::app
