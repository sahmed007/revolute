#pragma once
#include "revolute/app/app_state.hpp"

namespace revolute::ui {

// Draw the workbench and return true when the wall-clock timestamp needs resetting.
bool draw_workspace(app::AppState& state, SimulationClock& clock, int& updates_this_frame);
} // namespace revolute::ui
