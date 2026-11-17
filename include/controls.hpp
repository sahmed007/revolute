#pragma once

#include "app_state.hpp"

namespace revolute {

// Draw the complete control panel.
bool draw_controls(AppState& state, SimulationClock& clock, int& updates_this_frame, float width);

} // namespace revolute
