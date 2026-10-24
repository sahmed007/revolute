#pragma once

#include "kinematics.hpp"
#include <imgui.h>

namespace revolute {
void draw_arm(const ArmPose& pose, const ImVec2& screen_origin, double scale);
} // namespace revolute
