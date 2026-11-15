#pragma once

#include "kinematics.hpp"

#include <imgui.h>

namespace revolute {
// Draw the arm's links and joints using the screen origin and pixels-per-meter scale.
void draw_arm(const ArmPose& pose, const ImVec2& screen_origin, double scale);

// Draw an outlined target marker using the screen origin and pixels-per-meter scale.
void draw_target(const Point2D& target, const ImVec2& screen_origin, double scale);

// Convert screen pixels to world coordinates in meters using the origin and positive
// pixels-per-meter scale.
Point2D screen_to_world(const ImVec2& screen_position, const ImVec2& screen_origin, double scale);
} // namespace revolute
