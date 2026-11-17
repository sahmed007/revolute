#include "revolute/ui/arm_renderer.hpp"

#include "revolute/core/kinematics.hpp"

#include <imgui.h>

namespace revolute::ui {

namespace {

const ImU32 JOINT_COLOR = IM_COL32(244, 183, 61, 255);
const ImU32 LINK_COLOR = IM_COL32(212, 210, 199, 255);
const ImU32 TARGET_COLOR = IM_COL32(253, 245, 203, 255);

ImVec2 world_to_screen(
    const revolute::Point2D& world_position,
    const ImVec2& screen_origin,
    double scale
) {
    double scaled_world_x = world_position.x * scale;
    double scaled_world_y = world_position.y * scale;
    double displaced_x = screen_origin.x + scaled_world_x;
    double displaced_y = screen_origin.y - scaled_world_y;
    return ImVec2{static_cast<float>(displaced_x), static_cast<float>(displaced_y)};
}
} // namespace

void draw_arm(const ArmPose& pose, const ImVec2& screen_origin, double scale) {
    // initialize markers
    ImVec2 base = world_to_screen(pose.base, screen_origin, scale);
    ImVec2 elbow = world_to_screen(pose.elbow, screen_origin, scale);
    ImVec2 hand = world_to_screen(pose.hand, screen_origin, scale);

    // obtain pointer to the background's draw list
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // draw links
    draw_list->AddLine(base, elbow, LINK_COLOR, 8.0f);
    draw_list->AddLine(elbow, hand, LINK_COLOR, 8.0f);

    // draw base
    draw_list->AddCircleFilled(base, 7.0f, JOINT_COLOR);

    // draw joints
    draw_list->AddCircleFilled(elbow, 7.0f, JOINT_COLOR);
    draw_list->AddCircleFilled(hand, 7.0f, JOINT_COLOR);
}

void draw_target(const Point2D& target, const ImVec2& screen_origin, double scale) {
    // initialize target
    ImVec2 screen_target = world_to_screen(target, screen_origin, scale);

    // obtain pointer to the background's draw list
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // draw
    draw_list->AddCircle(screen_target, 7.0f, TARGET_COLOR, 0, 2.0f);
}

Point2D screen_to_world(const ImVec2& screen_position, const ImVec2& screen_origin, double scale) {
    double world_x = (screen_position.x - screen_origin.x) / scale;
    double world_y = (screen_origin.y - screen_position.y) / scale;
    return Point2D{.x = world_x, .y = world_y};
}
} // namespace revolute::ui
