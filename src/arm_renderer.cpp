#include "arm_renderer.hpp"
#include "kinematics.hpp"
#include <imgui.h>

namespace revolute {
namespace {
const ImU32 JOINT_COLOR = IM_COL32(235, 80, 85, 255);
const ImU32 LINK_COLOR = IM_COL32(190, 195, 205, 255);

ImVec2 world_to_screen(const revolute::Point2D& world_position, const ImVec2& screen_origin, double scale) {
    double scaled_world_x = world_position.x * scale;
    double scaled_world_y = world_position.y * scale;
    double displaced_x = screen_origin.x + scaled_world_x;
    double displaced_y = screen_origin.y - scaled_world_y;
    return ImVec2{static_cast<float>(displaced_x), static_cast<float>(displaced_y)};
}
} // namespace

void draw_arm(const ArmPose& pose, const ImVec2& screen_origin, double scale) {
    // initialize markers
    ImVec2 base = revolute::world_to_screen(pose.base, screen_origin, scale);
    ImVec2 elbow = revolute::world_to_screen(pose.elbow, screen_origin, scale);
    ImVec2 hand = revolute::world_to_screen(pose.hand, screen_origin, scale);

    // obtain pointer to the background's draw list
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();

    // draw links
    draw_list->AddLine(base, elbow, LINK_COLOR, 4.0f);
    draw_list->AddLine(elbow, hand, LINK_COLOR, 4.0f);

    // draw base
    draw_list->AddCircleFilled(base, 5.0f, JOINT_COLOR);

    // draw joints
    draw_list->AddCircleFilled(elbow, 5.0f, JOINT_COLOR);
    draw_list->AddCircleFilled(hand, 5.0f, JOINT_COLOR);
}
} // namespace revolute
