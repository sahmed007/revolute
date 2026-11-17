#include "revolute/ui/workspace.hpp"

#include "revolute/app/simulation.hpp"
#include "revolute/ui/arm_renderer.hpp"
#include "revolute/ui/controls.hpp"
#include "revolute/ui/plots.hpp"

#include <algorithm>
#include <cmath>
#include <imgui.h>

namespace revolute::ui {

namespace {

const ImVec4 amber{0.95f, 0.70f, 0.25f, 1};

void draw_grid(ImDrawList* draw, ImVec2 top, ImVec2 bottom, ImVec2 origin, double scale) {
    const float spacing = static_cast<float>(scale * 0.5);
    for (float x = origin.x; x < bottom.x; x += spacing) {
        draw->AddLine({x, top.y}, {x, bottom.y}, IM_COL32(36, 38, 39, 255));
    }

    for (float x = origin.x - spacing; x > top.x; x -= spacing) {
        draw->AddLine({x, top.y}, {x, bottom.y}, IM_COL32(36, 38, 39, 255));
    }

    for (float y = origin.y; y < bottom.y; y += spacing) {
        draw->AddLine({top.x, y}, {bottom.x, y}, IM_COL32(36, 38, 39, 255));
    }

    for (float y = origin.y - spacing; y > top.y; y -= spacing) {
        draw->AddLine({top.x, y}, {bottom.x, y}, IM_COL32(36, 38, 39, 255));
    }
}

struct CanvasGeometry {
    ImVec2 top;
    ImVec2 size;
    ImVec2 bottom;
    ImVec2 origin;
    double scale;
};

CanvasGeometry canvas_geometry(double reach) {
    const ImVec2 top = ImGui::GetCursorScreenPos();
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const ImVec2 size{std::max(1.0f, available.x), std::max(1.0f, available.y)};
    const ImVec2 bottom{top.x + size.x, top.y + size.y};
    const ImVec2 origin{top.x + size.x * 0.5f, top.y + size.y * 0.5f};
    const double scale = std::max(1.0, std::min(size.x, size.y) * 0.42 / reach);

    return {top, size, bottom, origin, scale};
}

void draw_viewport_header(const app::AppState& state, const ArmPose& pose) {
    ImGui::TextDisabled("WORKSPACE / PLANAR 2R");
    ImGui::SameLine();
    ImGui::TextColored(amber, "%s", state.paused ? "PAUSED" : "RUNNING");

    ImGui::TextDisabled("Hand  %.3f, %.3f m", pose.hand.x, pose.hand.y);
}

void draw_reach_and_axes(
    ImDrawList* draw,
    const app::AppState& state,
    const CanvasGeometry& canvas
) {
    const double reach = state.upper_arm_length + state.forearm_length;
    draw->AddCircle(
        canvas.origin,
        static_cast<float>(reach * canvas.scale),
        IM_COL32(108, 91, 52, 130),
        100
    );
    const double inner = std::abs(state.upper_arm_length - state.forearm_length);

    if (inner > 0) {
        draw->AddCircle(
            canvas.origin,
            static_cast<float>(inner * canvas.scale),
            IM_COL32(108, 91, 52, 130),
            100
        );
    }

    draw->AddLine(
        {canvas.top.x, canvas.origin.y},
        {canvas.bottom.x, canvas.origin.y},
        IM_COL32(72, 72, 67, 255)
    );
    draw->AddLine(
        {canvas.origin.x, canvas.top.y},
        {canvas.origin.x, canvas.bottom.y},
        IM_COL32(72, 72, 67, 255)
    );
}

void draw_canvas_labels(
    ImDrawList* draw,
    const CanvasGeometry& canvas,
    const app::AppState& state
) {
    draw->AddText(
        {canvas.top.x + 12, canvas.bottom.y - 24},
        IM_COL32(135, 136, 128, 255),
        state.task != app::AppState::Task::manual_torque
                && state.target_source == app::AppState::TargetSource::cartesian
            ? "Click to edit target  /  grid 0.5 m"
            : "Grid 0.5 m"
    );

    draw->AddText({canvas.bottom.x - 22, canvas.origin.y + 5}, IM_COL32(135, 136, 128, 255), "x");
    draw->AddText({canvas.origin.x + 6, canvas.top.y + 5}, IM_COL32(135, 136, 128, 255), "y");
}

void handle_target_click(app::AppState& state, const CanvasGeometry& canvas) {
    ImGui::InvisibleButton("WorkspaceCanvas", canvas.size);
    if (state.task == app::AppState::Task::manual_torque
        || state.target_source != app::AppState::TargetSource::cartesian) {
        return;
    }

    if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
        const auto target = screen_to_world(ImGui::GetMousePos(), canvas.origin, canvas.scale);
        state.target_x = target.x;
        state.target_y = target.y;
        app::mark_target_pending(state);
    }
}

void draw_viewport_target(const app::AppState& state, const CanvasGeometry& canvas) {
    if (state.task != app::AppState::Task::manual_torque) {
        const auto active_pose = forward_kinematics(
            state.upper_arm_length,
            state.forearm_length,
            state.active_target.shoulder,
            state.active_target.elbow
        );
        const auto target = state.target_source == app::AppState::TargetSource::cartesian
            ? Point2D{state.target_x, state.target_y}
            : active_pose.hand;
        if (std::isfinite(target.x) && std::isfinite(target.y)) {
            draw_target(target, canvas.origin, canvas.scale);
        }
    }
}

void draw_viewport(app::AppState& state) {
    const auto pose = forward_kinematics(
        state.upper_arm_length,
        state.forearm_length,
        state.shoulder_state.angle,
        state.elbow_state.angle
    );

    draw_viewport_header(state, pose);
    const auto canvas = canvas_geometry(state.upper_arm_length + state.forearm_length);

    auto* draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(canvas.top, canvas.bottom, true);
    draw->AddRectFilled(canvas.top, canvas.bottom, IM_COL32(20, 22, 24, 255), 6);

    draw_grid(draw, canvas.top, canvas.bottom, canvas.origin, canvas.scale);
    draw_reach_and_axes(draw, state, canvas);
    draw_arm(pose, canvas.origin, canvas.scale);
    draw_viewport_target(state, canvas);

    draw_canvas_labels(draw, canvas, state);
    draw->PopClipRect();
    handle_target_click(state, canvas);
}

const char* task_name(app::AppState::Task task) {
    switch (task) {
    case app::AppState::Task::pose:
        return "Pose";
    case app::AppState::Task::position_control:
        return "Position control";
    case app::AppState::Task::manual_torque:
        return "Manual torque";
    }

    return "";
}

const char* workflow_hint(const app::AppState& state) {
    if (state.task == app::AppState::Task::manual_torque) {
        return "Set torques, then Play or Step. Torque edits take effect immediately while "
               "running.";
    }

    if (state.task == app::AppState::Task::pose) {
        return state.target_source == app::AppState::TargetSource::cartesian
            ? "Set X/Y or click the canvas, then Solve & apply to move the arm instantly."
            : "Set joint angles, then Apply pose to move the arm instantly.";
    }

    return state.paused
        ? "Edit a target and gains, then Play or Step. Cartesian targets are solved first."
        : "Edit a target, then Update target to commit it. Pause stops time.";
}

void item_tooltip(const char* text) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void draw_task_selector(app::AppState& state, SimulationClock& clock) {
    ImGui::SetNextItemWidth(210);
    if (ImGui::BeginCombo("##Task", task_name(state.task))) {
        for (auto task :
             {app::AppState::Task::pose,
              app::AppState::Task::position_control,
              app::AppState::Task::manual_torque}) {
            if (ImGui::Selectable(task_name(task), state.task == task)) {
                app::switch_task(state, clock, task);
            }
        }

        ImGui::EndCombo();
    }
}

void draw_playback_actions(app::AppState& state, int& updates_this_frame) {
    if (ImGui::Button(state.paused ? "Play" : "Pause", {78, 0})) {
        if (state.paused) {
            app::start_simulation(state);
        } else {
            state.paused = true;
        }
    }

    item_tooltip(workflow_hint(state));
    ImGui::SameLine();
    ImGui::BeginDisabled(!state.paused);
    if (ImGui::Button("Step") && app::step_simulation(state)) {
        ++updates_this_frame;
    }

    ImGui::EndDisabled();
    item_tooltip(
        state.paused ? "Commit the edited target and advance one simulation update."
                     : "Pause before stepping one update."
    );
}

void draw_motion_actions(app::AppState& state, int& updates_this_frame) {
    if (state.task == app::AppState::Task::pose) {
        if (ImGui::Button(
                state.target_source == app::AppState::TargetSource::cartesian ? "Solve & apply"
                                                                              : "Apply pose"
            )) {
            app::apply_pose(state);
        }

        item_tooltip(workflow_hint(state));
        return;
    }
    draw_playback_actions(state, updates_this_frame);

    if (state.task == app::AppState::Task::position_control && !state.paused) {
        ImGui::SameLine();
        if (ImGui::Button("Update target")) {
            app::commit_target(state);
        }

        item_tooltip(
            "Commit the edited target without pausing. Failed IK keeps the active target."
        );
    }
}

void draw_toolbar_header(const app::AppState& state) {
    ImGui::TextDisabled("CONTROLS");
    ImGui::SameLine();
    ImGui::TextColored(
        amber,
        "%s  %.2f s",
        state.paused ? "PAUSED" : "RUNNING",
        state.simulation_time
    );
    if (!state.action_message.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(
            state.action_failed ? ImVec4(1, .45f, .35f, 1) : amber,
            "%s",
            state.action_failed ? "Action failed (!)" : "Done"
        );
        item_tooltip(state.action_message.c_str());
    }
}

bool draw_reset_action(app::AppState& state, SimulationClock& clock, int& updates_this_frame) {
    bool reset = false;
    if (ImGui::Button("Reset motion")) {
        app::reset_simulation(state, clock);
        updates_this_frame = 0;
        reset = true;
    }

    item_tooltip(
        "Pause and restore the starting pose; clear motion, time, and plots. Keep targets and "
        "gains."
    );
    return reset;
}

bool draw_toolbar(app::AppState& state, SimulationClock& clock, int& updates_this_frame) {
    bool reset = false;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {16, 10});
    const float height = 20 + ImGui::GetTextLineHeightWithSpacing() + ImGui::GetFrameHeight();
    if (ImGui::BeginChild("Command panel", {0, height}, ImGuiChildFlags_Borders)) {
        draw_toolbar_header(state);

        draw_task_selector(state, clock);
        ImGui::SameLine();
        draw_motion_actions(state, updates_this_frame);
        ImGui::SameLine();
        reset = draw_reset_action(state, clock, updates_this_frame);
    }

    ImGui::EndChild();
    ImGui::PopStyleVar();
    return reset;
}

void draw_scene_panels(app::AppState& state, float body_height) {
    const float right_width = std::max(100.0f, ImGui::GetContentRegionAvail().x);
    // Rotated axis labels need their text width in vertical space, plus room
    // for the telemetry tabs, plot title, and time axis.
    const float label_height = ImGui::CalcTextSize("Angular velocity (rad/s)").x;
    const float telemetry_height = label_height + 160.0f;
    const float plot_height = std::min(
        std::max(body_height * 0.42f, telemetry_height),
        std::max(180.0f, body_height - 140.0f)
    );
    const float viewport_height =
        std::max(80.0f, body_height - plot_height - ImGui::GetStyle().ItemSpacing.y);

    if (ImGui::BeginChild(
            "Viewport",
            {right_width, viewport_height},
            ImGuiChildFlags_Borders,
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
        )) {
        draw_viewport(state);
    }

    ImGui::EndChild();
    if (ImGui::BeginChild("Telemetry", {right_width, plot_height}, ImGuiChildFlags_Borders)) {
        draw_plots(state);
    }

    ImGui::EndChild();
}

void draw_panels(app::AppState& state) {
    const ImVec2 available = ImGui::GetContentRegionAvail();
    // Reserve the footer text, separator, and spacing after both the body and separator.
    const float footer_height =
        ImGui::GetTextLineHeight() + 2 * ImGui::GetStyle().ItemSpacing.y + 1.0f;
    const float body_height = std::max(1.0f, available.y - footer_height);
    const float inspector_width = std::clamp(available.x * 0.26f, 270.0f, 340.0f);

    if (ImGui::BeginChild("Inspector", {inspector_width, body_height}, ImGuiChildFlags_Borders)) {
        draw_controls(state);
    }

    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginGroup();

    draw_scene_panels(state, body_height);

    ImGui::EndGroup();
}

void draw_status_bar(
    const app::AppState& state,
    const SimulationClock& clock,
    int updates_this_frame
) {
    ImGui::Separator();
    ImGui::TextDisabled(
        "%s  |  dt %.0f ms  |  %d updates  |  %.0f FPS  |  Dropped %.3f s",
        task_name(state.task),
        app::TIME_STEP * 1000,
        updates_this_frame,
        ImGui::GetIO().Framerate,
        clock.dropped_time
    );
}
} // namespace

bool draw_workspace(app::AppState& state, SimulationClock& clock, int& updates_this_frame) {
    const auto* main_viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(main_viewport->WorkPos);
    ImGui::SetNextWindowSize(main_viewport->WorkSize);

    const auto flags = ImGuiWindowFlags_NoDecoration
        | ImGuiWindowFlags_NoMove
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoBringToFrontOnFocus;

    ImGui::Begin("Revolute workbench", nullptr, flags);
    const bool reset = draw_toolbar(state, clock, updates_this_frame);
    draw_panels(state);
    draw_status_bar(state, clock, updates_this_frame);
    ImGui::End();
    return reset;
}
} // namespace revolute::ui
