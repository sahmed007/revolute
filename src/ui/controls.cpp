#include "revolute/ui/controls.hpp"

#include "revolute/app/simulation.hpp"

#include <cmath>
#include <imgui.h>

namespace revolute::ui {

namespace {

void draw_gain_input(const char* label, double& gain, ControllerState& controller) {
    ImGui::TextUnformatted(label);
    ImGui::PushID(label);
    ImGui::SetNextItemWidth(-1);
    double edited = gain;
    if (ImGui::InputDouble("##gain", &edited) && std::isfinite(edited) && edited >= 0.0) {
        gain = edited;
        controller = ControllerState{};
    }

    ImGui::PopID();
}

void draw_solver_status(const app::AppState& state) {
    if (state.solver_mode == app::AppState::SolverMode::numerical && state.numerical_result) {
        const auto& result = *state.numerical_result;

        const char* status = "Unknown";
        switch (result.status) {
        case revolute::NumericalIKStatus::converged:
            status = "Converged";
            break;
        case revolute::NumericalIKStatus::iteration_limit:
            status = "Iteration limit reached";
            break;
        case revolute::NumericalIKStatus::joint_limit:
            status = "Joint limit reached";
            break;
        case revolute::NumericalIKStatus::numerical_failure:
            status = "Numerical failure";
            break;
        }

        ImGui::TextUnformatted("Last numerical solve:");
        ImGui::Text("Status: %s", status);
        ImGui::Text("Updates: %d", result.iterations);
        ImGui::Text("Final error: %.8f m", result.position_error);
    }

    if (state.target_unreachable) {
        ImGui::TextWrapped("Target position is unreachable.");
    }
}

void draw_joint_targets(app::AppState& state) {
    ImGui::PushItemWidth(-1.0f);
    ImGui::TextUnformatted("Shoulder (degrees)");
    ImGui::SliderFloat("##shoulder", &state.shoulder_angle_degrees, -180.0f, 180.0f);
    if (ImGui::IsItemEdited()) {
        app::mark_target_pending(state);
    }

    ImGui::TextUnformatted("Elbow (degrees)");
    ImGui::SliderFloat("##elbow", &state.elbow_angle_degrees, -180.0f, 180.0f);
    if (ImGui::IsItemEdited()) {
        app::mark_target_pending(state);
    }

    ImGui::PopItemWidth();
}

void draw_solver_settings(app::AppState& state) {
    ImGui::SeparatorText("IK solver");
    if (ImGui::RadioButton(
            "Analytical",
            state.solver_mode == app::AppState::SolverMode::analytical
        )) {
        state.solver_mode = app::AppState::SolverMode::analytical;
        app::mark_target_pending(state);
    }

    ImGui::SameLine();
    if (ImGui::RadioButton(
            "Numerical",
            state.solver_mode == app::AppState::SolverMode::numerical
        )) {
        state.solver_mode = app::AppState::SolverMode::numerical;
        app::mark_target_pending(state);
    }

    if (state.solver_mode == app::AppState::SolverMode::analytical) {
        ImGui::TextUnformatted("Solution branch");
        ImGui::RadioButton("Positive elbow", &state.selected_solution, 0);
        if (ImGui::IsItemEdited()) {
            app::mark_target_pending(state);
        }

        ImGui::RadioButton("Negative elbow", &state.selected_solution, 1);
        if (ImGui::IsItemEdited()) {
            app::mark_target_pending(state);
        }
    }
    draw_solver_status(state);
}

void draw_cartesian_target(app::AppState& state) {
    ImGui::TextUnformatted("Target X (m)");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputDouble("##target_x", &state.target_x);

    if (ImGui::IsItemEdited()) {
        app::mark_target_pending(state);
    }

    ImGui::TextUnformatted("Target Y (m)");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputDouble("##target_y", &state.target_y);

    if (ImGui::IsItemEdited()) {
        app::mark_target_pending(state);
    }

    draw_solver_settings(state);
}

void draw_target_source(app::AppState& state) {
    ImGui::SeparatorText("Target source");
    if (ImGui::RadioButton(
            "Joint angles",
            state.target_source == app::AppState::TargetSource::joint_angles
        )) {
        state.target_source = app::AppState::TargetSource::joint_angles;
        app::mark_target_pending(state);
    }

    if (ImGui::RadioButton(
            "Cartesian target",
            state.target_source == app::AppState::TargetSource::cartesian
        )) {
        state.target_source = app::AppState::TargetSource::cartesian;
        app::mark_target_pending(state);
    }
}

void draw_targets(app::AppState& state) {
    draw_target_source(state);

    ImGui::SeparatorText("Edited target");
    ImGui::TextDisabled("%s", state.target_pending ? "Pending - use the top actions" : "Committed");

    if (state.target_source == app::AppState::TargetSource::joint_angles) {
        draw_joint_targets(state);
    } else {
        draw_cartesian_target(state);
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Active target (degrees)");
    ImGui::Text(
        "Shoulder %.2f | Elbow %.2f",
        state.active_target.shoulder * 180 / std::numbers::pi,
        state.active_target.elbow * 180 / std::numbers::pi
    );
}

void draw_pid_gains(app::AppState& state) {
    ImGui::SeparatorText("Shoulder gains");
    draw_gain_input("Shoulder Kp (N·m/rad)", state.shoulder_gains.kp, state.shoulder_controller);

    draw_gain_input(
        "Shoulder Ki (N·m/(rad·s))",
        state.shoulder_gains.ki,
        state.shoulder_controller
    );

    draw_gain_input("Shoulder Kd (N·m·s/rad)", state.shoulder_gains.kd, state.shoulder_controller);
    ImGui::SeparatorText("Elbow gains");
    draw_gain_input("Elbow Kp (N·m/rad)", state.elbow_gains.kp, state.elbow_controller);
    draw_gain_input("Elbow Ki (N·m/(rad·s))", state.elbow_gains.ki, state.elbow_controller);
    draw_gain_input("Elbow Kd (N·m·s/rad)", state.elbow_gains.kd, state.elbow_controller);
}

void draw_torque_inputs(app::AppState& state) {
    ImGui::SeparatorText("Commanded torque");
    ImGui::TextUnformatted("Shoulder torque (N·m)");
    ImGui::SetNextItemWidth(-1);
    double shoulder = state.shoulder_torque;

    if (ImGui::InputDouble("##shoulder_torque", &shoulder) && std::isfinite(shoulder)) {
        state.shoulder_torque = shoulder;
    }

    ImGui::TextUnformatted("Elbow torque (N·m)");
    ImGui::SetNextItemWidth(-1);
    double elbow = state.elbow_torque;

    if (ImGui::InputDouble("##elbow_torque", &elbow) && std::isfinite(elbow)) {
        state.elbow_torque = elbow;
    }
}

void draw_readout_row(const char* label, double shoulder, double elbow) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(label);
    ImGui::TableNextColumn();
    ImGui::Text("%.3f", shoulder);
    ImGui::TableNextColumn();
    ImGui::Text("%.3f", elbow);
}

void draw_live_state(const app::AppState& state) {
    ImGui::Spacing();
    ImGui::SeparatorText("Live state");

    if (ImGui::BeginTable("JointReadouts", 3, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableSetupColumn("Joint");
        ImGui::TableSetupColumn("Shoulder");
        ImGui::TableSetupColumn("Elbow");
        ImGui::TableHeadersRow();

        draw_readout_row("rad", state.shoulder_state.angle, state.elbow_state.angle);
        draw_readout_row(
            "rad/s",
            state.shoulder_state.angular_velocity,
            state.elbow_state.angular_velocity
        );
        draw_readout_row("N·m", state.shoulder_applied_torque, state.elbow_applied_torque);
        ImGui::EndTable();
    }
}

} // namespace

void draw_controls(app::AppState& state) {
    ImGui::TextDisabled("INSPECTOR");
    if (state.task == app::AppState::Task::manual_torque) {
        draw_torque_inputs(state);
    } else {
        draw_targets(state);
        if (state.task == app::AppState::Task::position_control) {
            draw_pid_gains(state);
        }
    }
    draw_live_state(state);
}
} // namespace revolute::ui
