#include "controls.hpp"

#include "simulation.hpp"

#include <cmath>
#include <imgui.h>

namespace revolute {
namespace {
void draw_gain_input(const char* label, double& gain, ControllerState& controller) {
    double edited = gain;
    if (ImGui::InputDouble(label, &edited) && std::isfinite(edited) && edited >= 0.0) {
        gain = edited;
        controller = ControllerState{};
    }
}
} // namespace

bool draw_controls(
    AppState& state,
    SimulationClock& simulation_clock,
    int& updates_this_frame,
    float width
) {
    bool reset_requested = false;
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({width, ImGui::GetIO().DisplaySize.y});
    // create control panel area
    bool controls_panel =
        ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    if (controls_panel) {
        const char* sim_run_label = state.paused ? "Resume" : "Pause";
        if (ImGui::Button(sim_run_label)) {
            state.paused = !state.paused;
        }

        if (ImGui::Button("Step") && state.paused) {
            advance_simulation(state);
            updates_this_frame++;
        }

        if (ImGui::Button("Reset")) {
            reset_simulation(state, simulation_clock);
            reset_requested = true;
            updates_this_frame = 0;
        }

        if (ImGui::Checkbox("Joint position control", &state.position_control)) {
            state.shoulder_torque = 0.0;
            state.elbow_torque = 0.0;
            state.shoulder_controller = revolute::ControllerState{};
            state.elbow_controller = revolute::ControllerState{};
        }

        draw_gain_input(
            "Shoulder Kp (N·m/rad)",
            state.shoulder_gains.kp,
            state.shoulder_controller
        );

        draw_gain_input(
            "Shoulder Ki (N·m/(rad·s))",
            state.shoulder_gains.ki,
            state.shoulder_controller
        );

        draw_gain_input(
            "Shoulder Kd (N·m·s/rad)",
            state.shoulder_gains.kd,
            state.shoulder_controller
        );

        draw_gain_input("Elbow Kp (N·m/rad)", state.elbow_gains.kp, state.elbow_controller);
        draw_gain_input("Elbow Ki (N·m/(rad·s))", state.elbow_gains.ki, state.elbow_controller);
        draw_gain_input("Elbow Kd (N·m·s/rad)", state.elbow_gains.kd, state.elbow_controller);

        ImGui::TextDisabled("Gains must be finite and nonnegative.");
        ImGui::BeginDisabled(state.position_control);
        ImGui::InputDouble("Shoulder torque (N·m)", &state.shoulder_torque);
        ImGui::InputDouble("Elbow torque (N·m)", &state.elbow_torque);
        ImGui::EndDisabled();

        ImGui::PushItemWidth(-1.0f);
        ImGui::TextUnformatted("Shoulder angle (degrees)");
        ImGui::SliderFloat("##shoulder", &state.shoulder_angle_degrees, -180.0f, 180.0f);
        ImGui::TextUnformatted("Elbow angle (degrees)");
        ImGui::SliderFloat("##elbow", &state.elbow_angle_degrees, -180.0f, 180.0f);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::TextUnformatted("Target X (m)");
        ImGui::InputDouble("##target_x", &state.target_x);
        ImGui::TextUnformatted("Target Y (m)");
        ImGui::InputDouble("##target_y", &state.target_y);
        ImGui::PopItemWidth();

        ImGui::RadioButton("Positive elbow", &state.selected_solution, 0);
        ImGui::RadioButton("Negative elbow", &state.selected_solution, 1);

        if (ImGui::RadioButton(
                "Analytical",
                state.solver_mode == AppState::SolverMode::analytical
            )) {
            state.solver_mode = AppState::SolverMode::analytical;
        }

        ImGui::SameLine();

        if (ImGui::RadioButton("Numerical", state.solver_mode == AppState::SolverMode::numerical)) {
            state.solver_mode = AppState::SolverMode::numerical;
        }

        if (ImGui::Button("Solve IK") && state.paused) {
            solve_target(state);
        }

        ImGui::BeginDisabled(!state.paused);
        if (ImGui::Button("Apply pose")) {
            apply_pose(state);
        }
        ImGui::EndDisabled();

        if (state.solver_mode == AppState::SolverMode::numerical && state.numerical_result) {
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

    // calculate forward kinematics
    revolute::ArmPose pose = revolute::forward_kinematics(
        state.upper_arm_length,
        state.forearm_length,
        state.shoulder_state.angle,
        state.elbow_state.angle
    );

    double target_error = std::hypot(state.target_x - pose.hand.x, state.target_y - pose.hand.y);

    if (controls_panel) {
        ImGui::Text("Hand position: (%.1f, %.1f)", pose.hand.x, pose.hand.y);
        ImGui::Text("Target error: %.6f m", target_error);
        ImGui::Text("Updates this frame: %d", updates_this_frame);
        ImGui::Text("Simulation time (s): %.3f", state.simulation_time);
        ImGui::Text("Shoulder angle (rad): %.3f", state.shoulder_state.angle);
        ImGui::Text("Elbow angle (rad): %.3f", state.elbow_state.angle);
        ImGui::Text(
            "Shoulder angular velocity (rad/s): %.3f",
            state.shoulder_state.angular_velocity
        );
        ImGui::Text("Elbow angular velocity (rad/s): %.3f", state.elbow_state.angular_velocity);
        ImGui::Text("Shoulder torque (N·m): %.3f", state.shoulder_applied_torque);
        ImGui::Text("Elbow torque (N·m): %.3f", state.elbow_applied_torque);
        ImGui::Text("Total dropped time (s): %.3f", simulation_clock.dropped_time);
    }

    ImGui::End();
    return reset_requested;
}

} // namespace revolute
