#include "arm_renderer.hpp"
#include "control.hpp"
#include "dynamics.hpp"
#include "kinematics.hpp"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <numbers>
#include <optional>

const int CONTROLS_WIDTH = 340;
const int PIXELS_PER_METER = 140;
const ImU32 AXIS_COLOR = IM_COL32(75, 80, 90, 255);
constexpr double TIME_STEP = 0.01;
const int MAX_UPDATES_PER_FRAME = 10;

namespace {

struct AppState {
    enum class SolverMode {
        analytical,
        numerical
    };

    // define angles
    double upper_arm_length = 1;
    double forearm_length = 1;
    float shoulder_angle_degrees = 0;
    float elbow_angle_degrees = 90;

    // define target
    double target_x = 1.0;
    double target_y = 1.0;
    bool target_unreachable = false;

    // IK selection state
    int selected_solution = 0;

    // IK solver state
    SolverMode solver_mode = SolverMode::analytical;
    std::optional<revolute::NumericalIKResult> numerical_result;

    // joint states
    revolute::JointState shoulder_state{0, 0};
    revolute::JointState elbow_state{std::numbers::pi / 2, 0};

    // joint params
    revolute::JointParameters shoulder_params{1.0, 0.2, 5.0};
    revolute::JointParameters elbow_params{1.0, 0.2, 5.0};

    // joint torques
    double shoulder_torque = 0.0;
    double elbow_torque = 0.0;

    // applied torques
    double shoulder_applied_torque = 0.0;
    double elbow_applied_torque = 0.0;

    // control
    bool position_control = false;
    double shoulder_kp = 5.0;
    double shoulder_kd = 0.0;

    // sim state
    bool paused = true;
    double simulation_time = 0.0;
};

void solve_target(AppState& state) {
    state.target_unreachable = false;
    state.numerical_result.reset();

    if (state.solver_mode == AppState::SolverMode::numerical) {
        revolute::JointAngles initial_angles{
            state.shoulder_angle_degrees * std::numbers::pi / 180.0,
            state.elbow_angle_degrees * std::numbers::pi / 180.0
        };

        state.numerical_result = revolute::num_inverse_kinematics(
            state.upper_arm_length,
            state.forearm_length,
            {state.target_x, state.target_y},
            initial_angles,
            1e-6, // position tolerance in meters
            100,  // maximum updates
            0.1   // maximum correction in radians
        );

        const auto& result = *state.numerical_result;
        if (result.status == revolute::NumericalIKStatus::converged) {
            state.shoulder_angle_degrees =
                static_cast<float>(result.angles.shoulder * 180.0 / std::numbers::pi);
            state.elbow_angle_degrees =
                static_cast<float>(result.angles.elbow * 180.0 / std::numbers::pi);
        }
        return;
    }

    auto result = revolute::inverse_kinematics(
        state.upper_arm_length,
        state.forearm_length,
        {state.target_x, state.target_y}
    );
    state.target_unreachable = result.status == revolute::IKStatus::unreachable;
    if (result.status == revolute::IKStatus::success) {
        const int solution_index = result.solutions.size() == 1 ? 0 : state.selected_solution;
        const auto& solution = result.solutions[solution_index];
        state.shoulder_angle_degrees =
            static_cast<float>((solution.shoulder * 180) / std::numbers::pi);
        state.elbow_angle_degrees = static_cast<float>((solution.elbow * 180) / std::numbers::pi);
    }
}

void advance_simulation(AppState& state) {
    if (state.position_control) {
        state.shoulder_torque = revolute::pd_torque(
            state.shoulder_angle_degrees * std::numbers::pi / 180.0,
            state.shoulder_state.angle,
            state.shoulder_state.angular_velocity,
            state.shoulder_kp,
            state.shoulder_kd
        );
    }

    state.shoulder_state = revolute::step_joint(
        state.shoulder_state,
        state.shoulder_params,
        state.shoulder_torque,
        TIME_STEP
    );

    state.elbow_state =
        revolute::step_joint(state.elbow_state, state.elbow_params, state.elbow_torque, TIME_STEP);

    state.shoulder_applied_torque = std::clamp(
        state.shoulder_torque,
        -state.shoulder_params.torque_limit,
        state.shoulder_params.torque_limit
    );

    state.elbow_applied_torque = std::clamp(
        state.elbow_torque,
        -state.elbow_params.torque_limit,
        state.elbow_params.torque_limit
    );

    state.simulation_time += TIME_STEP;
}

} // namespace

int main() {
    // ensure GLFW is initialized before program
    if (!glfwInit()) {
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(1100, 700, "Revolute", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    // use the current window as the current application context
    glfwMakeContextCurrent(window);

    // sync buffer swap with the display
    glfwSwapInterval(1);

    // set the clear color (Red, Green, Blue, Alpha) clamped
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    // setup Dear ImGui context
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    // setup platform backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    AppState state;
    revolute::SimulationClock simulation_clock;
    double previous_time = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        double current_time = glfwGetTime();
        int updates_this_frame = 0;

        if (!state.paused) {
            updates_this_frame = revolute::advance_simulation_clock(
                simulation_clock,
                current_time - previous_time,
                TIME_STEP,
                MAX_UPDATES_PER_FRAME
            );
        }

        previous_time = current_time;

        for (int update = 0; update < updates_this_frame; ++update) {
            advance_simulation(state);
        }

        // clear the screen using the color state specified above
        glClear(GL_COLOR_BUFFER_BIT);
        // poll and handle events
        glfwPollEvents();

        // start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::SetNextWindowPos({0, 0});
        ImGui::SetNextWindowSize({CONTROLS_WIDTH, io.DisplaySize.y});
        ImVec2 screen_origin = {(io.DisplaySize.x + CONTROLS_WIDTH) / 2, io.DisplaySize.y / 2};

        // obtain mouse position and check viewport bounds
        ImVec2 mouse_pos = ImGui::GetMousePos();
        float mx = mouse_pos.x;
        float my = mouse_pos.y;
        bool inside_viewport = false;

        if (mx >= CONTROLS_WIDTH && mx < io.DisplaySize.x && my >= 0 && my < io.DisplaySize.y) {
            inside_viewport = true;
        }

        if (inside_viewport
            && ImGui::IsMouseClicked(ImGuiMouseButton_Left)
            && io.WantCaptureMouse == false) {
            auto intended_target =
                revolute::screen_to_world(mouse_pos, screen_origin, PIXELS_PER_METER);
            state.target_x = intended_target.x;
            state.target_y = intended_target.y;
            state.target_unreachable = false;
        }

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
                state = AppState{};
                simulation_clock = revolute::SimulationClock();
                previous_time = glfwGetTime();
                updates_this_frame = 0;
            }

            if (ImGui::Checkbox("Shoulder position control", &state.position_control)) {
                state.shoulder_torque = 0.0;
            }

            double edited_kp = state.shoulder_kp;
            if (ImGui::InputDouble("Shoulder Kp (N·m/rad)", &edited_kp)) {
                if (std::isfinite(edited_kp) && edited_kp >= 0.0) {
                    state.shoulder_kp = edited_kp;
                }
            }

            double edited_kd = state.shoulder_kd;
            if (ImGui::InputDouble("Shoulder Kd (N·m·s/rad)", &edited_kd)) {
                if (std::isfinite(edited_kd) && edited_kd >= 0.0) {
                    state.shoulder_kd = edited_kd;
                }
            }

            ImGui::TextDisabled("Gains must be finite and nonnegative.");
            ImGui::BeginDisabled(state.position_control);
            ImGui::InputDouble("Shoulder torque (N·m)", &state.shoulder_torque);
            ImGui::EndDisabled();
            ImGui::InputDouble("Elbow torque (N·m)", &state.elbow_torque);

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

            if (ImGui::RadioButton(
                    "Numerical",
                    state.solver_mode == AppState::SolverMode::numerical
                )) {
                state.solver_mode = AppState::SolverMode::numerical;
            }

            if (ImGui::Button("Solve IK") && state.paused) {
                solve_target(state);
            }

            ImGui::BeginDisabled(!state.paused);
            if (ImGui::Button("Apply pose")) {
                state.shoulder_state = {
                    state.shoulder_angle_degrees * std::numbers::pi / 180.0,
                    0.0
                };
                state.elbow_state = {state.elbow_angle_degrees * std::numbers::pi / 180.0, 0.0};
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

        double target_error =
            std::hypot(state.target_x - pose.hand.x, state.target_y - pose.hand.y);

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

        // draw axes
        ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
        draw_list->AddLine(
            {CONTROLS_WIDTH, screen_origin.y},
            {io.DisplaySize.x, screen_origin.y},
            AXIS_COLOR
        );
        draw_list->AddLine({screen_origin.x, 0}, {screen_origin.x, io.DisplaySize.y}, AXIS_COLOR);

        // draw arm
        revolute::draw_arm(pose, screen_origin, PIXELS_PER_METER);

        // draw target
        revolute::draw_target(
            {.x = state.target_x, .y = state.target_y},
            screen_origin,
            PIXELS_PER_METER
        );

        // rendering
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // swap the front and back buffers to display the render
        glfwSwapBuffers(window);
    }

    // release ImGui resources
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    // destroy window before application closes
    glfwDestroyWindow(window);

    // terminate GLFW before application closes
    glfwTerminate();
    exit(EXIT_SUCCESS);
}
