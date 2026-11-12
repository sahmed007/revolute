#include "arm_renderer.hpp"
#include "kinematics.hpp"

#include <GLFW/glfw3.h>
#include <cstdlib>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <numbers>

const int CONTROLS_WIDTH = 340;
const int PIXELS_PER_METER = 140;
const ImU32 AXIS_COLOR = IM_COL32(75, 80, 90, 255);

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

    // define angles
    double upper_arm_length = 1;
    double forearm_length = 1;
    float shoulder_angle_degrees = 0;
    float elbow_angle_degrees = 90;

    // define target
    double target_x = 1.0;
    double target_y = 1.0;
    bool target_unreachable = false;

    while (!glfwWindowShouldClose(window)) {
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

        if (inside_viewport && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && io.WantCaptureMouse == false) {
            auto intended_target = revolute::screen_to_world(mouse_pos, screen_origin, PIXELS_PER_METER);
            target_x = intended_target.x;
            target_y = intended_target.y;
            target_unreachable = false;
        }

        // create control panel area
        bool controls_panel = ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
        if (controls_panel) {
            ImGui::PushItemWidth(-1.0f);
            ImGui::TextUnformatted("Shoulder angle (degrees)");
            ImGui::SliderFloat("##shoulder", &shoulder_angle_degrees, -180.0f, 180.0f);
            ImGui::TextUnformatted("Elbow angle (degrees)");
            ImGui::SliderFloat("##elbow", &elbow_angle_degrees, -180.0f, 180.0f);

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::TextUnformatted("Target X (m)");
            ImGui::InputDouble("##target_x", &target_x);
            ImGui::TextUnformatted("Target Y (m)");
            ImGui::InputDouble("##target_y", &target_y);
            ImGui::PopItemWidth();

            if (ImGui::Button("Solve IK")) {
                auto result = revolute::inverse_kinematics(upper_arm_length, forearm_length, {target_x, target_y});
                target_unreachable = result.status == revolute::IKStatus::unreachable;
                if (result.status == revolute::IKStatus::success) {
                    const auto& angles = result.solutions.front();
                    shoulder_angle_degrees = static_cast<float>((angles.shoulder * 180) / std::numbers::pi);
                    elbow_angle_degrees = static_cast<float>((angles.elbow * 180) / std::numbers::pi);
                }
            }

            if (target_unreachable) {
                ImGui::TextWrapped("Target position is unreachable.");
            }
        }

        // calculate forward kinematics
        auto shoulder_angle_radians = shoulder_angle_degrees * std::numbers::pi / 180;
        auto elbow_angle_radians = elbow_angle_degrees * std::numbers::pi / 180;
        revolute::ArmPose pose =
            revolute::forward_kinematics(upper_arm_length, forearm_length, shoulder_angle_radians, elbow_angle_radians);

        if (controls_panel) {
            ImGui::Text("Hand position: (%.1f, %.1f)", pose.hand.x, pose.hand.y);
        }

        ImGui::End();

        // draw axes
        ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
        draw_list->AddLine({CONTROLS_WIDTH, screen_origin.y}, {io.DisplaySize.x, screen_origin.y}, AXIS_COLOR);
        draw_list->AddLine({screen_origin.x, 0}, {screen_origin.x, io.DisplaySize.y}, AXIS_COLOR);

        // draw arm
        revolute::draw_arm(pose, screen_origin, PIXELS_PER_METER);

        // draw target
        revolute::draw_target({.x = target_x, .y = target_y}, screen_origin, PIXELS_PER_METER);

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
