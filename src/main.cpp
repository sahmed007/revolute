#include "app_state.hpp"
#include "arm_renderer.hpp"
#include "controls.hpp"
#include "dynamics.hpp"
#include "kinematics.hpp"
#include "plots.hpp"
#include "simulation.hpp"

#include <GLFW/glfw3.h>
#include <cstdlib>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

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
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    // setup platform backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    revolute::AppState state;
    revolute::SimulationClock simulation_clock;
    double previous_time = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        double current_time = glfwGetTime();
        int updates_this_frame = 0;

        if (!state.paused) {
            updates_this_frame = revolute::advance_simulation_clock(
                simulation_clock,
                current_time - previous_time,
                revolute::TIME_STEP,
                revolute::MAX_UPDATES_PER_FRAME
            );
        }

        previous_time = current_time;

        for (int update = 0; update < updates_this_frame; ++update) {
            revolute::advance_simulation(state);
        }

        // clear the screen using the color state specified above
        glClear(GL_COLOR_BUFFER_BIT);
        // poll and handle events
        glfwPollEvents();

        // start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
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

        if (revolute::draw_controls(state, simulation_clock, updates_this_frame, CONTROLS_WIDTH)) {
            previous_time = glfwGetTime();
        }
        const auto pose = revolute::forward_kinematics(
            state.upper_arm_length,
            state.forearm_length,
            state.shoulder_state.angle,
            state.elbow_state.angle
        );

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
        revolute::draw_plots(state);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // swap the front and back buffers to display the render
        glfwSwapBuffers(window);
    }

    // release ImGui resources
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    // destroy window before application closes
    glfwDestroyWindow(window);

    // terminate GLFW before application closes
    glfwTerminate();
    exit(EXIT_SUCCESS);
}
