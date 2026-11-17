#include "revolute/app/app_state.hpp"
#include "revolute/app/simulation.hpp"
#include "revolute/ui/ui_theme.hpp"
#include "revolute/ui/workspace.hpp"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <cstdlib>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

namespace {

GLFWwindow* create_window() {
    // ensure GLFW is initialized before program
    if (!glfwInit()) {
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(1440, 900, "Revolute", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwSetWindowSizeLimits(window, 900, 650, GLFW_DONT_CARE, GLFW_DONT_CARE);

    // use the current window as the current application context
    glfwMakeContextCurrent(window);

    // sync buffer swap with the display
    glfwSwapInterval(1);

    // set the clear color (Red, Green, Blue, Alpha) clamped
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    return window;
}

void initialize_gui(GLFWwindow* window) {
    // setup Dear ImGui context
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    float display_scale_x = 1.0f;
    float display_scale_y = 1.0f;

    glfwGetWindowContentScale(window, &display_scale_x, &display_scale_y);
    revolute::ui::load_ui_font(std::max(display_scale_x, display_scale_y));
    revolute::ui::apply_ui_theme();

    // setup platform backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
}

int update_simulation(
    revolute::app::AppState& state,
    revolute::SimulationClock& simulation_clock,
    double& previous_time
) {
    const double current_time = glfwGetTime();
    int updates_this_frame = 0;

    if (!state.paused) {
        updates_this_frame = revolute::advance_simulation_clock(
            simulation_clock,
            current_time - previous_time,
            revolute::app::TIME_STEP,
            revolute::app::MAX_UPDATES_PER_FRAME
        );
    }

    previous_time = current_time;

    for (int update = 0; update < updates_this_frame; ++update) {
        revolute::app::advance_simulation(state);
    }

    return updates_this_frame;
}

void begin_frame(GLFWwindow* window) {
    // clear the screen using the color state specified above
    int framebuffer_width, framebuffer_height;
    glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
    glViewport(0, 0, framebuffer_width, framebuffer_height);
    glClear(GL_COLOR_BUFFER_BIT);

    // poll and handle events
    glfwPollEvents();

    // start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void run_application(GLFWwindow* window) {
    revolute::app::AppState state;
    revolute::SimulationClock simulation_clock;
    double previous_time = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        int updates_this_frame = update_simulation(state, simulation_clock, previous_time);

        begin_frame(window);

        if (revolute::ui::draw_workspace(state, simulation_clock, updates_this_frame)) {
            previous_time = glfwGetTime();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // swap the front and back buffers to display the render
        glfwSwapBuffers(window);
    }
}

void shutdown_application(GLFWwindow* window) {
    // release ImGui resources
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    // destroy window before application closes
    glfwDestroyWindow(window);

    // terminate GLFW before application closes
    glfwTerminate();
}

} // namespace

int main() {
    GLFWwindow* window = create_window();
    initialize_gui(window);
    run_application(window);
    shutdown_application(window);
    return EXIT_SUCCESS;
}
