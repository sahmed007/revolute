#include "kinematics.hpp"
#include <GLFW/glfw3.h>
#include <cstdlib>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

ImVec2 world_to_screen(const revolute::Point2D& world_position, const ImVec2& screen_origin, double scale) {
    double scaled_world_x = world_position.x * scale;
    double scaled_world_y = world_position.y * scale;
    double displaced_x = screen_origin.x + scaled_world_x;
    double displaced_y = screen_origin.y - scaled_world_y;
    return ImVec2{static_cast<float>(displaced_x), static_cast<float>(displaced_y)};
}

int main() {
    // ensure GLFW is initialized before program
    if (!glfwInit()) {
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(1000, 700, "Revolute", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    // use the current window as the current application context
    glfwMakeContextCurrent(window);

    // sync buffer swap with the display
    glfwSwapInterval(1);

    // set the clear color (Red, Green, Blue, Alpha) clamped between 0.0f and 1.0f
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
        ImGui::SetNextWindowSize({250, io.DisplaySize.y});

        ImVec2 arm_area_center = {(io.DisplaySize.x + 250) / 2, io.DisplaySize.y / 2};

        bool controls_panel = ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
        if (controls_panel) {
            ImGui::Text("Controls will go here");
            ImGui::Text("Arm center: (%.1f, %.1f)", arm_area_center.x, arm_area_center.y);
        }
        ImGui::End();

        ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
        draw_list->AddCircleFilled(arm_area_center, 5.0f, IM_COL32(235, 80, 85, 255));
        draw_list->AddLine({250, arm_area_center.y}, {io.DisplaySize.x, 350}, IM_COL32(75, 80, 90, 255));
        draw_list->AddLine({arm_area_center.x, 0}, {arm_area_center.x, io.DisplaySize.y}, IM_COL32(75, 80, 90, 255));

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
