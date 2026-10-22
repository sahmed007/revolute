#include "kinematics.hpp"
#include <GLFW/glfw3.h>
#include <cstdlib>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <numbers>

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

    // calculate a fixed pose
    revolute::ArmPose pose = revolute::forward_kinematics(1, 1, 0, std::numbers::pi / 2);

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

        ImVec2 screen_origin = {(io.DisplaySize.x + 250) / 2, io.DisplaySize.y / 2};

        ImVec2 base = world_to_screen(pose.base, screen_origin, 140);
        ImVec2 elbow = world_to_screen(pose.elbow, screen_origin, 140);
        ImVec2 hand = world_to_screen(pose.hand, screen_origin, 140);

        bool controls_panel = ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
        if (controls_panel) {
            ImGui::Text("Controls will go here");
            ImGui::Text("Arm center: (%.1f, %.1f)", screen_origin.x, screen_origin.y);
        }
        ImGui::End();

        // draw axes
        ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
        draw_list->AddLine({250, screen_origin.y}, {io.DisplaySize.x, screen_origin.y}, IM_COL32(75, 80, 90, 255));
        draw_list->AddLine({screen_origin.x, 0}, {screen_origin.x, io.DisplaySize.y}, IM_COL32(75, 80, 90, 255));

        // draw links
        draw_list->AddLine(base, elbow, IM_COL32(190, 195, 205, 255), 4.0f);
        draw_list->AddLine(elbow, hand, IM_COL32(190, 195, 205, 255), 4.0f);

        // draw base
        draw_list->AddCircleFilled(base, 5.0f, IM_COL32(235, 80, 85, 255));

        // draw joints
        draw_list->AddCircleFilled(elbow, 5.0f, IM_COL32(235, 80, 85, 255));
        draw_list->AddCircleFilled(hand, 5.0f, IM_COL32(235, 80, 85, 255));

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
