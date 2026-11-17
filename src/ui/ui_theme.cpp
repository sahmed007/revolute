#include "revolute/ui/ui_theme.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <imgui.h>
#include <implot.h>

namespace revolute::ui {

void load_ui_font(float display_scale) {
    auto& io = ImGui::GetIO();
    const float scale = std::isfinite(display_scale) ? std::max(1.0f, display_scale) : 1.0f;
    const float raster_size = 18.0f * scale;

    // The atlas uses physical pixels; ImGui layout continues to use logical pixels.
    io.FontGlobalScale = 1.0f / scale;
    std::error_code error;
    if (std::filesystem::is_regular_file(REVOLUTE_FONT_PATH, error)) {
        io.FontDefault = io.Fonts->AddFontFromFileTTF(REVOLUTE_FONT_PATH, raster_size);
        if (io.FontDefault) {
            return;
        }
    }
    std::fprintf(
        stderr,
        "Unable to load Iosevka: %s; using the default font.\n",
        REVOLUTE_FONT_PATH
    );

    ImFontConfig config;
    config.SizePixels = raster_size;
    io.FontDefault = io.Fonts->AddFontDefault(&config);
}

namespace {

void apply_layout_style() {
    auto& style = ImGui::GetStyle();
    style.WindowPadding = {16, 14};
    style.FramePadding = {10, 6};
    style.ItemSpacing = {10, 9};
    style.ItemInnerSpacing = {8, 6};
    style.WindowRounding = 0;
    style.ChildRounding = 8;
    style.FrameRounding = 5;
    style.GrabRounding = 4;
    style.ScrollbarRounding = 6;
    style.TabRounding = 5;
    style.WindowBorderSize = 0;
    style.ChildBorderSize = 1;
    style.FrameBorderSize = 0;
    style.ScrollbarSize = 12;
}

void apply_color_palette() {
    const ImVec4 amber{0.95f, 0.70f, 0.25f, 1};
    auto* c = ImGui::GetStyle().Colors;

    c[ImGuiCol_Text] = {0.91f, 0.91f, 0.88f, 1};
    c[ImGuiCol_TextDisabled] = {0.50f, 0.51f, 0.49f, 1};
    c[ImGuiCol_WindowBg] = {0.065f, 0.069f, 0.073f, 1};
    c[ImGuiCol_ChildBg] = {0.09f, 0.095f, 0.10f, 1};
    c[ImGuiCol_PopupBg] = {0.12f, 0.125f, 0.13f, 1};
    c[ImGuiCol_Border] = {0.20f, 0.21f, 0.22f, 0.7f};
    c[ImGuiCol_FrameBg] = {0.145f, 0.15f, 0.155f, 1};
    c[ImGuiCol_FrameBgHovered] = {0.22f, 0.205f, 0.17f, 1};
    c[ImGuiCol_FrameBgActive] = {0.29f, 0.25f, 0.17f, 1};
    c[ImGuiCol_Button] = {0.19f, 0.19f, 0.18f, 1};
    c[ImGuiCol_ButtonHovered] = {0.35f, 0.29f, 0.17f, 1};
    c[ImGuiCol_ButtonActive] = {0.45f, 0.34f, 0.16f, 1};
    c[ImGuiCol_Header] = {0.22f, 0.20f, 0.15f, 1};
    c[ImGuiCol_HeaderHovered] = {0.31f, 0.26f, 0.16f, 1};
    c[ImGuiCol_HeaderActive] = {0.39f, 0.31f, 0.16f, 1};
    c[ImGuiCol_CheckMark] = amber;
    c[ImGuiCol_SliderGrab] = amber;
    c[ImGuiCol_SliderGrabActive] = {1, 0.80f, 0.36f, 1};
    c[ImGuiCol_Separator] = c[ImGuiCol_Border];
    c[ImGuiCol_SeparatorHovered] = amber;
    c[ImGuiCol_SeparatorActive] = amber;
    c[ImGuiCol_ResizeGrip] = {0.95f, 0.70f, 0.25f, 0.15f};
    c[ImGuiCol_ResizeGripHovered] = amber;
    c[ImGuiCol_ResizeGripActive] = amber;
    c[ImGuiCol_Tab] = {0.13f, 0.14f, 0.14f, 1};
    c[ImGuiCol_TabHovered] = {0.35f, 0.29f, 0.17f, 1};
    c[ImGuiCol_TabSelected] = {0.26f, 0.23f, 0.16f, 1};
    c[ImGuiCol_TabSelectedOverline] = amber;
    c[ImGuiCol_NavCursor] = amber;
    c[ImGuiCol_TextSelectedBg] = {0.95f, 0.70f, 0.25f, 0.25f};
}

void apply_plot_style() {
    ImPlot::StyleColorsDark();
    ImPlot::GetStyle().PlotPadding = {12, 12};
    ImPlot::GetStyle().Colors[ImPlotCol_PlotBg] = {0.075f, 0.08f, 0.085f, 1};
    ImPlot::GetStyle().Colors[ImPlotCol_FrameBg] = {0, 0, 0, 0};
    ImPlot::GetStyle().Colors[ImPlotCol_AxisGrid] = {0.3f, 0.31f, 0.32f, 0.25f};
}

} // namespace

void apply_ui_theme() {
    ImGui::StyleColorsDark();
    apply_layout_style();
    apply_color_palette();
    apply_plot_style();
}
} // namespace revolute::ui
