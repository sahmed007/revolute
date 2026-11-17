#include "revolute/ui/plots.hpp"

#include <algorithm>
#include <imgui.h>
#include <implot.h>
#include <vector>

namespace revolute::ui {

namespace {

void draw_pair(
    const char* title,
    const char* units,
    const std::vector<double>& times,
    const char* first_label,
    const std::vector<double>& first,
    const char* second_label,
    const std::vector<double>& second
) {
    ImGui::Spacing();
    if (ImPlot::BeginPlot(
            title,
            ImVec2(
                -1,
                std::max(ImGui::CalcTextSize(units).x + 100.0f, ImGui::GetContentRegionAvail().y)
            )
        )) {
        ImPlot::SetupAxes("Time (s)", units);
        if (!times.empty()) {
            const int count = static_cast<int>(times.size());
            ImPlot::SetNextLineStyle(ImVec4(0.95f, 0.70f, 0.25f, 1), 2);
            ImPlot::PlotLine(first_label, times.data(), first.data(), count);
            ImPlot::SetNextLineStyle(ImVec4(0.82f, 0.83f, 0.78f, 1), 2);
            ImPlot::PlotLine(second_label, times.data(), second.data(), count);
        }
        ImPlot::EndPlot();
    }
}

struct PlotData {
    std::vector<double> times;
    std::vector<double> shoulder_targets;
    std::vector<double> shoulder_angles;
    std::vector<double> elbow_targets;
    std::vector<double> elbow_angles;
    std::vector<double> shoulder_velocities;
    std::vector<double> elbow_velocities;
    std::vector<double> shoulder_torques;
    std::vector<double> elbow_torques;
    std::vector<double> shoulder_errors;
    std::vector<double> elbow_errors;
};

void reserve_plot_data(PlotData& data, std::size_t sample_count) {
    for (auto* values :
         {&data.times,
          &data.shoulder_targets,
          &data.shoulder_angles,
          &data.elbow_targets,
          &data.elbow_angles,
          &data.shoulder_velocities,
          &data.elbow_velocities,
          &data.shoulder_torques,
          &data.elbow_torques,
          &data.shoulder_errors,
          &data.elbow_errors}) {
        values->reserve(sample_count);
    }
}

PlotData collect_plot_data(const app::AppState& state) {
    PlotData data;
    reserve_plot_data(data, state.history.size());

    for (const auto& sample : state.history) {
        data.times.push_back(sample.time);
        data.shoulder_targets.push_back(sample.shoulder.target_angle);
        data.shoulder_angles.push_back(sample.shoulder.angle);
        data.elbow_targets.push_back(sample.elbow.target_angle);
        data.elbow_angles.push_back(sample.elbow.angle);
        data.shoulder_velocities.push_back(sample.shoulder.angular_velocity);
        data.elbow_velocities.push_back(sample.elbow.angular_velocity);
        data.shoulder_torques.push_back(sample.shoulder.applied_torque);
        data.elbow_torques.push_back(sample.elbow.applied_torque);
        data.shoulder_errors.push_back(sample.shoulder.error);
        data.elbow_errors.push_back(sample.elbow.error);
    }

    return data;
}

struct PlotDefinition {
    const char* tab;
    const char* title;
    const char* units;
    const char* first_label;
    const char* second_label;
    const std::vector<double>& first;
    const std::vector<double>& second;
};

void draw_plot_tab(const PlotDefinition& plot, const std::vector<double>& times) {
    if (ImGui::BeginTabItem(plot.tab)) {
        draw_pair(
            plot.title,
            plot.units,
            times,
            plot.first_label,
            plot.first,
            plot.second_label,
            plot.second
        );

        ImGui::EndTabItem();
    }
}

void draw_plot_tabs(const PlotData& data) {
    const PlotDefinition plots[] = {
        {"Shoulder",
         "Shoulder position",
         "Angle (rad)",
         "Target",
         "Actual",
         data.shoulder_targets,
         data.shoulder_angles},
        {"Elbow",
         "Elbow position",
         "Angle (rad)",
         "Target",
         "Actual",
         data.elbow_targets,
         data.elbow_angles},
        {"Velocity",
         "Joint velocity",
         "Angular velocity (rad/s)",
         "Shoulder",
         "Elbow",
         data.shoulder_velocities,
         data.elbow_velocities},
        {"Torque",
         "Applied torque",
         "Torque (N·m)",
         "Shoulder",
         "Elbow",
         data.shoulder_torques,
         data.elbow_torques},
        {"Error",
         "Tracking error",
         "Target - actual (rad)",
         "Shoulder",
         "Elbow",
         data.shoulder_errors,
         data.elbow_errors}
    };

    if (ImGui::BeginTabBar("SignalTabs")) {
        for (const auto& plot : plots) {
            draw_plot_tab(plot, data.times);
        }

        ImGui::EndTabBar();
    }
}

} // namespace

void draw_plots(const app::AppState& state) {
    ImGui::TextDisabled("TELEMETRY");
    const auto data = collect_plot_data(state);

    if (data.times.empty()) {
        ImGui::TextUnformatted("Resume or Step to record simulation samples.");
    }

    draw_plot_tabs(data);
}
} // namespace revolute::ui
