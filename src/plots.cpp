#include "plots.hpp"

#include <imgui.h>
#include <implot.h>
#include <vector>

namespace revolute {

void draw_plots(const AppState& state) {
    if (ImGui::Begin("Simulation plots")) {
        std::vector<double> times;
        std::vector<double> shoulder_targets;
        std::vector<double> shoulder_angles;
        std::vector<double> elbow_targets;
        std::vector<double> elbow_angles;

        for (const auto& sample : state.history) {
            times.push_back(sample.time);
            shoulder_targets.push_back(sample.shoulder.target_angle);
            shoulder_angles.push_back(sample.shoulder.angle);
            elbow_targets.push_back(sample.elbow.target_angle);
            elbow_angles.push_back(sample.elbow.angle);
        }

        if (ImPlot::BeginPlot("Shoulder position")) {
            ImPlot::SetupAxes("Time (s)", "Angle(rad)");
            if (!times.empty()) {
                int count = static_cast<int>(times.size());
                ImPlot::PlotLine("Target", times.data(), shoulder_targets.data(), count);
                ImPlot::PlotLine("Actual", times.data(), shoulder_angles.data(), count);
            }
            ImPlot::EndPlot();
        }

        if (ImPlot::BeginPlot("Elbow position")) {
            ImPlot::SetupAxes("Time (s)", "Angle (rad)");
            if (!times.empty()) {
                int count = static_cast<int>(times.size());
                ImPlot::PlotLine("Target", times.data(), elbow_targets.data(), count);
                ImPlot::PlotLine("Actual", times.data(), elbow_angles.data(), count);
            }
            ImPlot::EndPlot();
        }
    }

    ImGui::End();
}
} // namespace revolute
