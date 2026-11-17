#include "plots.hpp"

#include <imgui.h>
#include <implot.h>
#include <vector>

namespace revolute {
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
    if (ImPlot::BeginPlot(title, ImVec2(-1, 220))) {
        ImPlot::SetupAxes("Time (s)", units);
        if (!times.empty()) {
            const int count = static_cast<int>(times.size());
            ImPlot::PlotLine(first_label, times.data(), first.data(), count);
            ImPlot::PlotLine(second_label, times.data(), second.data(), count);
        }
        ImPlot::EndPlot();
    }
}

} // namespace

void draw_plots(const AppState& state) {
    if (ImGui::Begin("Simulation plots")) {
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

        for (auto* values : {&times, &shoulder_targets, &shoulder_angles, &elbow_targets,
                             &elbow_angles, &shoulder_velocities, &elbow_velocities,
                             &shoulder_torques, &elbow_torques, &shoulder_errors, &elbow_errors}) {
            values->reserve(state.history.size());
        }

        for (const auto& sample : state.history) {
            times.push_back(sample.time);
            shoulder_targets.push_back(sample.shoulder.target_angle);
            shoulder_angles.push_back(sample.shoulder.angle);
            elbow_targets.push_back(sample.elbow.target_angle);
            elbow_angles.push_back(sample.elbow.angle);
            shoulder_velocities.push_back(sample.shoulder.angular_velocity);
            elbow_velocities.push_back(sample.elbow.angular_velocity);
            shoulder_torques.push_back(sample.shoulder.applied_torque);
            elbow_torques.push_back(sample.elbow.applied_torque);
            shoulder_errors.push_back(sample.shoulder.error);
            elbow_errors.push_back(sample.elbow.error);
        }

        if (times.empty()) {
            ImGui::TextUnformatted("Resume or Step to record simulation samples.");
        }
        draw_pair(
            "Shoulder position", "Angle (rad)", times,
            "Target", shoulder_targets, "Actual", shoulder_angles
        );
        draw_pair(
            "Elbow position", "Angle (rad)", times,
            "Target", elbow_targets, "Actual", elbow_angles
        );
        draw_pair(
            "Joint velocity", "Angular velocity (rad/s)", times,
            "Shoulder", shoulder_velocities, "Elbow", elbow_velocities
        );
        draw_pair(
            "Applied torque", "Torque (N·m)", times,
            "Shoulder", shoulder_torques, "Elbow", elbow_torques
        );
        draw_pair(
            "Tracking error", "Target - actual (rad)", times,
            "Shoulder", shoulder_errors, "Elbow", elbow_errors
        );
    }

    ImGui::End();
}
} // namespace revolute
