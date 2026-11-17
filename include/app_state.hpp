#pragma once

#include "control.hpp"
#include "dynamics.hpp"
#include "kinematics.hpp"

#include <cstddef>
#include <deque>
#include <numbers>
#include <optional>

namespace revolute {

inline constexpr double TIME_STEP = 0.01;
inline constexpr int MAX_UPDATES_PER_FRAME = 10;
inline constexpr std::size_t MAX_HISTORY_SAMPLES = 2000;

struct JointSample {
    double target_angle;
    double angle;
    double angular_velocity;
    double applied_torque;
    double error;
};

struct SimulationSample {
    double time;
    JointSample shoulder;
    JointSample elbow;
};

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
    revolute::PIDGains shoulder_gains{5.0, 0.0, 0.0};
    revolute::PIDGains elbow_gains{5.0, 0.0, 0.0};
    revolute::ControllerState shoulder_controller;
    revolute::ControllerState elbow_controller;

    // sim state
    bool paused = true;
    double simulation_time = 0.0;

    // sampling
    std::deque<SimulationSample> history;
};

} // namespace revolute
