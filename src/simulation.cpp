#include "simulation.hpp"

#include "app_state.hpp"

#include <algorithm>
#include <numbers>

namespace revolute {

void solve_target(AppState& state) {
    state.target_unreachable = false;
    state.numerical_result.reset();

    if (state.solver_mode == AppState::SolverMode::numerical) {
        revolute::JointAngles initial_angles{
            state.shoulder_angle_degrees * std::numbers::pi / 180.0,
            state.elbow_angle_degrees * std::numbers::pi / 180.0
        };

        state.numerical_result = revolute::num_inverse_kinematics(
            state.upper_arm_length,
            state.forearm_length,
            {state.target_x, state.target_y},
            initial_angles,
            1e-6, // position tolerance in meters
            100,  // maximum updates
            0.1   // maximum correction in radians
        );

        const auto& result = *state.numerical_result;
        if (result.status == revolute::NumericalIKStatus::converged) {
            state.shoulder_angle_degrees =
                static_cast<float>(result.angles.shoulder * 180.0 / std::numbers::pi);
            state.elbow_angle_degrees =
                static_cast<float>(result.angles.elbow * 180.0 / std::numbers::pi);
        }
        return;
    }

    auto result = revolute::inverse_kinematics(
        state.upper_arm_length,
        state.forearm_length,
        {state.target_x, state.target_y}
    );
    state.target_unreachable = result.status == revolute::IKStatus::unreachable;
    if (result.status == revolute::IKStatus::success) {
        const int solution_index = result.solutions.size() == 1 ? 0 : state.selected_solution;
        const auto& solution = result.solutions[solution_index];
        state.shoulder_angle_degrees =
            static_cast<float>((solution.shoulder * 180) / std::numbers::pi);
        state.elbow_angle_degrees = static_cast<float>((solution.elbow * 180) / std::numbers::pi);
    }
}

void advance_simulation(AppState& state) {
    if (state.position_control) {
        state.shoulder_torque = revolute::pid_torque(
            state.shoulder_controller,
            state.shoulder_gains,
            state.shoulder_angle_degrees * std::numbers::pi / 180.0,
            state.shoulder_state.angle,
            state.shoulder_state.angular_velocity,
            TIME_STEP,
            state.shoulder_params.torque_limit
        );

        state.elbow_torque = revolute::pid_torque(
            state.elbow_controller,
            state.elbow_gains,
            state.elbow_angle_degrees * std::numbers::pi / 180.0,
            state.elbow_state.angle,
            state.elbow_state.angular_velocity,
            TIME_STEP,
            state.elbow_params.torque_limit
        );
    }

    state.shoulder_state = revolute::step_joint(
        state.shoulder_state,
        state.shoulder_params,
        state.shoulder_torque,
        TIME_STEP
    );

    state.elbow_state =
        revolute::step_joint(state.elbow_state, state.elbow_params, state.elbow_torque, TIME_STEP);

    state.shoulder_applied_torque = std::clamp(
        state.shoulder_torque,
        -state.shoulder_params.torque_limit,
        state.shoulder_params.torque_limit
    );

    state.elbow_applied_torque = std::clamp(
        state.elbow_torque,
        -state.elbow_params.torque_limit,
        state.elbow_params.torque_limit
    );

    state.simulation_time += TIME_STEP;

    double shoulder_target = state.shoulder_angle_degrees * std::numbers::pi / 180.0;
    double elbow_target = state.elbow_angle_degrees * std::numbers::pi / 180.0;
    SimulationSample sample{
        .time = state.simulation_time,
        .shoulder =
            {.target_angle = shoulder_target,
             .angle = state.shoulder_state.angle,
             .angular_velocity = state.shoulder_state.angular_velocity,
             .applied_torque = state.shoulder_applied_torque,
             .error = shoulder_target - state.shoulder_state.angle},
        .elbow = {
            .target_angle = elbow_target,
            .angle = state.elbow_state.angle,
            .angular_velocity = state.elbow_state.angular_velocity,
            .applied_torque = state.elbow_applied_torque,
            .error = elbow_target - state.elbow_state.angle
        }
    };

    state.history.push_back(sample);
    if (size(state.history) > MAX_HISTORY_SAMPLES) {
        state.history.pop_front();
    }
}

void apply_pose(AppState& state) {
    state.shoulder_state = {state.shoulder_angle_degrees * std::numbers::pi / 180.0, 0.0};
    state.elbow_state = {state.elbow_angle_degrees * std::numbers::pi / 180.0, 0.0};
}

void reset_simulation(AppState& state, SimulationClock& clock) {
    state = AppState{};
    clock = SimulationClock{};
}

} // namespace revolute
