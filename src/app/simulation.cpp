#include "revolute/app/simulation.hpp"

#include "revolute/app/app_state.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace revolute::app {

namespace {
void clear_motion(AppState& state) {
    state.paused = true;
    state.shoulder_state.angular_velocity = 0;
    state.elbow_state.angular_velocity = 0;
    state.shoulder_torque = state.elbow_torque = 0;
    state.shoulder_applied_torque = state.elbow_applied_torque = 0;
    state.shoulder_controller = {};
    state.elbow_controller = {};
}

bool fail_action(AppState& state, const char* message) {
    state.action_failed = true;
    state.action_message = message;
    return false;
}

std::optional<JointAngles> resolve_joint_target(AppState& state) {
    if (!std::isfinite(state.shoulder_angle_degrees) || !std::isfinite(state.elbow_angle_degrees)) {
        fail_action(state, "Enter finite joint angles. Active target retained.");
        return std::nullopt;
    }

    return JointAngles{
        state.shoulder_angle_degrees * std::numbers::pi / 180.0,
        state.elbow_angle_degrees * std::numbers::pi / 180.0
    };
}

std::optional<JointAngles> resolve_numerical_target(AppState& state) {
    state.numerical_result = num_inverse_kinematics(
        state.upper_arm_length,
        state.forearm_length,
        {state.target_x, state.target_y},
        {state.shoulder_state.angle, state.elbow_state.angle},
        1e-6,
        100,
        0.1
    );
    if (state.numerical_result->status == NumericalIKStatus::converged) {
        return state.numerical_result->angles;
    }

    fail_action(state, "IK did not converge. Active target retained.");
    return std::nullopt;
}

std::optional<JointAngles> resolve_analytical_target(AppState& state) {
    const auto result = inverse_kinematics(
        state.upper_arm_length,
        state.forearm_length,
        {state.target_x, state.target_y}
    );

    if (result.status == IKStatus::success) {
        return result.solutions[result.solutions.size() == 1 ? 0 : state.selected_solution];
    }

    state.target_unreachable = true;
    fail_action(state, "Target is unreachable. Active target retained.");
    return std::nullopt;
}

std::optional<JointAngles> resolve_target(AppState& state) {
    if (state.target_source == AppState::TargetSource::joint_angles) {
        return resolve_joint_target(state);
    }

    if (!std::isfinite(state.target_x) || !std::isfinite(state.target_y)) {
        fail_action(state, "Enter finite target coordinates. Active target retained.");
        return std::nullopt;
    }

    if (state.solver_mode == AppState::SolverMode::numerical) {
        return resolve_numerical_target(state);
    }

    return resolve_analytical_target(state);
}

bool prepare_motion(AppState& state) {
    if (state.task == AppState::Task::pose) {
        return false;
    }

    if (state.task == AppState::Task::position_control) {
        return commit_target(state);
    }

    if (!std::isfinite(state.shoulder_torque) || !std::isfinite(state.elbow_torque)) {
        return fail_action(state, "Enter finite torque commands.");
    }

    state.action_failed = false;
    state.action_message.clear();
    return true;
}

void update_controller_torques(AppState& state) {
    state.shoulder_torque = revolute::pid_torque(
        state.shoulder_controller,
        state.shoulder_gains,
        state.active_target.shoulder,
        state.shoulder_state.angle,
        state.shoulder_state.angular_velocity,
        TIME_STEP,
        state.shoulder_params.torque_limit
    );

    state.elbow_torque = revolute::pid_torque(
        state.elbow_controller,
        state.elbow_gains,
        state.active_target.elbow,
        state.elbow_state.angle,
        state.elbow_state.angular_velocity,
        TIME_STEP,
        state.elbow_params.torque_limit
    );
}

void integrate_joints(AppState& state) {
    state.shoulder_state =
        step_joint(state.shoulder_state, state.shoulder_params, state.shoulder_torque, TIME_STEP);
    state.elbow_state =
        step_joint(state.elbow_state, state.elbow_params, state.elbow_torque, TIME_STEP);

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
}

void record_sample(AppState& state) {
    double shoulder_target = state.active_target.shoulder;
    double elbow_target = state.active_target.elbow;
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

} // namespace

void mark_target_pending(AppState& state) {
    state.target_pending = true;
    state.target_unreachable = false;
    state.numerical_result.reset();
    state.action_failed = false;
    state.action_message.clear();
}

bool commit_target(AppState& state) {
    state.target_unreachable = false;
    state.numerical_result.reset();

    const auto candidate = resolve_target(state);
    if (!candidate) {
        return false;
    }

    state.active_target = *candidate;
    if (state.target_source == AppState::TargetSource::cartesian) {
        state.shoulder_angle_degrees =
            static_cast<float>(candidate->shoulder * 180 / std::numbers::pi);
        state.elbow_angle_degrees = static_cast<float>(candidate->elbow * 180 / std::numbers::pi);
    }

    state.shoulder_controller = {};
    state.elbow_controller = {};

    state.target_pending = false;
    state.action_failed = false;
    state.action_message = "Target committed.";
    return true;
}

bool start_simulation(AppState& state) {
    state.paused = true;
    if (!prepare_motion(state)) {
        return false;
    }

    state.paused = false;
    return true;
}

bool step_simulation(AppState& state) {
    if (!state.paused || !prepare_motion(state)) {
        return false;
    }

    advance_simulation(state);
    return true;
}

void switch_task(AppState& state, SimulationClock& clock, AppState::Task task) {
    if (state.task == task) {
        return;
    }

    clear_motion(state);
    state.task = task;
    clock.accumulator = 0;
    mark_target_pending(state);
}

void advance_simulation(AppState& state) {
    if (state.task == AppState::Task::position_control) {
        update_controller_torques(state);
    }

    integrate_joints(state);
    state.simulation_time += TIME_STEP;
    record_sample(state);
}

bool apply_pose(AppState& state) {
    if (!state.paused || state.task != AppState::Task::pose || !commit_target(state)) {
        return false;
    }

    clear_motion(state);
    state.shoulder_state.angle = state.active_target.shoulder;
    state.elbow_state.angle = state.active_target.elbow;
    state.action_message = "Pose applied.";
    return true;
}

void reset_simulation(AppState& state, SimulationClock& clock) {
    clear_motion(state);
    state.shoulder_state.angle = 0;
    state.elbow_state.angle = std::numbers::pi / 2;
    state.active_target = {state.shoulder_state.angle, state.elbow_state.angle};
    state.simulation_time = 0;
    state.history.clear();
    clock = {};
    mark_target_pending(state);
    state.action_message = "Motion reset. Your setup is preserved.";
}

} // namespace revolute::app
