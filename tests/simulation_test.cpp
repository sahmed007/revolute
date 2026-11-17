#include "revolute/app/simulation.hpp"

#include <doctest/doctest.h>
#include <limits>
#include <numbers>

TEST_CASE("Manual simulation action advances both joints once even while paused") {
    revolute::app::AppState state;
    state.shoulder_torque = 8;
    state.elbow_torque = -8;
    const double original_elbow = state.elbow_state.angle;
    revolute::app::advance_simulation(state);
    CHECK(state.paused);
    CHECK(state.simulation_time == doctest::Approx(0.01));
    CHECK(state.shoulder_state.angular_velocity == doctest::Approx(0.05));
    CHECK(state.shoulder_state.angle == doctest::Approx(0.0005));
    CHECK(state.elbow_state.angular_velocity == doctest::Approx(-0.05));
    CHECK(state.elbow_state.angle == doctest::Approx(original_elbow - 0.0005));
    CHECK(state.shoulder_applied_torque == 5);
    CHECK(state.elbow_applied_torque == -5);
}

TEST_CASE("Simulation action calculates each joint controller before stepping") {
    revolute::app::AppState state;
    state.task = revolute::app::AppState::Task::position_control;
    state.shoulder_angle_degrees = 30;
    state.elbow_angle_degrees = 60;
    state.shoulder_gains = {5, 1, 2};
    state.elbow_gains = {3, 0.5, 1};
    REQUIRE(revolute::app::commit_target(state));
    revolute::app::advance_simulation(state);
    CHECK(state.shoulder_controller.integral_error > 0);
    CHECK(state.elbow_controller.integral_error < 0);
    CHECK(state.shoulder_state.angular_velocity > 0);
    CHECK(state.elbow_state.angular_velocity < 0);
    CHECK(state.shoulder_torque == state.shoulder_applied_torque);
    CHECK(state.elbow_torque == state.elbow_applied_torque);
}

TEST_CASE("Reset restores motion and preserves setup") {
    revolute::app::AppState state;
    state.paused = false;
    state.task = revolute::app::AppState::Task::position_control;
    state.shoulder_angle_degrees = 45;
    state.elbow_angle_degrees = -30;
    state.shoulder_gains = {12, 0.5, 8};
    state.elbow_gains = {10, 0.4, 7};
    revolute::app::advance_simulation(state);
    state.target_unreachable = true;
    state.numerical_result = revolute::NumericalIKResult{};
    revolute::SimulationClock clock{0.007, 0.23};
    revolute::app::reset_simulation(state, clock);
    CHECK(state.paused);
    CHECK(state.task == revolute::app::AppState::Task::position_control);
    CHECK(state.simulation_time == 0);
    CHECK(state.shoulder_angle_degrees == 45);
    CHECK(state.elbow_angle_degrees == -30);
    CHECK(state.active_target.shoulder == 0);
    CHECK(state.active_target.elbow == doctest::Approx(std::numbers::pi / 2));
    CHECK(state.target_pending);
    CHECK(state.shoulder_state.angle == 0);
    CHECK(state.elbow_state.angle == doctest::Approx(std::numbers::pi / 2));
    CHECK(state.shoulder_state.angular_velocity == 0);
    CHECK(state.elbow_state.angular_velocity == 0);
    CHECK(state.shoulder_torque == 0);
    CHECK(state.elbow_torque == 0);
    CHECK(state.shoulder_applied_torque == 0);
    CHECK(state.elbow_applied_torque == 0);
    CHECK(state.shoulder_controller.integral_error == 0);
    CHECK(state.elbow_controller.integral_error == 0);
    CHECK(state.shoulder_gains.kp == 12);
    CHECK(state.shoulder_gains.ki == 0.5);
    CHECK(state.shoulder_gains.kd == 8);
    CHECK(state.elbow_gains.kp == 10);
    CHECK(state.elbow_gains.ki == 0.4);
    CHECK(state.elbow_gains.kd == 7);
    CHECK_FALSE(state.target_unreachable);
    CHECK_FALSE(state.numerical_result.has_value());
    CHECK(clock.accumulator == 0);
    CHECK(clock.dropped_time == 0);
}

TEST_CASE("IK edits the requested pose and apply pose moves the simulated joints") {
    for (auto mode :
         {revolute::app::AppState::SolverMode::analytical,
          revolute::app::AppState::SolverMode::numerical}) {
        revolute::app::AppState state;
        state.solver_mode = mode;
        state.target_source = revolute::app::AppState::TargetSource::cartesian;
        state.target_x = 1.2;
        state.target_y = 0.8;
        state.shoulder_state.angular_velocity = 0.3;
        state.elbow_state.angular_velocity = -0.2;
        REQUIRE(revolute::app::commit_target(state));
        CHECK_FALSE(state.target_unreachable);
        CHECK(state.shoulder_state.angle == 0);
        CHECK(state.elbow_state.angle == doctest::Approx(std::numbers::pi / 2));
        if (mode == revolute::app::AppState::SolverMode::numerical) {
            REQUIRE(state.numerical_result.has_value());
            CHECK(state.numerical_result->status == revolute::NumericalIKStatus::converged);
        }
        REQUIRE(revolute::app::apply_pose(state));
        const auto pose =
            revolute::forward_kinematics(1, 1, state.shoulder_state.angle, state.elbow_state.angle);
        CHECK(pose.hand.x == doctest::Approx(1.2).epsilon(1e-5));
        CHECK(pose.hand.y == doctest::Approx(0.8).epsilon(1e-5));
        CHECK(state.shoulder_state.angular_velocity == 0);
        CHECK(state.elbow_state.angular_velocity == 0);
        CHECK(state.simulation_time == 0);
    }
}

TEST_CASE("Simulation records post-step states and applied torque with radian targets") {
    revolute::app::AppState state;
    state.shoulder_angle_degrees = 30;
    state.elbow_angle_degrees = 60;
    REQUIRE(revolute::app::commit_target(state));
    state.shoulder_torque = 8;
    state.elbow_torque = -8;
    revolute::app::advance_simulation(state);
    REQUIRE(state.history.size() == 1);
    const auto& sample = state.history.back();
    CHECK(sample.time == doctest::Approx(0.01));
    CHECK(sample.shoulder.target_angle == doctest::Approx(std::numbers::pi / 6));
    CHECK(sample.elbow.target_angle == doctest::Approx(std::numbers::pi / 3));
    CHECK(sample.shoulder.angle == doctest::Approx(0.0005));
    CHECK(sample.elbow.angle == doctest::Approx(std::numbers::pi / 2 - 0.0005));
    CHECK(sample.shoulder.angular_velocity == doctest::Approx(0.05));
    CHECK(sample.elbow.angular_velocity == doctest::Approx(-0.05));
    CHECK(sample.shoulder.applied_torque == 5);
    CHECK(sample.elbow.applied_torque == -5);
    CHECK(sample.shoulder.error == doctest::Approx(std::numbers::pi / 6 - 0.0005));
    CHECK(sample.elbow.error == doctest::Approx(-std::numbers::pi / 6 + 0.0005));
}

TEST_CASE("Simulation history retains the newest samples and reset clears it") {
    revolute::app::AppState state;
    for (std::size_t step = 0; step < revolute::app::MAX_HISTORY_SAMPLES + 25; ++step) {
        revolute::app::advance_simulation(state);
        CHECK(state.history.size() <= revolute::app::MAX_HISTORY_SAMPLES);
    }
    REQUIRE(state.history.size() == revolute::app::MAX_HISTORY_SAMPLES);
    CHECK(state.history.front().time == doctest::Approx(26 * revolute::app::TIME_STEP));
    CHECK(state.history.back().time == doctest::Approx(2025 * revolute::app::TIME_STEP));
    for (std::size_t index = 1; index < state.history.size(); ++index) {
        CHECK(
            state.history[index].time - state.history[index - 1].time
            == doctest::Approx(revolute::app::TIME_STEP)
        );
    }
    revolute::SimulationClock clock;
    revolute::app::reset_simulation(state, clock);
    CHECK(state.history.empty());
    revolute::app::advance_simulation(state);
    REQUIRE(state.history.size() == 1);
    CHECK(state.history.front().time == doctest::Approx(revolute::app::TIME_STEP));
}

TEST_CASE("Draft changes do not change the active PID target or recorded error") {
    revolute::app::AppState state;
    state.task = revolute::app::AppState::Task::position_control;
    REQUIRE(revolute::app::commit_target(state));
    state.shoulder_angle_degrees = 90;
    revolute::app::mark_target_pending(state);
    revolute::app::advance_simulation(state);
    CHECK(state.shoulder_state.angle == 0);
    CHECK(state.history.back().shoulder.target_angle == 0);
    CHECK(state.target_pending);
    REQUIRE(revolute::app::commit_target(state));
    revolute::app::advance_simulation(state);
    CHECK(state.shoulder_state.angle > 0);
    CHECK_FALSE(state.target_pending);
}

TEST_CASE("Failed Cartesian actions retain pose and active target") {
    for (auto mode :
         {revolute::app::AppState::SolverMode::analytical,
          revolute::app::AppState::SolverMode::numerical}) {
        revolute::app::AppState state;
        state.target_source = revolute::app::AppState::TargetSource::cartesian;
        state.solver_mode = mode;
        state.target_x = 10;
        const auto original = state.active_target;
        CHECK_FALSE(revolute::app::apply_pose(state));
        CHECK(state.shoulder_state.angle == 0);
        CHECK(state.active_target.shoulder == original.shoulder);
        CHECK(state.active_target.elbow == original.elbow);
        state.task = revolute::app::AppState::Task::position_control;
        CHECK_FALSE(revolute::app::start_simulation(state));
        CHECK(state.paused);
        CHECK_FALSE(revolute::app::step_simulation(state));
        CHECK(state.simulation_time == 0);
        state.paused = false;
        CHECK_FALSE(revolute::app::commit_target(state));
        CHECK_FALSE(state.paused);
        CHECK(state.action_failed);
    }
}

TEST_CASE("Task switching clears motion and commands but preserves pose and setup") {
    revolute::app::AppState state;
    state.task = revolute::app::AppState::Task::manual_torque;
    state.paused = false;
    state.shoulder_torque = 2;
    revolute::app::advance_simulation(state);
    const double angle = state.shoulder_state.angle;
    const double time = state.simulation_time;
    state.shoulder_controller.integral_error = 2;
    state.shoulder_angle_degrees = 35;
    revolute::SimulationClock clock{0.005, 0.2};
    revolute::app::switch_task(state, clock, revolute::app::AppState::Task::position_control);
    CHECK(state.paused);
    CHECK(state.shoulder_state.angle == angle);
    CHECK(state.shoulder_state.angular_velocity == 0);
    CHECK(state.shoulder_torque == 0);
    CHECK(state.shoulder_applied_torque == 0);
    CHECK(state.shoulder_controller.integral_error == 0);
    CHECK(state.shoulder_angle_degrees == 35);
    CHECK(state.simulation_time == time);
    CHECK(state.history.size() == 1);
    CHECK(clock.accumulator == 0);
    CHECK(clock.dropped_time == 0.2);
}

TEST_CASE("Step commits targets and advances once without starting playback") {
    revolute::app::AppState state;
    CHECK_FALSE(revolute::app::step_simulation(state));
    state.task = revolute::app::AppState::Task::position_control;
    state.shoulder_angle_degrees = 40;
    REQUIRE(revolute::app::step_simulation(state));
    CHECK(state.paused);
    CHECK(state.simulation_time == doctest::Approx(revolute::app::TIME_STEP));
    CHECK(state.shoulder_state.angle > 0);
    REQUIRE(revolute::app::start_simulation(state));
    CHECK_FALSE(revolute::app::step_simulation(state));
    CHECK(state.history.size() == 1);
}

TEST_CASE("Nonfinite targets never replace active commands") {
    revolute::app::AppState state;
    state.shoulder_angle_degrees = std::numeric_limits<float>::infinity();
    CHECK_FALSE(revolute::app::commit_target(state));
    CHECK(state.active_target.shoulder == 0);
    state.target_source = revolute::app::AppState::TargetSource::cartesian;
    state.target_x = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(revolute::app::commit_target(state));
    CHECK(state.active_target.shoulder == 0);
}

TEST_CASE("Running Cartesian updates commit without moving joints or stopping playback") {
    revolute::app::AppState state;
    state.task = revolute::app::AppState::Task::position_control;
    REQUIRE(revolute::app::start_simulation(state));
    state.target_source = revolute::app::AppState::TargetSource::cartesian;
    state.target_x = 1.2;
    state.target_y = 0.8;
    state.shoulder_controller.integral_error = 3;
    revolute::app::mark_target_pending(state);
    REQUIRE(revolute::app::commit_target(state));
    CHECK_FALSE(state.paused);
    CHECK(state.shoulder_state.angle == 0);
    CHECK(state.shoulder_controller.integral_error == 0);
    const auto pose =
        revolute::forward_kinematics(1, 1, state.active_target.shoulder, state.active_target.elbow);
    CHECK(pose.hand.x == doctest::Approx(1.2));
    CHECK(pose.hand.y == doctest::Approx(0.8));
}
