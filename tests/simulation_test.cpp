#include "simulation.hpp"

#include <doctest/doctest.h>
#include <numbers>

TEST_CASE("Manual simulation action advances both joints once even while paused") {
    revolute::AppState state;
    state.shoulder_torque = 8;
    state.elbow_torque = -8;
    const double original_elbow = state.elbow_state.angle;
    revolute::advance_simulation(state);
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
    revolute::AppState state;
    state.position_control = true;
    state.shoulder_angle_degrees = 30;
    state.elbow_angle_degrees = 60;
    state.shoulder_gains = {5, 1, 2};
    state.elbow_gains = {3, 0.5, 1};
    revolute::advance_simulation(state);
    CHECK(state.shoulder_controller.integral_error > 0);
    CHECK(state.elbow_controller.integral_error < 0);
    CHECK(state.shoulder_state.angular_velocity > 0);
    CHECK(state.elbow_state.angular_velocity < 0);
    CHECK(state.shoulder_torque == state.shoulder_applied_torque);
    CHECK(state.elbow_torque == state.elbow_applied_torque);
}

TEST_CASE("Reset restores simulation defaults and clears the clock") {
    revolute::AppState state;
    state.paused = false;
    state.position_control = true;
    state.shoulder_angle_degrees = 45;
    state.elbow_angle_degrees = -30;
    state.shoulder_gains = {12, 0.5, 8};
    state.elbow_gains = {10, 0.4, 7};
    revolute::advance_simulation(state);
    state.target_unreachable = true;
    state.numerical_result = revolute::NumericalIKResult{};
    revolute::SimulationClock clock{0.007, 0.23};
    revolute::reset_simulation(state, clock);
    CHECK(state.paused);
    CHECK_FALSE(state.position_control);
    CHECK(state.simulation_time == 0);
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
    CHECK(state.shoulder_gains.kp == 5);
    CHECK(state.shoulder_gains.ki == 0);
    CHECK(state.shoulder_gains.kd == 0);
    CHECK(state.elbow_gains.kp == 5);
    CHECK(state.elbow_gains.ki == 0);
    CHECK(state.elbow_gains.kd == 0);
    CHECK_FALSE(state.target_unreachable);
    CHECK_FALSE(state.numerical_result.has_value());
    CHECK(clock.accumulator == 0);
    CHECK(clock.dropped_time == 0);
}

TEST_CASE("IK edits the requested pose and apply pose moves the simulated joints") {
    for (auto mode :
         {revolute::AppState::SolverMode::analytical, revolute::AppState::SolverMode::numerical}) {
        revolute::AppState state;
        state.solver_mode = mode;
        state.target_x = 1.2;
        state.target_y = 0.8;
        state.shoulder_state.angular_velocity = 0.3;
        state.elbow_state.angular_velocity = -0.2;
        revolute::solve_target(state);
        CHECK_FALSE(state.target_unreachable);
        CHECK(state.shoulder_state.angle == 0);
        CHECK(state.elbow_state.angle == doctest::Approx(std::numbers::pi / 2));
        if (mode == revolute::AppState::SolverMode::numerical) {
            REQUIRE(state.numerical_result.has_value());
            CHECK(state.numerical_result->status == revolute::NumericalIKStatus::converged);
        }
        revolute::apply_pose(state);
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
    revolute::AppState state;
    state.shoulder_angle_degrees = 30;
    state.elbow_angle_degrees = 60;
    state.shoulder_torque = 8;
    state.elbow_torque = -8;
    revolute::advance_simulation(state);
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
    revolute::AppState state;
    for (std::size_t step = 0; step < revolute::MAX_HISTORY_SAMPLES + 25; ++step) {
        revolute::advance_simulation(state);
        CHECK(state.history.size() <= revolute::MAX_HISTORY_SAMPLES);
    }
    REQUIRE(state.history.size() == revolute::MAX_HISTORY_SAMPLES);
    CHECK(state.history.front().time == doctest::Approx(26 * revolute::TIME_STEP));
    CHECK(state.history.back().time == doctest::Approx(2025 * revolute::TIME_STEP));
    for (std::size_t index = 1; index < state.history.size(); ++index) {
        CHECK(state.history[index].time - state.history[index - 1].time
            == doctest::Approx(revolute::TIME_STEP));
    }
    revolute::SimulationClock clock;
    revolute::reset_simulation(state, clock);
    CHECK(state.history.empty());
    revolute::advance_simulation(state);
    REQUIRE(state.history.size() == 1);
    CHECK(state.history.front().time == doctest::Approx(revolute::TIME_STEP));
}
