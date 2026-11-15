#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "dynamics.hpp"

#include <array>
#include <cmath>
#include <doctest/doctest.h>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

// Integer step counts keep every comparison at exactly the requested duration.
revolute::JointState simulate(
    revolute::JointState state,
    const revolute::JointParameters& parameters,
    double torque,
    double duration,
    int steps
) {
    const double dt = duration / steps;
    for (int step = 0; step < steps; ++step) {
        state = revolute::step_joint(state, parameters, torque, dt);
    }
    return state;
}

} // namespace

TEST_CASE("One joint step matches a hand-calculated semi-implicit Euler update") {
    const revolute::JointState initial{0, 2};
    const revolute::JointParameters parameters{2, 1, 5};

    const auto result = revolute::step_joint(initial, parameters, 8, 0.1);

    // Torque clamps to 5; acceleration = (5 - 1 * 2) / 2 = 1.5 rad/s².
    // New velocity = 2 + 1.5 * 0.1 = 2.15 rad/s.
    // Semi-implicit Euler uses that NEW velocity: angle = 0 + 2.15 * 0.1.
    CHECK(result.angular_velocity == doctest::Approx(2.15));
    CHECK(result.angle == doctest::Approx(0.215));
    CHECK(initial.angle == 0);
    CHECK(initial.angular_velocity == 2);
}

TEST_CASE("One joint step clamps negative commands and permits negative acceleration") {
    const revolute::JointParameters parameters{2, 1, 5};
    const auto result = revolute::step_joint({1, 2}, parameters, -8, 0.1);

    // Applied torque = -5; acceleration = (-5 - 2) / 2 = -3.5 rad/s².
    CHECK(result.angular_velocity == doctest::Approx(1.65));
    CHECK(result.angle == doctest::Approx(1.165));
}

TEST_CASE("Joint stepping rejects invalid inertia before dividing") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double inertia : {0.0, -1.0, infinity, -infinity, nan}) {
        CAPTURE(inertia);
        const revolute::JointParameters parameters{inertia, 1, 5};
        CHECK_THROWS_AS(revolute::step_joint({0, 2}, parameters, 8, 0.1), std::invalid_argument);
    }
}

TEST_CASE("Joint stepping rejects invalid timesteps") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const revolute::JointParameters parameters{2, 1, 5};
    for (double dt : {0.0, -0.1, infinity, -infinity, nan}) {
        CAPTURE(dt);
        CHECK_THROWS_AS(revolute::step_joint({0, 2}, parameters, 8, dt), std::invalid_argument);
    }
}

TEST_CASE("An unforced stationary joint remains stationary over time") {
    for (double damping : {0.0, 1.0}) {
        const auto result = simulate({1.25, 0}, {2, damping, 5}, 0, 10, 1000);
        CHECK(result.angle == 1.25);
        CHECK(result.angular_velocity == 0);
    }
}

TEST_CASE("An unforced undamped joint keeps its velocity in either direction") {
    for (double velocity : {-2.0, 2.0}) {
        CAPTURE(velocity);
        const auto result = simulate({1, velocity}, {2, 0, 5}, 0, 3, 300);
        CHECK(result.angular_velocity == velocity);
        CHECK(result.angle == doctest::Approx(1 + velocity * 3).epsilon(1e-12));
    }
}

TEST_CASE("Constant torque follows analytical motion with first-order position convergence") {
    const revolute::JointState initial{0.5, -0.25};
    const revolute::JointParameters parameters{2, 0, 5};
    constexpr double duration = 2;

    for (double torque : {-3.0, 3.0}) {
        const double acceleration = torque / parameters.inertia;
        const double expected_velocity = initial.angular_velocity + acceleration * duration;
        const double expected_angle = initial.angle
            + initial.angular_velocity * duration
            + 0.5 * acceleration * duration * duration;
        double previous_error = 0;

        for (int steps : {20, 40, 80, 160}) {
            CAPTURE(torque);
            CAPTURE(steps);
            const auto result = simulate(initial, parameters, torque, duration, steps);
            CHECK(result.angular_velocity == doctest::Approx(expected_velocity).epsilon(1e-12));
            const double error = std::abs(result.angle - expected_angle);
            // Semi-implicit Euler has first-order global position error:
            // halving dt should approximately halve the error at fixed duration.
            if (previous_error > 0) {
                CHECK(error < previous_error);
                CHECK(error / previous_error == doctest::Approx(0.5).epsilon(1e-10));
            }
            previous_error = error;
        }
        CHECK(previous_error < 0.02);
    }
}

TEST_CASE("Damping slows an unforced joint and approaches exponential decay") {
    const revolute::JointParameters parameters{2, 1, 5};
    constexpr double duration = 2;
    const double decay_rate = parameters.damping / parameters.inertia;

    for (double velocity : {-2.0, 2.0}) {
        const revolute::JointState initial{0.5, velocity};
        const double expected_velocity = velocity * std::exp(-decay_rate * duration);
        const double expected_angle =
            initial.angle + velocity / decay_rate * (1 - std::exp(-decay_rate * duration));
        double previous_velocity_error = std::numeric_limits<double>::infinity();
        double previous_angle_error = std::numeric_limits<double>::infinity();

        for (int steps : {20, 40, 80, 160}) {
            CAPTURE(velocity);
            CAPTURE(steps);
            auto state = initial;
            const double dt = duration / steps;
            // These timesteps satisfy damping * dt / inertia < 1,
            // so the explicit velocity update should decay without sign flips.
            for (int step = 0; step < steps; ++step) {
                const auto next = revolute::step_joint(state, parameters, 0, dt);
                CHECK(std::abs(next.angular_velocity) < std::abs(state.angular_velocity));
                CHECK(next.angular_velocity * velocity > 0);
                state = next;
            }
            const double velocity_error = std::abs(state.angular_velocity - expected_velocity);
            const double angle_error = std::abs(state.angle - expected_angle);
            CHECK(velocity_error < previous_velocity_error);
            CHECK(angle_error < previous_angle_error);
            previous_velocity_error = velocity_error;
            previous_angle_error = angle_error;
        }
        CHECK(previous_velocity_error < 0.003);
        CHECK(previous_angle_error < 0.012);
    }
}

TEST_CASE("Torque saturation bounds undamped acceleration over repeated steps") {
    const revolute::JointParameters parameters{2, 0, 5};
    constexpr double dt = 0.01;
    for (double direction : {-1.0, 1.0}) {
        revolute::JointState state{0, 0};
        for (int step = 0; step < 200; ++step) {
            const auto next = revolute::step_joint(state, parameters, direction * 100, dt);
            const double acceleration = (next.angular_velocity - state.angular_velocity) / dt;
            CHECK(acceleration == doctest::Approx(direction * 2.5).epsilon(1e-12));
            state = next;
        }
        CHECK(state.angular_velocity == doctest::Approx(direction * 5).epsilon(1e-12));
        const auto at_limit = simulate({0, 0}, parameters, direction * 5, 2, 200);
        CHECK(state.angle == at_limit.angle);
        CHECK(state.angular_velocity == at_limit.angular_velocity);
        const auto repeated = simulate({0, 0}, parameters, direction * 100, 2, 200);
        CHECK(state.angle == repeated.angle);
        CHECK(state.angular_velocity == repeated.angular_velocity);
    }
    const auto disabled = simulate({1, 0}, {2, 0, 0}, 100, 2, 200);
    CHECK(disabled.angle == 1);
    CHECK(disabled.angular_velocity == 0);
}

TEST_CASE("Frame schedules produce identical joint-state sequences without dropped time") {
    constexpr double dt = 0.01;
    using Pose = std::array<revolute::JointState, 2>;
    const auto run = [&](const std::vector<double>& frames) {
        revolute::SimulationClock clock;
        Pose joints{{{0.2, 0.3}, {1.1, -0.2}}};
        std::vector<Pose> trace;
        for (double elapsed : frames) {
            const int updates = revolute::advance_simulation_clock(clock, elapsed, dt, 10);
            for (int update = 0; update < updates; ++update) {
                // Commands depend on physics-step number, not rendering frequency.
                const double shoulder_torque = trace.size() < 100 ? 2.0 : -1.0;
                joints[0] = revolute::step_joint(joints[0], {1, 0.2, 5}, shoulder_torque, dt);
                joints[1] = revolute::step_joint(joints[1], {2, 0.4, 3}, -0.5, dt);
                trace.push_back(joints);
            }
        }
        CHECK(clock.dropped_time == 0);
        CHECK(std::abs(clock.accumulator) < 1e-12);
        return trace;
    };

    const auto reference = run(std::vector<double>(200, dt));
    REQUIRE(reference.size() == 200);
    std::vector<std::vector<double>> schedules;
    for (int fps : {30, 60, 144}) {
        schedules.emplace_back(2 * fps, 1.0 / fps);
    }
    std::vector<double> irregular;
    for (int pair = 0; pair < 40; ++pair) {
        irregular.push_back(0.003);
        irregular.push_back(0.047);
    }
    schedules.push_back(irregular);
    for (const auto& frames : schedules) {
        CAPTURE(frames.size());
        const auto actual = run(frames);
        REQUIRE(actual.size() == reference.size());
        for (std::size_t step = 0; step < reference.size(); ++step) {
            for (std::size_t joint = 0; joint < 2; ++joint) {
                CHECK(actual[step][joint].angle == reference[step][joint].angle);
                CHECK(actual[step][joint].angular_velocity == reference[step][joint].angular_velocity);
            }
        }
    }
}

TEST_CASE("Simulation clock retains fractional time and handles exact boundaries") {
    revolute::SimulationClock clock;
    CHECK(revolute::advance_simulation_clock(clock, 0.007, 0.01, 10) == 0);
    CHECK(clock.accumulator == doctest::Approx(0.007));
    CHECK(revolute::advance_simulation_clock(clock, 0.003, 0.01, 10) == 1);
    CHECK(std::abs(clock.accumulator) < 1e-14);
    CHECK(revolute::advance_simulation_clock(clock, 0, 0.01, 10) == 0);
    CHECK(revolute::advance_simulation_clock(clock, 0.1, 0.01, 10) == 10);
    CHECK(clock.dropped_time == 0);
    CHECK(std::abs(clock.accumulator) < 1e-14);
}

TEST_CASE("Simulation clock caps work and accounts for dropped time across frames") {
    revolute::SimulationClock clock;
    CHECK(revolute::advance_simulation_clock(clock, 0.337, 0.01, 10) == 10);
    CHECK(clock.accumulator == doctest::Approx(0.007));
    CHECK(clock.dropped_time == doctest::Approx(0.23));
    // simulated + dropped + remainder accounts for all elapsed time.
    CHECK(0.1 + clock.dropped_time + clock.accumulator == doctest::Approx(0.337));
    CHECK(revolute::advance_simulation_clock(clock, 0.003, 0.01, 10) == 1);
    CHECK(std::abs(clock.accumulator) < 1e-14);
    CHECK(clock.dropped_time == doctest::Approx(0.23));
    CHECK(revolute::advance_simulation_clock(clock, 0.2, 0.01, 10) == 10);
    CHECK(clock.dropped_time == doctest::Approx(0.33));
    CHECK(std::abs(clock.accumulator) < 1e-14);
}

TEST_CASE("Simulation clock rejects invalid timing inputs") {
    revolute::SimulationClock clock;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double invalid : {-1.0, nan, inf}) {
        CHECK_THROWS_AS(revolute::advance_simulation_clock(clock, invalid, 0.01, 10), std::invalid_argument);
        CHECK_THROWS_AS(revolute::advance_simulation_clock(clock, 0.01, invalid, 10), std::invalid_argument);
    }
    CHECK_THROWS_AS(revolute::advance_simulation_clock(clock, 0.01, 0, 10), std::invalid_argument);
    CHECK_THROWS_AS(revolute::advance_simulation_clock(clock, 0.01, 0.01, 0), std::invalid_argument);
    CHECK_THROWS_AS(revolute::advance_simulation_clock(clock, 0.01, 0.01, -1), std::invalid_argument);
    CHECK(clock.accumulator == 0);
    CHECK(clock.dropped_time == 0);
}
