#include "control.hpp"
#include "dynamics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <doctest/doctest.h>
#include <iostream>
#include <numbers>

namespace {
constexpr double degrees = 180.0 / std::numbers::pi;
constexpr double duration = 10.0;
constexpr double band = 1.0 / degrees;
const std::array<double, 2> targets{30.0 / degrees, 60.0 / degrees};
const std::array<revolute::PIDGains, 2> gains{{{12, 0.5, 8}, {10, 0.4, 7}}};

struct Metrics {
    double final_error = 0;
    double tail_error = 0;
    double overshoot = 0;
    double settling_time = 0;
    double peak_torque = 0;
};

struct Scenario {
    const char* name;
    std::array<revolute::JointParameters, 2> parameters;
    bool disturbed;
};

std::array<Metrics, 2> track(const Scenario& scenario, double dt) {
    std::array<revolute::JointState, 2> joints{{{0, 0}, {90.0 / degrees, 0}}};
    std::array<revolute::ControllerState, 2> controllers{};
    const auto initial = joints;
    std::array<Metrics, 2> metrics{};
    const int steps = static_cast<int>(std::lround(duration / dt));
    const int disturbance_step = static_cast<int>(std::lround(4.0 / dt));
    const int tail_start = static_cast<int>(std::lround(8.0 / dt));
    for (int step = 0; step < steps; ++step) {
        if (scenario.disturbed && step == disturbance_step) {
            // External impulse: change velocity, not actuator command. This avoids
            // incorrectly clipping the disturbance through the motor torque limit.
            joints[0].angular_velocity += 0.35;
            joints[1].angular_velocity -= 0.25;
        }
        for (std::size_t joint = 0; joint < 2; ++joint) {
            const auto& parameters = scenario.parameters[joint];
            const double torque = revolute::pid_torque(
                controllers[joint], gains[joint], targets[joint], joints[joint].angle,
                joints[joint].angular_velocity, dt, parameters.torque_limit
            );
            joints[joint] = revolute::step_joint(joints[joint], parameters, torque, dt);
            auto& result = metrics[joint];
            const double error = std::abs(targets[joint] - joints[joint].angle);
            REQUIRE(std::isfinite(error));
            REQUIRE(std::isfinite(joints[joint].angular_velocity));
            result.final_error = error;
            result.peak_torque = std::max(result.peak_torque, std::abs(torque));
            if (step + 1 >= tail_start) {
                result.tail_error = std::max(result.tail_error, error);
            }
            if (error > band) {
                // First possible in-band sample after the last violation.
                result.settling_time = (step + 2) * dt;
            }
            const double change = targets[joint] - initial[joint].angle;
            const double signed_overshoot = (joints[joint].angle - targets[joint]) / change;
            result.overshoot = std::max(result.overshoot, signed_overshoot);
        }
    }
    return metrics;
}
} // namespace

TEST_CASE("Both PID joints meet tracking thresholds across dynamics and timesteps") {
    const std::array<Scenario, 3> scenarios{{
        {"nominal", {{{1, 0.2, 5}, {1, 0.2, 5}}}, false},
        {"altered", {{{2, 0.1, 5}, {0.5, 0.6, 5}}}, false},
        {"disturbed", {{{1, 0.2, 5}, {1, 0.2, 5}}}, true}
    }};
    for (const auto& scenario : scenarios) {
        for (double dt : {0.02, 0.01, 0.005}) {
            const auto results = track(scenario, dt);
            for (std::size_t joint = 0; joint < 2; ++joint) {
                CAPTURE(scenario.name);
                CAPTURE(dt);
                CAPTURE(joint);
                const auto& result = results[joint];
                CHECK(result.final_error < band);
                CHECK(result.tail_error <= band);
                CHECK(result.settling_time <= 8.0);
                CHECK(result.overshoot < 0.10);
                CHECK(result.peak_torque <= scenario.parameters[joint].torque_limit);
                if (dt == 0.01) {
                    std::cout << scenario.name << " joint=" << joint
                              << " final_deg=" << result.final_error * degrees
                              << " tail_max_deg=" << result.tail_error * degrees
                              << " overshoot_pct=" << result.overshoot * 100
                              << " settled_s=" << result.settling_time
                              << " peak_torque=" << result.peak_torque << '\n';
                }
            }
        }
    }
}

TEST_CASE("Measured-velocity derivative torque does not acquire a timestep factor") {
    for (double dt : {0.02, 0.01, 0.005}) {
        revolute::ControllerState state;
        CHECK(revolute::pid_torque(state, {0, 0, 3}, 0.5, 0.5, 2, dt, 10) == -6);
        CHECK(revolute::pid_torque(state, {0, 0, 3}, 0.5, 0.5, -2, dt, 10) == 6);
    }
}

TEST_CASE("Joint controllers maintain independent integral histories") {
    revolute::ControllerState shoulder;
    revolute::ControllerState elbow;
    const revolute::PIDGains integral_only{0, 1, 0};
    for (int step = 0; step < 100; ++step) {
        revolute::pid_torque(shoulder, integral_only, 1, 0, 0, 0.01, 5);
        revolute::pid_torque(elbow, integral_only, -0.5, 0, 0, 0.01, 5);
    }
    CHECK(shoulder.integral_error == doctest::Approx(1));
    CHECK(elbow.integral_error == doctest::Approx(-0.5));
    shoulder = revolute::ControllerState{};
    CHECK(elbow.integral_error == doctest::Approx(-0.5));
}
