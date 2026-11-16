#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "control.hpp"
#include "dynamics.hpp"

#include <algorithm>
#include <doctest/doctest.h>
#include <limits>
#include <stdexcept>

TEST_CASE("Proportional torque follows signed unwrapped angle error") {
    CHECK(revolute::proportional_torque(0.8, 0.2, 5) == doctest::Approx(3));
    CHECK(revolute::proportional_torque(0.8, 1.0, 5) == doctest::Approx(-1));
    CHECK(revolute::proportional_torque(0.5, 0.5, 5) == 0);
    CHECK(revolute::proportional_torque(1, 0, 0) == 0);
    CHECK(revolute::proportional_torque(3, -3, 5) == 30);
}

TEST_CASE("Derivative torque opposes velocity even at the target") {
    CHECK(revolute::pd_torque(0.5, 0.5, 2, 5, 3) == -6);
    CHECK(revolute::pd_torque(0.5, 0.5, -2, 5, 3) == 6);
    CHECK(revolute::pd_torque(0.5, 0.5, 0, 5, 3) == 0);
    CHECK(revolute::pd_torque(0.8, 0.2, 2, 5, 0)
        == revolute::proportional_torque(0.8, 0.2, 5));
}

TEST_CASE("A target step changes P torque without derivative kick") {
    const double before = revolute::pd_torque(0.5, 0.2, 0.4, 5, 3);
    const double after = revolute::pd_torque(0.8, 0.2, 0.4, 5, 3);
    CHECK(after - before == doctest::Approx(1.5));
    CHECK(before - revolute::proportional_torque(0.5, 0.2, 5) == doctest::Approx(-1.2));
    CHECK(after - revolute::proportional_torque(0.8, 0.2, 5) == doctest::Approx(-1.2));
    // The controller has no stored history; restoring the original inputs
    // immediately restores the original output, including on the first call.
    CHECK(revolute::pd_torque(0.5, 0.2, 0.4, 5, 3) == before);
}

TEST_CASE("PD reduces overshoot in a repeatable joint step response") {
    const auto peak_angle = [](double kd) {
        revolute::JointState state{0, 0};
        const revolute::JointParameters parameters{1, 0.2, 5};
        double peak = 0;
        for (int step = 0; step < 1000; ++step) {
            const double torque = revolute::pd_torque(0.5, state.angle, state.angular_velocity, 5, kd);
            state = revolute::step_joint(state, parameters, torque, 0.01);
            peak = std::max(peak, state.angle);
        }
        if (kd > 0) {
            CHECK(state.angle == doctest::Approx(0.5).epsilon(0.001));
        }
        return peak;
    };
    const double p_peak = peak_angle(0);
    const double pd_peak = peak_angle(2);
    CHECK(p_peak > 0.8);
    CHECK(pd_peak < 0.65);
    CHECK(pd_peak - 0.5 < (p_peak - 0.5) * 0.5);
    CHECK(peak_angle(2) == pd_peak);
}

TEST_CASE("Controllers reject non-finite inputs and negative gains") {
    for (double invalid : {std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        CHECK_THROWS_AS(revolute::proportional_torque(invalid, 0, 5), std::invalid_argument);
        CHECK_THROWS_AS(revolute::proportional_torque(0, invalid, 5), std::invalid_argument);
        CHECK_THROWS_AS(revolute::proportional_torque(0, 0, invalid), std::invalid_argument);
        CHECK_THROWS_AS(revolute::pd_torque(invalid, 0, 0, 5, 2), std::invalid_argument);
        CHECK_THROWS_AS(revolute::pd_torque(0, invalid, 0, 5, 2), std::invalid_argument);
        CHECK_THROWS_AS(revolute::pd_torque(0, 0, invalid, 5, 2), std::invalid_argument);
        CHECK_THROWS_AS(revolute::pd_torque(0, 0, 0, invalid, 2), std::invalid_argument);
        CHECK_THROWS_AS(revolute::pd_torque(0, 0, 0, 5, invalid), std::invalid_argument);
    }
    CHECK_THROWS_AS(revolute::proportional_torque(0, 0, -1), std::invalid_argument);
    CHECK_THROWS_AS(revolute::pd_torque(0, 0, 0, -1, 2), std::invalid_argument);
    CHECK_THROWS_AS(revolute::pd_torque(0, 0, 0, 5, -1), std::invalid_argument);
    CHECK(revolute::pd_torque(1, 0, 2, 0, 0) == 0);
}

TEST_CASE("PID rejects non-finite inputs without changing integral state") {
    const revolute::PIDGains gains{5, 1, 2};
    for (double invalid : {std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        revolute::ControllerState state{0.25};
        CHECK_THROWS_AS(revolute::pid_torque(state, gains, invalid, 0, 0, 0.01, 5), std::invalid_argument);
        CHECK_THROWS_AS(revolute::pid_torque(state, gains, 1, invalid, 0, 0.01, 5), std::invalid_argument);
        CHECK_THROWS_AS(revolute::pid_torque(state, gains, 1, 0, invalid, 0.01, 5), std::invalid_argument);
        CHECK_THROWS_AS(revolute::pid_torque(state, gains, 1, 0, 0, invalid, 5), std::invalid_argument);
        CHECK_THROWS_AS(revolute::pid_torque(state, gains, 1, 0, 0, 0.01, invalid), std::invalid_argument);
        CHECK(state.integral_error == 0.25);
        revolute::ControllerState invalid_state{invalid};
        CHECK_THROWS_AS(revolute::pid_torque(invalid_state, gains, 1, 0, 0, 0.01, 5), std::invalid_argument);
    }
}

TEST_CASE("PID validates gains and positive timestep while permitting zero gains and limit") {
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        revolute::ControllerState state{0.25};
        for (const revolute::PIDGains gains : {revolute::PIDGains{invalid, 1, 2},
                                             {5, invalid, 2}, {5, 1, invalid}}) {
            CHECK_THROWS_AS(revolute::pid_torque(state, gains, 1, 0, 0, 0.01, 5), std::invalid_argument);
            CHECK(state.integral_error == 0.25);
        }
    }
    revolute::ControllerState state{0.25};
    const revolute::PIDGains gains{5, 1, 2};
    for (double dt : {0.0, -0.01}) {
        CHECK_THROWS_AS(revolute::pid_torque(state, gains, 1, 0, 0, dt, 5), std::invalid_argument);
        CHECK(state.integral_error == 0.25);
    }
    CHECK_THROWS_AS(revolute::pid_torque(state, gains, 1, 0, 0, 0.01, -5), std::invalid_argument);
    CHECK(state.integral_error == 0.25);
    CHECK(revolute::pid_torque(state, {0, 0, 0}, 1, 0, 0, 0.01, 0) == 0);
}

TEST_CASE("PID integral accumulation scales with elapsed simulation time") {
    for (int steps : {10, 100, 1000}) {
        revolute::ControllerState state;
        double torque = 0;
        for (int step = 0; step < steps; ++step) {
            torque = revolute::pid_torque(state, {0, 2, 0}, 0.5, 0, 0, 1.0 / steps, 5);
        }
        CHECK(state.integral_error == doctest::Approx(0.5));
        CHECK(torque == doctest::Approx(1.0));
    }
}

TEST_CASE("Conditional integration prevents prolonged saturation windup in both directions") {
    for (double direction : {-1.0, 1.0}) {
        revolute::ControllerState state;
        double naive_integral = 0;
        // Hold the joint fixed, modelling a blocked actuator that cannot reach the target.
        for (int step = 0; step < 1000; ++step) {
            const double torque = revolute::pid_torque(state, {5, 1, 0}, direction * 10, 0, 0, 0.01, 5);
            CHECK(torque == direction * 5);
            CHECK(state.integral_error == 0);
            naive_integral += direction * 10 * 0.01;
        }
        // A small reversed target should immediately reverse torque. A naive
        // integrator still commands the old direction due to its stored error.
        const double reversed_error = -direction * 0.1;
        const double recovered = revolute::pid_torque(state, {5, 1, 0}, reversed_error, 0, 0, 0.01, 5);
        CHECK(recovered * direction < 0);
        CHECK(recovered == doctest::Approx(-direction * 0.501));
        const double naive_torque = std::clamp(5 * reversed_error + naive_integral, -5.0, 5.0);
        CHECK(naive_torque == direction * 5);
    }
}

TEST_CASE("Conditional integration permits unwinding an existing saturated integral") {
    for (double direction : {-1.0, 1.0}) {
        revolute::ControllerState state{direction * 10};
        const double torque = revolute::pid_torque(state, {0, 1, 0}, -direction, 0, 0, 0.1, 5);
        CHECK(state.integral_error == doctest::Approx(direction * 9.9));
        CHECK(torque == direction * 5);
    }
}

TEST_CASE("Resetting controller state clears accumulated integral torque") {
    revolute::ControllerState state;
    const revolute::PIDGains gains{0, 2, 0};
    CHECK(revolute::pid_torque(state, gains, 1, 0, 0, 0.5, 5) == 1);
    CHECK(revolute::pid_torque(state, gains, 0, 0, 0, 0.01, 5) == 1);
    state = revolute::ControllerState{};
    CHECK(state.integral_error == 0);
    CHECK(revolute::pid_torque(state, gains, 0, 0, 0, 0.01, 5) == 0);
}
