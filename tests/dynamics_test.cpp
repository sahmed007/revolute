#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "dynamics.hpp"

#include <doctest/doctest.h>
#include <limits>
#include <stdexcept>

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
