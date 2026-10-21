#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "kinematics.hpp"
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace {

// Absolute tolerance in meters for these meter-scale, double-precision examples.
constexpr double tolerance = 1e-10;
constexpr double pi = std::numbers::pi;

void check_point(revolute::Point2D actual, revolute::Point2D expected) {
    CHECK(std::abs(actual.x - expected.x) <= tolerance);
    CHECK(std::abs(actual.y - expected.y) <= tolerance);
}

double distance(revolute::Point2D a, revolute::Point2D b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

} // namespace

TEST_CASE("Known one-meter arm poses match hand calculations") {
    struct Example {
        double shoulder;
        double elbow;
        revolute::Point2D expected_elbow;
        revolute::Point2D expected_hand;
    };
    const Example examples[] = {
        {0, 0, {1, 0}, {2, 0}},
        {pi / 2, 0, {0, 1}, {0, 2}},
        {0, pi / 2, {1, 0}, {1, 1}},
        {0, pi, {1, 0}, {0, 0}},
    };

    for (const auto& example : examples) {
        CAPTURE(example.shoulder);
        CAPTURE(example.elbow);
        const auto pose = revolute::forward_kinematics(1, 1, example.shoulder, example.elbow);
        check_point(pose.base, {0, 0});
        check_point(pose.elbow, example.expected_elbow);
        check_point(pose.hand, example.expected_hand);
    }
}

TEST_CASE("Unequal fractional links and negative angles work") {
    const auto pose = revolute::forward_kinematics(1.5, 0.75, pi / 2, -pi / 2);
    check_point(pose.base, {0, 0});
    check_point(pose.elbow, {0, 1.5});
    check_point(pose.hand, {0.75, 1.5});
}

TEST_CASE("Rotating joints preserves both link lengths") {
    for (double shoulder : {-2.3, -pi / 2, 0.0, 0.7, pi, 3 * pi}) {
        for (double elbow : {-pi, -0.4, 0.0, 1.2, 4 * pi}) {
            CAPTURE(shoulder);
            CAPTURE(elbow);
            const auto pose = revolute::forward_kinematics(1.5, 0.75, shoulder, elbow);
            CHECK(std::abs(distance(pose.base, pose.elbow) - 1.5) <= tolerance);
            CHECK(std::abs(distance(pose.elbow, pose.hand) - 0.75) <= tolerance);
        }
    }
}

TEST_CASE("Whole turns preserve the pose") {
    const auto expected = revolute::forward_kinematics(1.5, 0.75, 0.3, -0.7);
    const auto actual = revolute::forward_kinematics(1.5, 0.75, 0.3 + 2 * pi, -0.7 - 4 * pi);
    check_point(actual.elbow, expected.elbow);
    check_point(actual.hand, expected.hand);
}

TEST_CASE("Invalid lengths are rejected for either link") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double length : {0.0, -1.0, infinity, -infinity, nan}) {
        CAPTURE(length);
        CHECK_THROWS_AS(revolute::forward_kinematics(length, 1, 0, 0), std::invalid_argument);
        CHECK_THROWS_AS(revolute::forward_kinematics(1, length, 0, 0), std::invalid_argument);
    }
}

TEST_CASE("Non-finite angles are rejected for either joint") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double angle : {infinity, -infinity, nan}) {
        CAPTURE(angle);
        CHECK_THROWS_AS(revolute::forward_kinematics(1, 1, angle, 0), std::invalid_argument);
        CHECK_THROWS_AS(revolute::forward_kinematics(1, 1, 0, angle), std::invalid_argument);
    }
}
