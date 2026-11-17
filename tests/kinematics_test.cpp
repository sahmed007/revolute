#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "revolute/core/kinematics.hpp"

#include <cmath>
#include <doctest/doctest.h>
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

TEST_CASE("Unequal links reach an annulus regardless of link order") {
    struct Example {
        revolute::Point2D target;
        bool reachable;
    };

    const Example examples[] = {
        {{0, 0}, false},
        {{1, 1}, false},
        {{2, 0}, true},
        {{0, 3}, true},
        {{-4, 0}, true},
        {{3, 3}, false},
        {{-2, -2}, true},
        {{2, -2}, true},
        {{2 - 1e-6, 0}, false},
        {{2 + 1e-6, 0}, true},
        {{4 - 1e-6, 0}, true},
        {{4 + 1e-6, 0}, false},
    };

    for (const auto& example : examples) {
        CAPTURE(example.target.x);
        CAPTURE(example.target.y);
        CHECK(revolute::reachable(3, 1, example.target) == example.reachable);
        CHECK(revolute::reachable(1, 3, example.target) == example.reachable);
    }
}

TEST_CASE("Equal links reach the origin and the disk out to full extension") {
    CHECK(revolute::reachable(1, 1, {0, 0}));
    CHECK(revolute::reachable(1, 1, {0.5, 0}));
    CHECK(revolute::reachable(1, 1, {1, 1}));
    CHECK(revolute::reachable(1, 1, {0, -2}));
    CHECK_FALSE(revolute::reachable(1, 1, {0, 2 + 1e-6}));
}

TEST_CASE("Reachability rejects invalid lengths for either link") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double length : {0.0, -1.0, infinity, -infinity, nan}) {
        CAPTURE(length);
        CHECK_THROWS_AS(revolute::reachable(length, 1, {1, 0}), std::invalid_argument);
        CHECK_THROWS_AS(revolute::reachable(1, length, {1, 0}), std::invalid_argument);
    }
}

TEST_CASE("Reachability rejects non-finite target coordinates") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double coordinate : {infinity, -infinity, nan}) {
        CAPTURE(coordinate);
        CHECK_THROWS_AS(revolute::reachable(1, 1, {coordinate, 0}), std::invalid_argument);
        CHECK_THROWS_AS(revolute::reachable(1, 1, {0, coordinate}), std::invalid_argument);
    }
}

namespace {

void check_ik_candidates(
    double upper_length,
    double forearm_length,
    revolute::Point2D target,
    const revolute::IKResult& result
) {
    for (const auto& angles : result.solutions) {
        CAPTURE(angles.shoulder);
        CAPTURE(angles.elbow);
        REQUIRE(std::isfinite(angles.shoulder));
        REQUIRE(std::isfinite(angles.elbow));
        CHECK(angles.shoulder >= -pi);
        CHECK(angles.shoulder <= pi);
        CHECK(angles.elbow >= -pi);
        CHECK(angles.elbow <= pi);
        const auto pose = revolute::forward_kinematics(
            upper_length,
            forearm_length,
            angles.shoulder,
            angles.elbow
        );
        check_point(pose.hand, target);
    }
    if (result.solutions.size() == 2) {
        const auto first = result.solutions[0];
        const auto second = result.solutions[1];
        const auto first_pose =
            revolute::forward_kinematics(upper_length, forearm_length, first.shoulder, first.elbow);
        const auto second_pose = revolute::forward_kinematics(
            upper_length,
            forearm_length,
            second.shoulder,
            second.elbow
        );
        // Distinct elbow positions prove these are two physical configurations.
        CHECK(distance(first_pose.elbow, second_pose.elbow) > tolerance);
    }
}

} // namespace

TEST_CASE("IK returns distinct solutions in every quadrant with either link order") {
    for (double upper_length : {1.0, 1.5}) {
        double forearm_length = 2.5 - upper_length;
        for (double x : {-1.0, 1.0}) {
            for (double y : {-1.0, 1.0}) {
                CAPTURE(upper_length);
                CAPTURE(x);
                CAPTURE(y);
                revolute::Point2D target{x, y};
                auto result = revolute::inverse_kinematics(upper_length, forearm_length, target);
                REQUIRE(result.status == revolute::IKStatus::success);
                REQUIRE(result.solutions.size() == 2);
                check_ik_candidates(upper_length, forearm_length, target, result);
            }
        }
    }
}

TEST_CASE("Unit-link IK agrees with hand-calculated angles") {
    auto result = revolute::inverse_kinematics(1, 1, {1, 1});
    REQUIRE(result.status == revolute::IKStatus::success);
    REQUIRE(result.solutions.size() == 2);
    CHECK(result.solutions[0].shoulder == doctest::Approx(0).epsilon(1e-12));
    CHECK(result.solutions[0].elbow == doctest::Approx(pi / 2));
    CHECK(result.solutions[1].shoulder == doctest::Approx(pi / 2));
    CHECK(result.solutions[1].elbow == doctest::Approx(-pi / 2));
    check_ik_candidates(1, 1, {1, 1}, result);
}

TEST_CASE("IK handles inner and outer boundaries and their neighborhoods") {
    struct Example {
        double radius;
        unsigned count;
    };

    const Example examples[] =
        {{0, 0}, {2 - 1e-6, 0}, {2, 1}, {2 + 1e-6, 2}, {4 - 1e-6, 2}, {4, 1}, {4 + 1e-6, 0}};
    for (double upper_length : {1.0, 3.0}) {
        double forearm_length = 4 - upper_length;
        for (auto example : examples) {
            for (auto direction :
                 {revolute::Point2D{1, 0},
                  revolute::Point2D{-1, 0},
                  revolute::Point2D{0, 1},
                  revolute::Point2D{0, -1}}) {
                revolute::Point2D target{
                    direction.x * example.radius,
                    direction.y * example.radius
                };
                CAPTURE(upper_length);
                CAPTURE(target.x);
                CAPTURE(target.y);
                auto result = revolute::inverse_kinematics(upper_length, forearm_length, target);
                CHECK(
                    result.status
                    == (example.count == 0 ? revolute::IKStatus::unreachable
                                           : revolute::IKStatus::success)
                );
                REQUIRE(result.solutions.size() == example.count);
                check_ik_candidates(upper_length, forearm_length, target, result);
            }
        }
    }
}

TEST_CASE("Equal links choose a folded origin representative and solve nearby targets") {
    auto origin = revolute::inverse_kinematics(1, 1, {0, 0});
    REQUIRE(origin.status == revolute::IKStatus::success);
    REQUIRE(origin.solutions.size() == 1);
    CHECK(origin.solutions[0].shoulder == 0);
    CHECK(origin.solutions[0].elbow == pi);
    check_ik_candidates(1, 1, {0, 0}, origin);

    for (auto target :
         {revolute::Point2D{1e-3, 0},
          revolute::Point2D{0, -1e-3},
          revolute::Point2D{2 - 1e-6, 0},
          revolute::Point2D{2, 0}}) {
        auto result = revolute::inverse_kinematics(1, 1, target);
        REQUIRE(result.status == revolute::IKStatus::success);
        REQUIRE(result.solutions.size() == (target.x == 2 ? 1 : 2));
        check_ik_candidates(1, 1, target, result);
    }
    auto outside = revolute::inverse_kinematics(1, 1, {2 + 1e-6, 0});
    CHECK(outside.status == revolute::IKStatus::unreachable);
    CHECK(outside.solutions.empty());
}

TEST_CASE("IK rejects invalid lengths and non-finite targets") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double length : {0.0, -1.0, infinity, -infinity, nan}) {
        CAPTURE(length);
        CHECK_THROWS_AS(revolute::inverse_kinematics(length, 1, {1, 0}), std::invalid_argument);
        CHECK_THROWS_AS(revolute::inverse_kinematics(1, length, {1, 0}), std::invalid_argument);
    }
    for (double coordinate : {infinity, -infinity, nan}) {
        CAPTURE(coordinate);
        CHECK_THROWS_AS(revolute::inverse_kinematics(1, 1, {coordinate, 0}), std::invalid_argument);
        CHECK_THROWS_AS(revolute::inverse_kinematics(1, 1, {0, coordinate}), std::invalid_argument);
    }
}

TEST_CASE("Joint limits include endpoints and reject each out-of-range joint") {
    revolute::JointLimits limits{{-pi / 2, 0}, {pi / 2, pi}};
    CHECK(revolute::within_joint_limits({0, pi / 2}, limits));
    CHECK(revolute::within_joint_limits(limits.minimum, limits));
    CHECK(revolute::within_joint_limits(limits.maximum, limits));
    CHECK_FALSE(revolute::within_joint_limits({-pi / 2 - 1e-6, pi / 2}, limits));
    CHECK_FALSE(revolute::within_joint_limits({pi / 2 + 1e-6, pi / 2}, limits));
    CHECK_FALSE(revolute::within_joint_limits({0, -1e-6}, limits));
    CHECK_FALSE(revolute::within_joint_limits({0, pi + 1e-6}, limits));
}

TEST_CASE("Geometric reachability is separate from admissibility under joint limits") {
    auto result = revolute::inverse_kinematics(1, 1, {1, 1});
    REQUIRE(result.status == revolute::IKStatus::success);
    REQUIRE(result.solutions.size() == 2);
    revolute::JointLimits positive_elbow{{-pi, 0}, {pi, pi}};
    revolute::JointLimits restricted_elbow{{-pi, -pi / 4}, {pi, pi / 4}};
    unsigned admissible = 0;
    for (auto angles : result.solutions) {
        if (revolute::within_joint_limits(angles, positive_elbow)) {
            ++admissible;
        }
        CHECK_FALSE(revolute::within_joint_limits(angles, restricted_elbow));
    }
    CHECK(admissible == 1);
    CHECK(result.status == revolute::IKStatus::success);
}

TEST_CASE("Jacobian matches the hand-calculated straight-right pose") {
    const auto matrix = revolute::jacobian(1, 1, 0, 0);
    CHECK(std::abs(matrix(0, 0)) <= tolerance);
    CHECK(std::abs(matrix(0, 1)) <= tolerance);
    CHECK(std::abs(matrix(1, 0) - 2) <= tolerance);
    CHECK(std::abs(matrix(1, 1) - 1) <= tolerance);
}

TEST_CASE("Jacobian columns agree with central differences of forward kinematics") {
    struct Example {
        double upper_length;
        double forearm_length;
        double shoulder;
        double elbow;
    };

    const Example examples[] = {
        {1, 1, 0, pi / 2},
        {1, 1, -pi / 2, -pi / 3},
        {1.5, 0.75, 0.7, -1.1},
        {0.75, 1.5, -2.3, 0.4},
        {3, 1, 2.6, 2.2},
    };
    // Absolute derivative tolerance in meters/radian. For these lengths and
    // h <= 1e-4, central-difference truncation is O(h^2), about 7e-9 or less.
    // 1e-7 also allows floating-point cancellation at the smallest step.
    constexpr double derivative_tolerance = 1e-7;
    for (const auto& example : examples) {
        CAPTURE(example.upper_length);
        CAPTURE(example.forearm_length);
        CAPTURE(example.shoulder);
        CAPTURE(example.elbow);
        const auto analytical = revolute::jacobian(
            example.upper_length,
            example.forearm_length,
            example.shoulder,
            example.elbow
        );
        REQUIRE(analytical.allFinite());
        for (double step : {1e-4, 1e-5, 1e-6}) {
            CAPTURE(step);
            for (int column = 0; column < 2; ++column) {
                CAPTURE(column);
                // Perturb only the joint belonging to this column.
                double shoulder_step = column == 0 ? step : 0;
                double elbow_step = column == 1 ? step : 0;
                const auto plus = revolute::forward_kinematics(
                    example.upper_length,
                    example.forearm_length,
                    example.shoulder + shoulder_step,
                    example.elbow + elbow_step
                );
                const auto minus = revolute::forward_kinematics(
                    example.upper_length,
                    example.forearm_length,
                    example.shoulder - shoulder_step,
                    example.elbow - elbow_step
                );
                Eigen::Vector2d numerical{
                    (plus.hand.x - minus.hand.x) / (2 * step),
                    (plus.hand.y - minus.hand.y) / (2 * step)
                };
                REQUIRE(numerical.allFinite());
                for (int row = 0; row < 2; ++row) {
                    CAPTURE(row);
                    CAPTURE(numerical(row));
                    CAPTURE(analytical(row, column));
                    CHECK(
                        std::abs(numerical(row) - analytical(row, column)) <= derivative_tolerance
                    );
                }
            }
        }
    }
}

TEST_CASE("Jacobian rejects invalid geometry and non-finite angles") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double length : {0.0, -1.0, infinity, -infinity, nan}) {
        CAPTURE(length);
        CHECK_THROWS_AS(revolute::jacobian(length, 1, 0, 0), std::invalid_argument);
        CHECK_THROWS_AS(revolute::jacobian(1, length, 0, 0), std::invalid_argument);
    }
    for (double angle : {infinity, -infinity, nan}) {
        CAPTURE(angle);
        CHECK_THROWS_AS(revolute::jacobian(1, 1, angle, 0), std::invalid_argument);
        CHECK_THROWS_AS(revolute::jacobian(1, 1, 0, angle), std::invalid_argument);
    }
}

TEST_CASE("Numerical IK converges and reports the final residual") {
    const revolute::Point2D target{1, 1};
    auto result = revolute::num_inverse_kinematics(1, 1, target, {0.1, 1.4}, 1e-8, 100, 0.1);
    CHECK(result.status == revolute::NumericalIKStatus::converged);
    CHECK(result.iterations > 0);
    CHECK(result.iterations <= 100);
    const auto pose =
        revolute::forward_kinematics(1, 1, result.angles.shoulder, result.angles.elbow);
    CHECK(distance(pose.hand, target) <= 1e-8);
    CHECK(result.position_error == doctest::Approx(distance(pose.hand, target)));
}

TEST_CASE("Numerical IK respects zero budgets and bounds each update") {
    auto solved = revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, pi / 2}, 1e-8, 0, 0.1);
    CHECK(solved.status == revolute::NumericalIKStatus::converged);
    CHECK(solved.iterations == 0);
    auto exhausted = revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 1}, 1e-8, 0, 0.1);
    CHECK(exhausted.status == revolute::NumericalIKStatus::iteration_limit);
    CHECK(exhausted.iterations == 0);
    auto step = revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 1}, 1e-8, 1, 0.01);
    CHECK(step.status == revolute::NumericalIKStatus::iteration_limit);
    CHECK(step.iterations == 1);
    CHECK(std::hypot(step.angles.shoulder, step.angles.elbow - 1) == doctest::Approx(0.01));
    auto last = revolute::num_inverse_kinematics(
        1,
        1,
        {1, 1},
        {0, 1},
        step.position_error + 1e-10,
        1,
        0.01
    );
    CHECK(last.status == revolute::NumericalIKStatus::converged);
    CHECK(last.iterations == 1);
}

TEST_CASE("Numerical IK rejects joint limit crossings without applying the update") {
    const revolute::JointLimits limits{{0, 1}, {0, 1}};
    auto result = revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 1}, 1e-8, 100, 0.1, limits);
    CHECK(result.status == revolute::NumericalIKStatus::joint_limit);
    CHECK(result.iterations == 0);
    CHECK(result.angles.shoulder == 0);
    CHECK(result.angles.elbow == 1);
    CHECK_THROWS_AS(
        revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 0}, 1e-8, 100, 0.1, limits),
        std::invalid_argument
    );
    const revolute::JointLimits reversed{{1, 1}, {0, 0}};
    CHECK_THROWS_AS(
        revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 0}, 1e-8, 100, 0.1, reversed),
        std::invalid_argument
    );
}

TEST_CASE("Numerical IK validates settings and reports numerical failure") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double invalid : {0.0, -1.0, nan, inf}) {
        CHECK_THROWS_AS(
            revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 1}, invalid, 10, 0.1),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 1}, 1e-8, 10, invalid),
            std::invalid_argument
        );
    }
    CHECK_THROWS_AS(
        revolute::num_inverse_kinematics(1, 1, {nan, 1}, {0, 1}, 1e-8, 10, 0.1),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        revolute::num_inverse_kinematics(1, 1, {1, inf}, {0, 1}, 1e-8, 10, 0.1),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 1}, 1e-8, -1, 0.1),
        std::invalid_argument
    );
    const double huge = std::numeric_limits<double>::max();
    auto result = revolute::num_inverse_kinematics(huge, huge, {0, 0}, {0, 0}, 1e-8, 10, 0.1);
    CHECK(result.status == revolute::NumericalIKStatus::numerical_failure);
    CHECK(result.iterations == 0);
    CHECK(std::isinf(result.position_error));
}

TEST_CASE("Damped IK converges across damping values and initial guesses") {
    for (double damping : {0.01, 0.1, 1.0}) {
        for (revolute::JointAngles initial : {revolute::JointAngles{0.1, 1.4}, {0, 0}, {0, 1e-6}}) {
            CAPTURE(damping);
            CAPTURE(initial.shoulder);
            CAPTURE(initial.elbow);
            auto result = revolute::num_inverse_kinematics(
                1,
                1,
                {1, 1},
                initial,
                1e-8,
                500,
                0.1,
                std::nullopt,
                damping
            );
            CHECK(result.status == revolute::NumericalIKStatus::converged);
            CHECK(result.iterations <= 500);
            const auto pose =
                revolute::forward_kinematics(1, 1, result.angles.shoulder, result.angles.elbow);
            CHECK(distance(pose.hand, {1, 1}) <= 1e-8);
        }
    }
}

TEST_CASE("Stronger damping reduces an uncapped correction") {
    auto weak =
        revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 1}, 1e-8, 1, 10, std::nullopt, 0.01);
    auto strong =
        revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 1}, 1e-8, 1, 10, std::nullopt, 1.0);
    CHECK(weak.iterations == 1);
    CHECK(strong.iterations == 1);
    CHECK(
        std::hypot(strong.angles.shoulder, strong.angles.elbow - 1)
        < std::hypot(weak.angles.shoulder, weak.angles.elbow - 1)
    );
}

TEST_CASE("Damped IK terminates on unreachable targets and singular stagnation") {
    for (double damping : {0.01, 0.1, 1.0}) {
        auto unreachable = revolute::num_inverse_kinematics(
            1,
            1,
            {3, 0},
            {0.1, 1},
            1e-8,
            30,
            0.1,
            std::nullopt,
            damping
        );
        CHECK(unreachable.status == revolute::NumericalIKStatus::iteration_limit);
        CHECK(unreachable.iterations == 30);
        CHECK(std::isfinite(unreachable.position_error));
        CHECK(unreachable.position_error >= 1.0 - 1e-12);
        // A radial error at an exactly straight pose yields no local joint correction.
        auto stagnant = revolute::num_inverse_kinematics(
            1,
            1,
            {1, 0},
            {0, 0},
            1e-8,
            30,
            0.1,
            std::nullopt,
            damping
        );
        CHECK(stagnant.status == revolute::NumericalIKStatus::iteration_limit);
        CHECK(stagnant.iterations == 30);
        CHECK(stagnant.angles.shoulder == 0);
        CHECK(stagnant.angles.elbow == 0);
        CHECK(stagnant.position_error == 1);
    }
}

TEST_CASE("Damped IK rejects invalid damping and handles matrix overflow") {
    for (double damping :
         {0.0,
          -1.0,
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity()}) {
        CHECK_THROWS_AS(
            revolute::
                num_inverse_kinematics(1, 1, {1, 1}, {0, 1}, 1e-8, 10, 0.1, std::nullopt, damping),
            std::invalid_argument
        );
    }
    // FK and the residual are finite, but forming J-transpose times J overflows.
    auto overflow = revolute::num_inverse_kinematics(1e200, 1e200, {0, 0}, {0, 1}, 1e-8, 10, 0.1);
    CHECK(overflow.status == revolute::NumericalIKStatus::numerical_failure);
    CHECK(overflow.iterations == 0);
    CHECK(std::isfinite(overflow.position_error));
    auto huge_damping =
        revolute::num_inverse_kinematics(1, 1, {1, 1}, {0, 1}, 1e-8, 10, 0.1, std::nullopt, 1e200);
    CHECK(huge_damping.status == revolute::NumericalIKStatus::numerical_failure);
    CHECK(huge_damping.iterations == 0);
}
