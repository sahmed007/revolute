#include "kinematics.hpp"
#include <iostream>
#include <numbers>

int main() {
    double upper_arm = 1;
    double forearm = 1;

    double shoulder_angle_zero = 0;
    double elbow_angle_zero = std::numbers::pi;

    auto result =
        revolute::forward_kinematics(upper_arm, forearm, shoulder_angle_zero, elbow_angle_zero);

    std::cout << "Expected elbow: " << "(" << result.elbow.x << ", " << result.elbow.y << ") "
              << "and Expected hand: " << "(" << result.hand.x << ", " << result.hand.y << ")\n";

    return 0;
}
