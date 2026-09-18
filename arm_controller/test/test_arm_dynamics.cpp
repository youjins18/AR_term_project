#include "arm_controller/arm_dynamics_library.hpp"
#include "gtest/gtest.h"

TEST(ArmDynamics, JointPositionIsClampedToMjcfLimits) {
  const auto result = arm_controller::clamp_joint_position({-1.0, 2.0, -2.0});
  EXPECT_DOUBLE_EQ(result[0], 0.0);
  EXPECT_DOUBLE_EQ(result[1], 1.5708);
  EXPECT_DOUBLE_EQ(result[2], -1.5708);
}

TEST(ArmDynamics, GravityFeedforwardHasExplicitEnableSwitch) {
  const arm_controller::JointVector gravity{1.0, -2.0, 3.0};
  EXPECT_EQ(arm_controller::select_gravity_feedforward(gravity, true), gravity);
  EXPECT_EQ(
    arm_controller::select_gravity_feedforward(gravity, false),
    arm_controller::JointVector{});
}
