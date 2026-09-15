// Copyright 2026 mrl_nuc
// SPDX-License-Identifier: Apache-2.0

#include <Eigen/Geometry>

#include <cmath>

#include "chr_controller/chr_dynamics_library.hpp"
#include "gtest/gtest.h"

namespace {
using chr_controller::ChrKinematics;

TEST(ChrKinematics, HomePoseMatchesMjcf) {
  const auto pose = ChrKinematics::tcp_in_world(
    ChrKinematics::base_pose(
      Eigen::Vector3d(0.0, 0.0, 1.2), Eigen::Quaterniond::Identity()),
    Eigen::Vector3d::Zero());

  EXPECT_NEAR(pose.translation().x(), -3.66401025e-6, 1e-10);
  EXPECT_NEAR(pose.translation().y(), -9.18311789e-7, 1e-10);
  EXPECT_NEAR(pose.translation().z(), 0.27775, 1e-10);
}

TEST(ChrKinematics, JointLimitsMatchMjcf) {
  const auto clamped = ChrKinematics::clamp_joints(Eigen::Vector3d(-1.0, 2.0, -2.0));
  EXPECT_DOUBLE_EQ(clamped[0], 0.0);
  EXPECT_DOUBLE_EQ(clamped[1], 1.5708);
  EXPECT_DOUBLE_EQ(clamped[2], -1.5708);
}

TEST(ChrKinematics, PoseIkAlwaysReturnsLevelBase) {
  const Eigen::Vector3d base_position(0.0, 0.0, 1.2);
  const Eigen::Quaterniond target_base(
    Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d target_joints(0.45, 0.25, -0.35);
  const auto target_pose = ChrKinematics::tcp_in_world(
    ChrKinematics::base_pose(base_position, target_base), target_joints);
  const Eigen::Quaterniond tilted_seed =
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());

  chr_controller::IkOptions options;
  const auto result = ChrKinematics::solve_pose_dls(
    base_position, tilted_seed, target_pose, Eigen::Vector3d::Zero(), options);

  EXPECT_TRUE(result.converged);
  EXPECT_LT(result.position_residual_m, options.tolerance_m);
  EXPECT_LT(result.orientation_residual_rad, options.orientation_tolerance_rad);
  EXPECT_NEAR(result.base_orientation.x(), 0.0, 1e-12);
  EXPECT_NEAR(result.base_orientation.y(), 0.0, 1e-12);
}
}  // namespace
