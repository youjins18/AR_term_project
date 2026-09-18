// Copyright 2026 mrl_nuc
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>

namespace chr_controller {

using JointVector = Eigen::Vector3d;

struct IkOptions {
  // Damping and task-space weights are deliberately explicit because position
  // is measured in metres while attitude error is measured in radians.
  double damping{0.03};
  double tolerance_m{1e-4};
  double orientation_tolerance_rad{0.01};
  double orientation_weight_m_per_rad{0.25};
  double base_translation_scale{0.35};
  double base_yaw_scale{0.5};
  double maximum_step_rad{0.12};
  std::size_t maximum_iterations{100};
};

struct IkResult {
  JointVector joint_position{JointVector::Zero()};
  double residual_m{0.0};
  std::size_t iterations{0};
  bool converged{false};
};

struct PoseIkResult {
  Eigen::Vector3d base_position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond base_orientation{Eigen::Quaterniond::Identity()};
  JointVector joint_position{JointVector::Zero()};
  double position_residual_m{0.0};
  double orientation_residual_rad{0.0};
  double minimum_singular_value{0.0};
  double condition_number{0.0};
  std::size_t iterations{0};
  bool converged{false};
};

class ChrKinematics {
 public:
  /// Clamp J1--J3 to the mechanical limits declared in arm.xml.
  static JointVector clamp_joints(const JointVector &joint_position);
  /// Remove base roll and pitch while preserving quaternion yaw.
  static Eigen::Quaterniond level_yaw_orientation(const Eigen::Quaterniond &orientation);
  /// Construct a world-to-base transform from a normalized base pose.
  static Eigen::Isometry3d base_pose(
    const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);
  static Eigen::Isometry3d tcp_in_world(
    const Eigen::Isometry3d &world_from_base, const JointVector &joint_position);
  static Eigen::Matrix3d position_jacobian(
    const Eigen::Isometry3d &world_from_base, const JointVector &joint_position);
  static IkResult solve_position_dls(
    const Eigen::Isometry3d &world_from_base,
    const Eigen::Vector3d &target_position,
    const JointVector &seed,
    const IkOptions &options);
  /**
   * Solve a six-dimensional TCP pose task using seven allowed coordinates:
   * base XYZ, base yaw, and J1--J3. Base roll and pitch are excluded from the
   * Jacobian, so every returned Palletrone attitude is level by construction.
   */
  static PoseIkResult solve_pose_dls(
    const Eigen::Vector3d &base_position,
    const Eigen::Quaterniond &seed_base_orientation,
    const Eigen::Isometry3d &target_pose,
    const JointVector &seed_joint_position,
    const IkOptions &options);
};

}  // namespace chr_controller
