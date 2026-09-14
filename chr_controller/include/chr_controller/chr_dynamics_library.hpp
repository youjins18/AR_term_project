#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>

namespace chr_controller {

using JointVector = Eigen::Vector3d;

struct IkOptions {
  double damping{0.03};
  double tolerance_m{1e-4};
  double maximum_step_rad{0.12};
  std::size_t maximum_iterations{100};
};

struct IkResult {
  JointVector joint_position{JointVector::Zero()};
  double residual_m{0.0};
  std::size_t iterations{0};
  bool converged{false};
};

class ChrKinematics {
 public:
  static JointVector clamp_joints(const JointVector &joint_position);
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
};

}  // namespace chr_controller
