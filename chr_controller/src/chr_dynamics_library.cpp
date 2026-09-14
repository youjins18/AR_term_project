#include "chr_controller/chr_dynamics_library.hpp"

#include <algorithm>
#include <cmath>

namespace chr_controller {
namespace {
Eigen::Isometry3d fixed_transform(
    const Eigen::Vector3d &translation, const Eigen::Quaterniond &quaternion) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translate(translation);
  transform.rotate(quaternion.normalized());
  return transform;
}

Eigen::Isometry3d revolute_z(double angle) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
  return transform;
}
}  // namespace

JointVector ChrKinematics::clamp_joints(const JointVector &joint_position) {
  const JointVector lower(0.0, -1.5708, -1.5708);
  const JointVector upper(1.5708, 1.5708, 1.5708);
  return joint_position.cwiseMax(lower).cwiseMin(upper);
}

Eigen::Isometry3d ChrKinematics::base_pose(
    const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
  return fixed_transform(position, orientation);
}

Eigen::Isometry3d ChrKinematics::tcp_in_world(
    const Eigen::Isometry3d &world_from_base, const JointVector &joint_position) {
  // This is the exact nominal MJCF body-frame chain in chr_description/arm.xml.
  Eigen::Isometry3d transform = world_from_base;
  transform = transform * fixed_transform(
    Eigen::Vector3d(0.0, 0.0, -0.065), Eigen::Quaterniond::Identity());
  transform = transform * fixed_transform(
    Eigen::Vector3d::Zero(), Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0));
  transform = transform * fixed_transform(
    Eigen::Vector3d(0.0, 0.0, 0.04475),
    Eigen::Quaterniond(0.4999981634, -0.5, -0.5, -0.5000018366));
  transform = transform * revolute_z(joint_position[0]);
  transform = transform * fixed_transform(
    Eigen::Vector3d(0.333, 0.0, 0.0), Eigen::Quaterniond::Identity());
  transform = transform * revolute_z(joint_position[1]);
  transform = transform * fixed_transform(
    Eigen::Vector3d(0.0835, 0.0, 0.0),
    Eigen::Quaterniond(0.7071054825, -0.7071080799, 0.0, 0.0));
  transform = transform * revolute_z(joint_position[2]);
  transform = transform * fixed_transform(
    Eigen::Vector3d(0.081, 0.0, 0.0),
    Eigen::Quaterniond(0.4999981634, -0.5, 0.5000018366, -0.5));
  transform = transform * fixed_transform(
    Eigen::Vector3d(0.0, 0.0, 0.25), Eigen::Quaterniond::Identity());
  return transform;
}

Eigen::Matrix3d ChrKinematics::position_jacobian(
    const Eigen::Isometry3d &world_from_base, const JointVector &joint_position) {
  constexpr double epsilon = 1e-6;
  Eigen::Matrix3d jacobian;
  for (Eigen::Index column = 0; column < 3; ++column) {
    JointVector positive = joint_position;
    JointVector negative = joint_position;
    positive[column] += epsilon;
    negative[column] -= epsilon;
    jacobian.col(column) =
      (tcp_in_world(world_from_base, positive).translation() -
       tcp_in_world(world_from_base, negative).translation()) / (2.0 * epsilon);
  }
  return jacobian;
}

IkResult ChrKinematics::solve_position_dls(
    const Eigen::Isometry3d &world_from_base,
    const Eigen::Vector3d &target_position,
    const JointVector &seed,
    const IkOptions &options) {
  IkResult result;
  result.joint_position = clamp_joints(seed);
  const double damping_squared = options.damping * options.damping;
  for (std::size_t iteration = 0; iteration < options.maximum_iterations; ++iteration) {
    const Eigen::Vector3d error = target_position -
      tcp_in_world(world_from_base, result.joint_position).translation();
    result.residual_m = error.norm();
    result.iterations = iteration;
    if (result.residual_m <= options.tolerance_m) {
      result.converged = true;
      return result;
    }
    const Eigen::Matrix3d jacobian = position_jacobian(world_from_base, result.joint_position);
    JointVector step = jacobian.transpose() *
      (jacobian * jacobian.transpose() + damping_squared * Eigen::Matrix3d::Identity())
      .ldlt().solve(error);
    if (step.norm() > options.maximum_step_rad) {
      step *= options.maximum_step_rad / step.norm();
    }
    result.joint_position = clamp_joints(result.joint_position + step);
  }
  result.residual_m = (
    target_position - tcp_in_world(world_from_base, result.joint_position).translation()).norm();
  return result;
}

}  // namespace chr_controller
