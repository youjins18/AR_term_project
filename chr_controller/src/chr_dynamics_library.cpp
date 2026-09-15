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

Eigen::Vector3d rotation_vector(const Eigen::Matrix3d &rotation) {
  const Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() < 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.axis() * angle_axis.angle();
}

Eigen::Quaterniond rotation_increment(const Eigen::Vector3d &rotation_vector_value) {
  const double angle = rotation_vector_value.norm();
  if (angle < 1e-12) return Eigen::Quaterniond::Identity();
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector_value / angle));
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
    Eigen::Vector3d(0.0, 0.0, -0.13), Eigen::Quaterniond::Identity());
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

PoseIkResult ChrKinematics::solve_pose_dls(
    const Eigen::Vector3d &base_position,
    const Eigen::Quaterniond &seed_base_orientation,
    const Eigen::Isometry3d &target_pose,
    const JointVector &seed_joint_position,
    const IkOptions &options) {
  PoseIkResult result;
  result.base_position = base_position;
  result.base_orientation = seed_base_orientation.normalized();
  result.joint_position = clamp_joints(seed_joint_position);
  const double damping_squared = options.damping * options.damping;
  const double orientation_weight = std::max(options.orientation_weight_m_per_rad, 1e-6);

  for (std::size_t iteration = 0; iteration < options.maximum_iterations; ++iteration) {
    const auto world_from_base = base_pose(result.base_position, result.base_orientation);
    const auto current_pose = tcp_in_world(world_from_base, result.joint_position);
    const Eigen::Vector3d position_error = target_pose.translation() - current_pose.translation();
    const Eigen::Vector3d orientation_error = rotation_vector(
      target_pose.rotation() * current_pose.rotation().transpose());
    result.position_residual_m = position_error.norm();
    result.orientation_residual_rad = orientation_error.norm();
    result.iterations = iteration;
    if (result.position_residual_m <= options.tolerance_m &&
        result.orientation_residual_rad <= options.orientation_tolerance_rad) {
      result.converged = true;
      return result;
    }

    constexpr double epsilon = 1e-6;
    Eigen::Matrix<double, 6, 9> jacobian;
    for (Eigen::Index column = 0; column < 9; ++column) {
      Eigen::Vector3d positive_position = result.base_position;
      Eigen::Vector3d negative_position = result.base_position;
      Eigen::Quaterniond positive_orientation = result.base_orientation;
      Eigen::Quaterniond negative_orientation = result.base_orientation;
      JointVector positive_joints = result.joint_position;
      JointVector negative_joints = result.joint_position;
      if (column < 3) {
        positive_position[column] += epsilon;
        negative_position[column] -= epsilon;
      } else if (column < 6) {
        Eigen::Vector3d perturbation = Eigen::Vector3d::Zero();
        perturbation[column - 3] = epsilon;
        positive_orientation = (rotation_increment(perturbation) * positive_orientation).normalized();
        negative_orientation = (rotation_increment(-perturbation) * negative_orientation).normalized();
      } else {
        positive_joints[column - 6] += epsilon;
        negative_joints[column - 6] -= epsilon;
      }
      const auto positive_pose = tcp_in_world(
        base_pose(positive_position, positive_orientation), positive_joints);
      const auto negative_pose = tcp_in_world(
        base_pose(negative_position, negative_orientation), negative_joints);
      jacobian.block<3, 1>(0, column) =
        (positive_pose.translation() - negative_pose.translation()) / (2.0 * epsilon);
      jacobian.block<3, 1>(3, column) = orientation_weight * rotation_vector(
        positive_pose.rotation() * negative_pose.rotation().transpose()) / (2.0 * epsilon);
    }

    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = position_error;
    error.tail<3>() = orientation_weight * orientation_error;
    Eigen::Matrix<double, 9, 1> coordinate_scale;
    coordinate_scale <<
      options.base_translation_scale, options.base_translation_scale,
      options.base_translation_scale, options.base_rotation_scale,
      options.base_rotation_scale, options.base_rotation_scale, 1.0, 1.0, 1.0;
    const Eigen::Matrix<double, 6, 9> weighted_jacobian =
      jacobian * coordinate_scale.asDiagonal();
    Eigen::Matrix<double, 9, 1> step = coordinate_scale.asDiagonal() *
      weighted_jacobian.transpose() *
      (weighted_jacobian * weighted_jacobian.transpose() +
       damping_squared * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(error);
    if (step.norm() > options.maximum_step_rad) {
      step *= options.maximum_step_rad / step.norm();
    }
    result.base_position += step.head<3>();
    result.base_orientation = (
      rotation_increment(step.segment<3>(3)) * result.base_orientation).normalized();
    result.joint_position = clamp_joints(result.joint_position + step.tail<3>());
  }

  const auto current_pose = tcp_in_world(
    base_pose(result.base_position, result.base_orientation), result.joint_position);
  result.position_residual_m = (target_pose.translation() - current_pose.translation()).norm();
  result.orientation_residual_rad = rotation_vector(
    target_pose.rotation() * current_pose.rotation().transpose()).norm();
  return result;
}

}  // namespace chr_controller
