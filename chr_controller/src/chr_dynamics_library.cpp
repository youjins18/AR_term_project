#include "chr_controller/chr_dynamics_library.hpp"

#include <Eigen/SVD>

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>

namespace chr_controller {
namespace {
constexpr double kFiniteDifferenceStep = 1e-6;
constexpr Eigen::Index kTaskDimension = 6;
constexpr Eigen::Index kCoordinateCount = 7;
constexpr Eigen::Index kBaseYawIndex = 3;
constexpr Eigen::Index kArmCoordinateOffset = 4;

using TaskVector = Eigen::Matrix<double, kTaskDimension, 1>;
using CoordinateVector = Eigen::Matrix<double, kCoordinateCount, 1>;
using TaskJacobian = Eigen::Matrix<double, kTaskDimension, kCoordinateCount>;

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

double quaternion_yaw(const Eigen::Quaterniond &input) {
  const auto quaternion = input.normalized();
  return std::atan2(
    2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
    1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));
}

Eigen::Quaterniond yaw_orientation(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}
}  // namespace

JointVector ChrKinematics::clamp_joints(const JointVector &joint_position) {
  const JointVector lower(0.0, -1.5708, -1.5708);
  const JointVector upper(1.5708, 1.5708, 1.5708);
  return joint_position.cwiseMax(lower).cwiseMin(upper);
}

Eigen::Quaterniond ChrKinematics::level_yaw_orientation(
    const Eigen::Quaterniond &orientation) {
  return yaw_orientation(quaternion_yaw(orientation));
}

Eigen::Isometry3d ChrKinematics::base_pose(
    const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
  return fixed_transform(position, orientation);
}

Eigen::Isometry3d ChrKinematics::tcp_in_world(
    const Eigen::Isometry3d &world_from_base, const JointVector &joint_position) {
  // Keep this chain synchronized with chr.xml and arm.xml.
  Eigen::Isometry3d transform = world_from_base;
  transform = transform * fixed_transform(
    Eigen::Vector3d(-0.10, 0.0, -0.13), Eigen::Quaterniond::Identity());
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

PoseIkResult ChrKinematics::solve_pose_dls(
    const Eigen::Vector3d &base_position,
    const Eigen::Quaterniond &seed_base_orientation,
    const Eigen::Isometry3d &target_pose,
    const JointVector &seed_joint_position,
    const IkOptions &options) {
  PoseIkResult result;
  result.base_position = base_position;
  double base_yaw = quaternion_yaw(seed_base_orientation);
  result.base_orientation = yaw_orientation(base_yaw);
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

    TaskJacobian jacobian;
    for (Eigen::Index column = 0; column < kCoordinateCount; ++column) {
      Eigen::Vector3d positive_position = result.base_position;
      Eigen::Vector3d negative_position = result.base_position;
      double positive_yaw = base_yaw;
      double negative_yaw = base_yaw;
      JointVector positive_joints = result.joint_position;
      JointVector negative_joints = result.joint_position;
      if (column < 3) {
        positive_position[column] += kFiniteDifferenceStep;
        negative_position[column] -= kFiniteDifferenceStep;
      } else if (column == kBaseYawIndex) {
        positive_yaw += kFiniteDifferenceStep;
        negative_yaw -= kFiniteDifferenceStep;
      } else {
        positive_joints[column - kArmCoordinateOffset] += kFiniteDifferenceStep;
        negative_joints[column - kArmCoordinateOffset] -= kFiniteDifferenceStep;
      }
      const auto positive_pose = tcp_in_world(
        base_pose(positive_position, yaw_orientation(positive_yaw)), positive_joints);
      const auto negative_pose = tcp_in_world(
        base_pose(negative_position, yaw_orientation(negative_yaw)), negative_joints);
      jacobian.block<3, 1>(0, column) =
        (positive_pose.translation() - negative_pose.translation()) /
        (2.0 * kFiniteDifferenceStep);
      jacobian.block<3, 1>(3, column) = orientation_weight * rotation_vector(
        positive_pose.rotation() * negative_pose.rotation().transpose()) /
        (2.0 * kFiniteDifferenceStep);
    }

    TaskVector error;
    error.head<3>() = position_error;
    error.tail<3>() = orientation_weight * orientation_error;
    CoordinateVector coordinate_scale;
    coordinate_scale <<
      options.base_translation_scale, options.base_translation_scale,
      options.base_translation_scale, options.base_yaw_scale, 1.0, 1.0, 1.0;
    const TaskJacobian weighted_jacobian =
      jacobian * coordinate_scale.asDiagonal();
    const Eigen::JacobiSVD<TaskJacobian> svd(weighted_jacobian);
    const auto singular_values = svd.singularValues();
    result.minimum_singular_value = singular_values[singular_values.size() - 1];
    result.condition_number = result.minimum_singular_value > 1e-12 ?
      singular_values[0] / result.minimum_singular_value :
      std::numeric_limits<double>::infinity();
    if (result.position_residual_m <= options.tolerance_m &&
        result.orientation_residual_rad <= options.orientation_tolerance_rad) {
      result.converged = true;
      return result;
    }
    CoordinateVector step = coordinate_scale.asDiagonal() *
      weighted_jacobian.transpose() *
      (weighted_jacobian * weighted_jacobian.transpose() +
       damping_squared * Eigen::Matrix<double, kTaskDimension, kTaskDimension>::Identity())
      .ldlt().solve(error);
    if (step.norm() > options.maximum_step_rad) {
      step *= options.maximum_step_rad / step.norm();
    }
    result.base_position += step.head<3>();
    base_yaw += step[kBaseYawIndex];
    result.base_orientation = yaw_orientation(base_yaw);
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
