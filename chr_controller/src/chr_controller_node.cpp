#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "chr_controller/chr_dynamics_library.hpp"
#include "chr_msgs/msg/chr_reference.hpp"
#include "chr_msgs/msg/chr_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class ChrController final : public rclcpp::Node {
 public:
  ChrController() : Node("chr_controller") {
    planner_mode_ = declare_parameter("planner_mode", "hold");
    command_hz_ = declare_parameter("command_hz", 100.0);
    const auto base_position = vector_parameter("default_base_position", {0.0, 0.0, 1.2});
    const auto base_rpy = vector_parameter("default_base_rpy", {0.0, 0.0, 0.0});
    const auto joints = vector_parameter("default_joint_position", {0.0, 0.0, 0.0});
    ik_options_.damping = declare_parameter("dls_damping", 0.03);
    ik_options_.tolerance_m = declare_parameter("dls_tolerance_m", 1e-4);
    ik_options_.orientation_tolerance_rad = declare_parameter(
      "dls_orientation_tolerance_rad", 0.01);
    ik_options_.orientation_weight_m_per_rad = declare_parameter(
      "dls_orientation_weight_m_per_rad", 0.25);
    ik_options_.base_translation_scale = declare_parameter(
      "dls_base_translation_scale", 0.35);
    ik_options_.base_rotation_scale = declare_parameter(
      "dls_base_rotation_scale", 0.5);
    if (ik_options_.orientation_tolerance_rad <= 0.0 ||
        ik_options_.orientation_weight_m_per_rad <= 0.0 ||
        ik_options_.base_translation_scale <= 0.0 ||
        ik_options_.base_rotation_scale <= 0.0) {
      throw std::runtime_error("DLS pose tolerances, weights and coordinate scales must be positive");
    }
    ik_options_.maximum_step_rad = declare_parameter("dls_maximum_step_rad", 0.12);
    ik_options_.maximum_iterations = static_cast<std::size_t>(
      declare_parameter("dls_maximum_iterations", 100));

    const std::set<std::string> supported{"hold", "dls_ik", "external"};
    if (!supported.count(planner_mode_)) {
      throw std::runtime_error(
        "unsupported planner_mode '" + planner_mode_ +
        "'; supported modes are hold, dls_ik and external. RL requires a policy adapter.");
    }

    desired_base_position_ = Eigen::Vector3d(base_position[0], base_position[1], base_position[2]);
    desired_base_orientation_ = rpy_to_quaternion(base_rpy[0], base_rpy[1], base_rpy[2]);
    desired_joint_position_ = chr_controller::ChrKinematics::clamp_joints(
      Eigen::Vector3d(joints[0], joints[1], joints[2]));

    state_subscriber_ = create_subscription<chr_msgs::msg::ChrState>(
      "/chr/state", 1, [this](const chr_msgs::msg::ChrState::SharedPtr message) {
        state_ = message;
      });
    tcp_target_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/chr/target/tcp_pose", 1,
      std::bind(&ChrController::receive_tcp_target, this, std::placeholders::_1));
    external_target_subscriber_ = create_subscription<chr_msgs::msg::ChrReference>(
      "/chr/target/whole_body", 1,
      std::bind(&ChrController::receive_external_target, this, std::placeholders::_1));
    reference_publisher_ = create_publisher<chr_msgs::msg::ChrReference>(
      "/chr/reference", 1);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / command_hz_),
      std::bind(&ChrController::publish_reference, this));
    RCLCPP_INFO(
      get_logger(), "planner_mode=%s; output=[base pose(6), J1, J2, J3]",
      planner_mode_.c_str());
  }

 private:
  std::array<double, 3> vector_parameter(
      const std::string &name, const std::array<double, 3> &defaults) {
    const auto values = declare_parameter<std::vector<double>>(
      name, {defaults[0], defaults[1], defaults[2]});
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain exactly three values");
    }
    return {values[0], values[1], values[2]};
  }

  static Eigen::Quaterniond rpy_to_quaternion(double roll, double pitch, double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  }

  void receive_tcp_target(const geometry_msgs::msg::PoseStamped::SharedPtr target) {
    if (planner_mode_ != "dls_ik") return;
    if (!target->header.frame_id.empty() && target->header.frame_id != "world") {
      RCLCPP_ERROR(get_logger(), "TCP target frame must be 'world'; target rejected");
      return;
    }
    Eigen::Quaterniond target_orientation(
      target->pose.orientation.w, target->pose.orientation.x,
      target->pose.orientation.y, target->pose.orientation.z);
    if (target_orientation.norm() < 1e-9) {
      RCLCPP_ERROR(get_logger(), "TCP target quaternion has zero norm; target rejected");
      return;
    }
    Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
    target_pose.translate(Eigen::Vector3d(
      target->pose.position.x, target->pose.position.y, target->pose.position.z));
    target_pose.rotate(target_orientation.normalized());
    const auto result = chr_controller::ChrKinematics::solve_pose_dls(
      desired_base_position_, desired_base_orientation_, target_pose,
      desired_joint_position_, ik_options_);
    desired_base_position_ = result.base_position;
    desired_base_orientation_ = result.base_orientation;
    desired_joint_position_ = result.joint_position;
    last_ik_residual_m_ = result.position_residual_m;
    last_ik_orientation_residual_rad_ = result.orientation_residual_rad;
    if (result.converged) {
      RCLCPP_INFO(
        get_logger(),
        "pose DLS-IK converged in %zu iterations; position=%.6f m, attitude=%.4f rad",
        result.iterations, result.position_residual_m, result.orientation_residual_rad);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "pose DLS-IK target is unreachable or singular; bounded best effort: position=%.4f m, attitude=%.3f rad",
        result.position_residual_m, result.orientation_residual_rad);
    }
  }

  void receive_external_target(const chr_msgs::msg::ChrReference::SharedPtr target) {
    if (planner_mode_ != "external") return;
    desired_base_position_ = Eigen::Vector3d(
      target->base_pose.position.x, target->base_pose.position.y, target->base_pose.position.z);
    desired_base_orientation_ = Eigen::Quaterniond(
      target->base_pose.orientation.w, target->base_pose.orientation.x,
      target->base_pose.orientation.y, target->base_pose.orientation.z).normalized();
    desired_base_linear_velocity_ = Eigen::Vector3d(
      target->base_twist.linear.x, target->base_twist.linear.y, target->base_twist.linear.z);
    desired_base_angular_velocity_ = Eigen::Vector3d(
      target->base_twist.angular.x, target->base_twist.angular.y, target->base_twist.angular.z);
    desired_joint_position_ = chr_controller::ChrKinematics::clamp_joints(Eigen::Vector3d(
      target->joint_position[0], target->joint_position[1], target->joint_position[2]));
    desired_joint_velocity_ = Eigen::Vector3d(
      target->joint_velocity[0], target->joint_velocity[1], target->joint_velocity[2]);
  }

  void publish_reference() {
    chr_msgs::msg::ChrReference reference;
    reference.header.stamp = now();
    reference.header.frame_id = "world";
    reference.base_pose.position.x = desired_base_position_.x();
    reference.base_pose.position.y = desired_base_position_.y();
    reference.base_pose.position.z = desired_base_position_.z();
    reference.base_pose.orientation.w = desired_base_orientation_.w();
    reference.base_pose.orientation.x = desired_base_orientation_.x();
    reference.base_pose.orientation.y = desired_base_orientation_.y();
    reference.base_pose.orientation.z = desired_base_orientation_.z();
    reference.base_twist.linear.x = desired_base_linear_velocity_.x();
    reference.base_twist.linear.y = desired_base_linear_velocity_.y();
    reference.base_twist.linear.z = desired_base_linear_velocity_.z();
    reference.base_twist.angular.x = desired_base_angular_velocity_.x();
    reference.base_twist.angular.y = desired_base_angular_velocity_.y();
    reference.base_twist.angular.z = desired_base_angular_velocity_.z();
    for (Eigen::Index index = 0; index < 3; ++index) {
      reference.joint_position[index] = desired_joint_position_[index];
      reference.joint_velocity[index] = desired_joint_velocity_[index];
    }
    reference.source = planner_mode_;
    reference_publisher_->publish(reference);
  }

  std::string planner_mode_;
  double command_hz_{100.0};
  double last_ik_residual_m_{0.0};
  double last_ik_orientation_residual_rad_{0.0};
  chr_controller::IkOptions ik_options_;
  Eigen::Vector3d desired_base_position_{0.0, 0.0, 1.2};
  Eigen::Quaterniond desired_base_orientation_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d desired_base_linear_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_base_angular_velocity_{Eigen::Vector3d::Zero()};
  chr_controller::JointVector desired_joint_position_{chr_controller::JointVector::Zero()};
  chr_controller::JointVector desired_joint_velocity_{chr_controller::JointVector::Zero()};
  chr_msgs::msg::ChrState::SharedPtr state_;
  rclcpp::Subscription<chr_msgs::msg::ChrState>::SharedPtr state_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_target_subscriber_;
  rclcpp::Subscription<chr_msgs::msg::ChrReference>::SharedPtr external_target_subscriber_;
  rclcpp::Publisher<chr_msgs::msg::ChrReference>::SharedPtr reference_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChrController>());
  rclcpp::shutdown();
  return 0;
}
