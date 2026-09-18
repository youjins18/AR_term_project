// Copyright 2026 mrl_nuc
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "arm_controller/arm_dynamics_library.hpp"
#include "chr_msgs/msg/arm_command.hpp"
#include "chr_msgs/msg/chr_reference.hpp"
#include "chr_msgs/msg/chr_state.hpp"
#include "rclcpp/rclcpp.hpp"

class ArmController final : public rclcpp::Node {
 public:
  ArmController() : Node("arm_controller") {
    control_hz_ = declare_parameter("control_hz", 250.0);
    reference_timeout_s_ = declare_parameter("reference_timeout_s", 0.5);
    gravity_compensation_ = declare_parameter("gravity_compensation", false);
    kp_ = vector_parameter("joint_kp", {20.0, 20.0, 16.0});
    kd_ = vector_parameter("joint_kd", {1.2, 1.2, 1.0});
    desired_position_ = vector_parameter("default_joint_position", {0.0, 0.0, 0.0});
    if (control_hz_ <= 0.0 || reference_timeout_s_ <= 0.0) {
      throw std::runtime_error("control_hz and reference_timeout_s must be positive");
    }
    for (std::size_t index = 0; index < kp_.size(); ++index) {
      if (kp_[index] < 0.0 || kd_[index] < 0.0) {
        throw std::runtime_error("joint_kp and joint_kd must be non-negative");
      }
    }

    state_subscriber_ = create_subscription<chr_msgs::msg::ChrState>(
      "/chr/state", 1, [this](const chr_msgs::msg::ChrState::SharedPtr message) {
        state_ = message;
      });
    reference_subscriber_ = create_subscription<chr_msgs::msg::ChrReference>(
      "/chr/reference", 1, [this](const chr_msgs::msg::ChrReference::SharedPtr message) {
        desired_position_ = message->joint_position;
        desired_velocity_ = message->joint_velocity;
        last_reference_time_ = now();
        have_reference_ = true;
      });
    command_publisher_ = create_publisher<chr_msgs::msg::ArmCommand>(
      "/chr/actuator/arm", 1);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_hz_),
      std::bind(&ArmController::update, this));
  }

 private:
  arm_controller::JointVector vector_parameter(
      const std::string &name, const arm_controller::JointVector &defaults) {
    const auto values = declare_parameter<std::vector<double>>(
      name, {defaults[0], defaults[1], defaults[2]});
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain exactly three values");
    }
    if (!std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
      })) {
      throw std::runtime_error(name + " must contain only finite values");
    }
    return {values[0], values[1], values[2]};
  }

  void update() {
    if (!state_) return;
    const bool stale = !have_reference_ ||
      (now() - last_reference_time_).seconds() > reference_timeout_s_;
    if (stale) {
      // Freezing the measured pose is safer than replaying an expired planner target.
      desired_position_ = state_->joint_position;
      desired_velocity_ = {};
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "CHR reference is stale; holding measured arm position");
    }

    const auto clamped_position = arm_controller::clamp_joint_position(desired_position_);
    const auto feedforward = arm_controller::select_gravity_feedforward(
      state_->joint_torque_grav, gravity_compensation_);
    chr_msgs::msg::ArmCommand command;
    command.header.stamp = now();
    command.header.frame_id = "arm_mount";
    command.joint_position = clamped_position;
    command.joint_velocity = desired_velocity_;
    command.joint_kp = kp_;
    command.joint_kd = kd_;
    command.joint_effort_feedforward = feedforward;
    command_publisher_->publish(command);
  }

  double control_hz_{250.0};
  double reference_timeout_s_{0.5};
  bool gravity_compensation_{false};
  bool have_reference_{false};
  arm_controller::JointVector kp_{};
  arm_controller::JointVector kd_{};
  arm_controller::JointVector desired_position_{};
  arm_controller::JointVector desired_velocity_{};
  rclcpp::Time last_reference_time_{0, 0, RCL_ROS_TIME};
  chr_msgs::msg::ChrState::SharedPtr state_;
  rclcpp::Subscription<chr_msgs::msg::ChrState>::SharedPtr state_subscriber_;
  rclcpp::Subscription<chr_msgs::msg::ChrReference>::SharedPtr reference_subscriber_;
  rclcpp::Publisher<chr_msgs::msg::ArmCommand>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmController>());
  rclcpp::shutdown();
  return 0;
}
