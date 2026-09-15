// Copyright 2026 mrl_nuc
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "chr_msgs/msg/chr_reference.hpp"
#include "chr_msgs/msg/chr_state.hpp"
#include "chr_msgs/msg/flight_diagnostics.hpp"
#include "chr_msgs/msg/palletrone_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
using Vec3 = std::array<double, 3>;
using Vec4 = std::array<double, 4>;

Vec3 add(const Vec3 &a, const Vec3 &b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 subtract(const Vec3 &a, const Vec3 &b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 multiply(const Vec3 &a, const Vec3 &b) {
  return {a[0] * b[0], a[1] * b[1], a[2] * b[2]};
}

Vec3 scale(const Vec3 &a, double value) {
  return {a[0] * value, a[1] * value, a[2] * value};
}

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Quaternion normalized(Quaternion q) {
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm < 1e-12) return {};
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return q;
}

Quaternion conjugate(const Quaternion &q) { return {q.w, -q.x, -q.y, -q.z}; }

Quaternion product(const Quaternion &a, const Quaternion &b) {
  return {
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

Vec3 world_to_body(const Quaternion &input, const Vec3 &v) {
  const auto q = normalized(input);
  return {
    (1 - 2 * (q.y * q.y + q.z * q.z)) * v[0] +
      2 * (q.x * q.y + q.w * q.z) * v[1] +
      2 * (q.x * q.z - q.w * q.y) * v[2],
    2 * (q.x * q.y - q.w * q.z) * v[0] +
      (1 - 2 * (q.x * q.x + q.z * q.z)) * v[1] +
      2 * (q.y * q.z + q.w * q.x) * v[2],
    2 * (q.x * q.z + q.w * q.y) * v[0] +
      2 * (q.y * q.z - q.w * q.x) * v[1] +
      (1 - 2 * (q.x * q.x + q.y * q.y)) * v[2],
  };
}

Vec3 orientation_error(const Quaternion &current_input, const Quaternion &desired_input) {
  const auto current = normalized(current_input);
  const auto desired = normalized(desired_input);
  auto error = product(conjugate(current), desired);
  const double sign = error.w >= 0.0 ? 1.0 : -1.0;
  return {2.0 * sign * error.x, 2.0 * sign * error.y, 2.0 * sign * error.z};
}

struct Allocation {
  Vec4 thrust{};
  Vec4 servo{};
  double residual{0.0};
  bool saturated{false};
};
}  // namespace

class PalletroneFlightController final : public rclcpp::Node {
 public:
  PalletroneFlightController() : Node("palletrone_flight_controller") {
    control_hz_ = declare_parameter("control_hz", 250.0);
    mass_kg_ = declare_parameter("mass_kg", 6.474);
    gravity_ = declare_parameter("gravity", 9.81);
    position_kp_ = vector_parameter("position_kp", {4.0, 4.0, 7.0});
    position_ki_ = vector_parameter("position_ki", {0.05, 0.05, 0.2});
    position_kd_ = vector_parameter("position_kd", {4.0, 4.0, 5.0});
    position_integral_limit_ = vector_parameter("position_integral_limit", {1.0, 1.0, 1.0});
    attitude_kp_ = vector_parameter("attitude_kp", {8.0, 8.0, 4.0});
    attitude_ki_ = vector_parameter("attitude_ki", {3.0, 3.0, 1.0});
    attitude_kd_ = vector_parameter("attitude_kd", {2.0, 2.0, 1.2});
    attitude_integral_limit_ = vector_parameter(
      "attitude_integral_limit", {0.5, 0.5, 0.5});
    inertia_ = vector_parameter("inertia_diagonal_kg_m2", {0.18, 0.18, 0.20});
    dob_gain_ = vector_parameter("dob_gain", {0.0, 0.0, 0.0});
    dob_enabled_ = declare_parameter("dob_enabled", false);
    dob_cutoff_hz_ = declare_parameter("dob_cutoff_hz", 8.0);
    rotor_radius_m_ = declare_parameter("rotor_radius_m", 0.21);
    reaction_coefficient_ = declare_parameter("reaction_torque_coefficient", 0.01);
    minimum_thrust_n_ = declare_parameter("minimum_thrust_n", 0.0);
    maximum_thrust_n_ = declare_parameter("maximum_thrust_n", 25.0);
    maximum_servo_angle_rad_ = declare_parameter("maximum_servo_angle_rad", 0.6);
    reference_timeout_s_ = declare_parameter("reference_timeout_s", 0.5);
    validate_parameters();

    state_subscriber_ = create_subscription<chr_msgs::msg::ChrState>(
      "/chr/state", 1, [this](const chr_msgs::msg::ChrState::SharedPtr message) {
        state_ = message;
      });
    reference_subscriber_ = create_subscription<chr_msgs::msg::ChrReference>(
      "/chr/reference", 1, [this](const chr_msgs::msg::ChrReference::SharedPtr message) {
        reference_ = message;
        last_reference_time_ = now();
      });
    command_publisher_ = create_publisher<chr_msgs::msg::PalletroneCommand>(
      "/chr/actuator/palletrone", 1);
    diagnostics_publisher_ = create_publisher<chr_msgs::msg::FlightDiagnostics>(
      "/chr/diagnostics/flight", 1);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_hz_),
      std::bind(&PalletroneFlightController::update, this));
  }

 private:
  Vec3 vector_parameter(const std::string &name, const Vec3 &defaults) {
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

  void validate_parameters() const {
    if (control_hz_ <= 0.0 || mass_kg_ <= 0.0 || gravity_ <= 0.0 ||
        dob_cutoff_hz_ <= 0.0 || rotor_radius_m_ <= 0.0 ||
        reaction_coefficient_ <= 0.0 || maximum_servo_angle_rad_ <= 0.0 ||
        reference_timeout_s_ <= 0.0 || minimum_thrust_n_ < 0.0 ||
        maximum_thrust_n_ <= minimum_thrust_n_) {
      throw std::runtime_error("invalid physical, rate, timeout, or actuator-limit parameter");
    }
  }

  Allocation allocate(const Vec3 &body_force, const Vec3 &body_torque) const {
    Vec4 vertical{
      body_force[2] / 4.0, body_force[2] / 4.0,
      body_force[2] / 4.0, body_force[2] / 4.0};
    bool saturated = false;

    // Apply each torque component sequentially and preserve its direction if a
    // rotor reaches a limit; this avoids clipping individual motors afterward.
    const auto apply_delta = [&](const Vec4 &delta) {
      double factor = 1.0;
      for (std::size_t i = 0; i < vertical.size(); ++i) {
        if (delta[i] > 0.0) {
          factor = std::min(factor, (maximum_thrust_n_ - vertical[i]) / delta[i]);
        } else if (delta[i] < 0.0) {
          factor = std::min(factor, (minimum_thrust_n_ - vertical[i]) / delta[i]);
        }
      }
      factor = std::clamp(factor, 0.0, 1.0);
      saturated = saturated || factor < 0.999999;
      for (std::size_t i = 0; i < vertical.size(); ++i) vertical[i] += factor * delta[i];
    };

    // X-configuration: rotor order is (+x,+y), (-x,+y), (-x,-y), (+x,-y).
    const double diagonal_coordinate = std::max(rotor_radius_m_ / std::sqrt(2.0), 1e-6);
    const double roll_delta = body_torque[0] / (4.0 * diagonal_coordinate);
    apply_delta({roll_delta, roll_delta, -roll_delta, -roll_delta});
    const double pitch_delta = body_torque[1] / (4.0 * diagonal_coordinate);
    apply_delta({-pitch_delta, pitch_delta, pitch_delta, -pitch_delta});
    const double yaw_delta = body_torque[2] / std::max(4.0 * reaction_coefficient_, 1e-6);
    apply_delta({yaw_delta, -yaw_delta, yaw_delta, -yaw_delta});

    Allocation result;
    constexpr double inverse_sqrt_two = 0.7071067811865476;
    const std::array<Vec3, 4> tangent{{
      {inverse_sqrt_two, -inverse_sqrt_two, 0.0},
      {inverse_sqrt_two, inverse_sqrt_two, 0.0},
      {-inverse_sqrt_two, inverse_sqrt_two, 0.0},
      {-inverse_sqrt_two, -inverse_sqrt_two, 0.0},
    }};
    for (std::size_t i = 0; i < result.servo.size(); ++i) {
      // For this X layout, Sum(t_i t_i^T) = 2I. Half of each tangent
      // projection distributes horizontal force without a matrix inverse.
      const double horizontal = 0.5 * (
        body_force[0] * tangent[i][0] + body_force[1] * tangent[i][1]);
      result.servo[i] = std::clamp(
        std::atan2(horizontal, std::max(vertical[i], 0.1)),
        -maximum_servo_angle_rad_, maximum_servo_angle_rad_);
    }
    for (std::size_t i = 0; i < result.thrust.size(); ++i) {
      const double unclamped = vertical[i] / std::max(std::cos(result.servo[i]), 0.2);
      result.thrust[i] = std::clamp(unclamped, minimum_thrust_n_, maximum_thrust_n_);
      saturated = saturated || std::abs(result.thrust[i] - unclamped) > 1e-9;
    }

    Vec3 achieved{0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < result.thrust.size(); ++i) {
      achieved[0] += result.thrust[i] * std::sin(result.servo[i]) * tangent[i][0];
      achieved[1] += result.thrust[i] * std::sin(result.servo[i]) * tangent[i][1];
      achieved[2] += result.thrust[i] * std::cos(result.servo[i]);
    }
    const Vec3 residual = subtract(body_force, achieved);
    result.residual = std::sqrt(
      residual[0] * residual[0] + residual[1] * residual[1] + residual[2] * residual[2]);
    result.saturated = saturated;
    return result;
  }

  void update() {
    if (!state_ || !reference_) return;
    if ((now() - last_reference_time_).seconds() > reference_timeout_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "CHR reference is stale; suppressing flight command");
      return;
    }

    const Vec3 position{
      state_->base_pose.position.x, state_->base_pose.position.y, state_->base_pose.position.z};
    const Vec3 velocity{
      state_->base_twist.linear.x, state_->base_twist.linear.y, state_->base_twist.linear.z};
    const Vec3 desired_position{
      reference_->base_pose.position.x, reference_->base_pose.position.y,
      reference_->base_pose.position.z};
    const Vec3 desired_velocity{
      reference_->base_twist.linear.x, reference_->base_twist.linear.y,
      reference_->base_twist.linear.z};
    const Vec3 position_error = subtract(desired_position, position);
    for (std::size_t i = 0; i < position_integral_.size(); ++i) {
      position_integral_[i] = std::clamp(
        position_integral_[i] + position_error[i] / control_hz_,
        -position_integral_limit_[i], position_integral_limit_[i]);
    }

    Vec3 world_force = scale(add(
      add(multiply(position_kp_, position_error), multiply(position_ki_, position_integral_)),
      multiply(position_kd_, subtract(desired_velocity, velocity))), mass_kg_);
    world_force[2] += mass_kg_ * gravity_;

    const Quaternion current{
      state_->base_pose.orientation.w, state_->base_pose.orientation.x,
      state_->base_pose.orientation.y, state_->base_pose.orientation.z};
    const Quaternion desired{
      reference_->base_pose.orientation.w, reference_->base_pose.orientation.x,
      reference_->base_pose.orientation.y, reference_->base_pose.orientation.z};
    const Vec3 omega{
      state_->base_twist.angular.x, state_->base_twist.angular.y,
      state_->base_twist.angular.z};
    const Vec3 desired_omega{
      reference_->base_twist.angular.x, reference_->base_twist.angular.y,
      reference_->base_twist.angular.z};
    const Vec3 attitude_error = orientation_error(current, desired);
    // The suspended arm produces a constant gravity moment. A bounded
    // integrator removes the steady tilt without allowing unlimited windup.
    for (std::size_t i = 0; i < attitude_integral_.size(); ++i) {
      attitude_integral_[i] = std::clamp(
        attitude_integral_[i] + attitude_error[i] / control_hz_,
        -attitude_integral_limit_[i], attitude_integral_limit_[i]);
    }
    Vec3 body_torque = add(
      add(multiply(attitude_kp_, attitude_error),
        multiply(attitude_ki_, attitude_integral_)),
      multiply(attitude_kd_, subtract(desired_omega, omega)));

    constexpr double pi = 3.14159265358979323846;
    const double alpha = 1.0 - std::exp(-2.0 * pi * dob_cutoff_hz_ / control_hz_);
    const Vec3 nominal_torque = body_torque;
    for (std::size_t i = 0; i < disturbance_estimate_.size(); ++i) {
      const double acceleration = (omega[i] - previous_omega_[i]) * control_hz_;
      const double raw_disturbance = inertia_[i] * acceleration - previous_torque_[i];
      disturbance_estimate_[i] += alpha * (raw_disturbance - disturbance_estimate_[i]);
      if (dob_enabled_) body_torque[i] -= dob_gain_[i] * disturbance_estimate_[i];
      previous_omega_[i] = omega[i];
      previous_torque_[i] = nominal_torque[i];
    }

    const Vec3 body_force = world_to_body(current, world_force);
    const Allocation allocation = allocate(body_force, body_torque);
    chr_msgs::msg::PalletroneCommand command;
    command.header.stamp = now();
    command.header.frame_id = "base";
    command.rotor_thrust = allocation.thrust;
    command.servo_angle = allocation.servo;
    command_publisher_->publish(command);

    chr_msgs::msg::FlightDiagnostics diagnostics;
    diagnostics.header = command.header;
    diagnostics.desired_wrench.force.x = body_force[0];
    diagnostics.desired_wrench.force.y = body_force[1];
    diagnostics.desired_wrench.force.z = body_force[2];
    diagnostics.desired_wrench.torque.x = body_torque[0];
    diagnostics.desired_wrench.torque.y = body_torque[1];
    diagnostics.desired_wrench.torque.z = body_torque[2];
    diagnostics.disturbance_estimate.torque.x = disturbance_estimate_[0];
    diagnostics.disturbance_estimate.torque.y = disturbance_estimate_[1];
    diagnostics.disturbance_estimate.torque.z = disturbance_estimate_[2];
    diagnostics.allocated_thrust = allocation.thrust;
    diagnostics.allocated_servo_angle = allocation.servo;
    diagnostics.allocation_residual_norm = allocation.residual;
    diagnostics.saturated = allocation.saturated;
    diagnostics_publisher_->publish(diagnostics);
  }

  double control_hz_{250.0};
  double mass_kg_{6.474};
  double gravity_{9.81};
  double dob_cutoff_hz_{8.0};
  double rotor_radius_m_{0.21};
  double reaction_coefficient_{0.01};
  double minimum_thrust_n_{0.0};
  double maximum_thrust_n_{25.0};
  double maximum_servo_angle_rad_{0.6};
  double reference_timeout_s_{0.5};
  bool dob_enabled_{false};
  Vec3 position_kp_{};
  Vec3 position_ki_{};
  Vec3 position_kd_{};
  Vec3 position_integral_limit_{};
  Vec3 position_integral_{};
  Vec3 attitude_kp_{};
  Vec3 attitude_ki_{};
  Vec3 attitude_kd_{};
  Vec3 attitude_integral_limit_{};
  Vec3 attitude_integral_{};
  Vec3 inertia_{};
  Vec3 dob_gain_{};
  Vec3 disturbance_estimate_{};
  Vec3 previous_omega_{};
  Vec3 previous_torque_{};
  rclcpp::Time last_reference_time_{0, 0, RCL_ROS_TIME};
  chr_msgs::msg::ChrState::SharedPtr state_;
  chr_msgs::msg::ChrReference::SharedPtr reference_;
  rclcpp::Subscription<chr_msgs::msg::ChrState>::SharedPtr state_subscriber_;
  rclcpp::Subscription<chr_msgs::msg::ChrReference>::SharedPtr reference_subscriber_;
  rclcpp::Publisher<chr_msgs::msg::PalletroneCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<chr_msgs::msg::FlightDiagnostics>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PalletroneFlightController>());
  rclcpp::shutdown();
  return 0;
}
