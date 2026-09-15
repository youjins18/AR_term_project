// Copyright 2026 mrl_nuc
// SPDX-License-Identifier: Apache-2.0

#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "chr_msgs/msg/chr_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr char kEscape = 27;
constexpr char kWorldFrame[] = "world";
constexpr char kStateTopic[] = "/chr/state";
constexpr char kTcpTargetTopic[] = "/chr/target/tcp_pose";

using Vec3 = std::array<double, 3>;
using Quaternion = geometry_msgs::msg::Quaternion;

Quaternion normalized(Quaternion q) {
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm < 1e-12) {
    q.w = 1.0;
    q.x = q.y = q.z = 0.0;
    return q;
  }
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return q;
}

Quaternion product(const Quaternion &a, const Quaternion &b) {
  Quaternion result;
  result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return normalized(result);
}

Quaternion axis_angle(int axis, double angle) {
  Quaternion result;
  result.w = std::cos(0.5 * angle);
  result.x = result.y = result.z = 0.0;
  if (axis == 0) result.x = std::sin(0.5 * angle);
  if (axis == 1) result.y = std::sin(0.5 * angle);
  if (axis == 2) result.z = std::sin(0.5 * angle);
  return result;
}

Vec3 quaternion_to_rpy(const Quaternion &input) {
  const auto q = normalized(input);
  const double roll = std::atan2(
    2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  const double pitch_argument = std::clamp(
    2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
  const double pitch = std::asin(pitch_argument);
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return {roll, pitch, yaw};
}
}  // namespace

class KeyboardCommander final : public rclcpp::Node {
 public:
  KeyboardCommander() : Node("chr_keyboard_commander") {
    enabled_ = declare_parameter("enabled", true);
    translation_step_m_ = declare_parameter("translation_step_m", 0.01);
    const double rotation_step_deg = declare_parameter("rotation_step_deg", 3.0);
    rotation_step_rad_ = rotation_step_deg * kPi / 180.0;
    poll_hz_ = declare_parameter("poll_hz", 50.0);
    workspace_min_ = vector_parameter("workspace_min", {-1.5, -1.5, 0.05});
    workspace_max_ = vector_parameter("workspace_max", {1.5, 1.5, 2.5});
    if (translation_step_m_ <= 0.0 || rotation_step_rad_ <= 0.0 || poll_hz_ <= 0.0) {
      throw std::runtime_error("teleop step sizes and poll_hz must be positive");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      if (workspace_min_[i] >= workspace_max_[i]) {
        throw std::runtime_error("workspace_min must be smaller than workspace_max");
      }
    }

    target_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      kTcpTargetTopic, 1);
    state_subscriber_ = create_subscription<chr_msgs::msg::ChrState>(
      kStateTopic, rclcpp::SensorDataQoS(),
      std::bind(&KeyboardCommander::receive_state, this, std::placeholders::_1));

    if (!enabled_) {
      RCLCPP_WARN(get_logger(), "keyboard teleop is disabled by parameter");
      return;
    }
    configure_terminal();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / poll_hz_),
      std::bind(&KeyboardCommander::poll_keyboard, this));
    print_help();
    RCLCPP_INFO(get_logger(), "waiting for /chr/state before accepting commands");
  }

  ~KeyboardCommander() override { restore_terminal(); }

 private:
  Vec3 vector_parameter(const std::string &name, const Vec3 &defaults) {
    const auto values = declare_parameter<std::vector<double>>(
      name, {defaults[0], defaults[1], defaults[2]});
    if (values.size() != 3) throw std::runtime_error(name + " must contain three values");
    if (!std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
      })) {
      throw std::runtime_error(name + " must contain only finite values");
    }
    return {values[0], values[1], values[2]};
  }

  void configure_terminal() {
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error(
        "keyboard teleop needs a TTY; run it directly with "
        "'ros2 run chr_commander keyboard_commander'");
    }
    if (tcgetattr(STDIN_FILENO, &original_terminal_) != 0) {
      throw std::runtime_error("failed to read terminal settings");
    }
    termios raw = original_terminal_;
    // Keep ISIG enabled so Ctrl-C remains available; only canonical buffering
    // and local echo are disabled for one-key commands.
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error("failed to enable raw keyboard input");
    }
    terminal_configured_ = true;
  }

  void restore_terminal() {
    if (terminal_configured_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
      terminal_configured_ = false;
    }
  }

  void receive_state(const chr_msgs::msg::ChrState::SharedPtr message) {
    measured_pose_ = message->tcp_pose;
    measured_pose_.orientation = normalized(measured_pose_.orientation);
    if (target_initialized_) return;
    target_pose_ = measured_pose_;
    home_pose_ = measured_pose_;
    target_initialized_ = true;
    publish_target();
    RCLCPP_INFO(get_logger(), "TCP target initialized from measured pose");
    print_target();
  }

  void poll_keyboard() {
    char key = 0;
    while (read(STDIN_FILENO, &key, 1) > 0) handle_key(key);
  }

  void handle_key(char raw_key) {
    if (raw_key == kEscape || raw_key == 'q' || raw_key == 'Q') {
      RCLCPP_INFO(get_logger(), "keyboard teleop stopped");
      restore_terminal();
      rclcpp::shutdown();
      return;
    }
    if (raw_key == '?') {
      print_help();
      return;
    }
    if (!target_initialized_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "no /chr/state received yet; command ignored");
      return;
    }

    const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(raw_key)));
    bool changed = true;
    switch (key) {
      case 'w': target_pose_.position.x += translation_step_m_; break;
      case 's': target_pose_.position.x -= translation_step_m_; break;
      case 'a': target_pose_.position.y += translation_step_m_; break;
      case 'd': target_pose_.position.y -= translation_step_m_; break;
      case 'r': target_pose_.position.z += translation_step_m_; break;
      case 'f': target_pose_.position.z -= translation_step_m_; break;
      case 'i': rotate_local(0, rotation_step_rad_); break;
      case 'k': rotate_local(0, -rotation_step_rad_); break;
      case 'j': rotate_local(1, rotation_step_rad_); break;
      case 'l': rotate_local(1, -rotation_step_rad_); break;
      case 'u': rotate_local(2, rotation_step_rad_); break;
      case 'o': rotate_local(2, -rotation_step_rad_); break;
      case ' ': target_pose_ = measured_pose_; break;
      case '0': target_pose_ = home_pose_; break;
      case 'p': print_target(); changed = false; break;
      default: changed = false; break;
    }
    if (!changed) return;
    clamp_position();
    publish_target();
    print_target();
  }

  void rotate_local(int axis, double angle) {
    // Right multiplication expresses the increment in the current TCP frame.
    target_pose_.orientation = product(target_pose_.orientation, axis_angle(axis, angle));
  }

  void clamp_position() {
    target_pose_.position.x = std::clamp(
      target_pose_.position.x, workspace_min_[0], workspace_max_[0]);
    target_pose_.position.y = std::clamp(
      target_pose_.position.y, workspace_min_[1], workspace_max_[1]);
    target_pose_.position.z = std::clamp(
      target_pose_.position.z, workspace_min_[2], workspace_max_[2]);
  }

  void publish_target() {
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = now();
    target.header.frame_id = kWorldFrame;
    target.pose = target_pose_;
    target_publisher_->publish(target);
  }

  void print_help() {
    RCLCPP_INFO(
      get_logger(),
      "\nCHR TCP teleop (local TCP attitude axes)\n"
      "  position: W/S +X/-X, A/D +Y/-Y, R/F +Z/-Z\n"
      "  attitude: I/K +roll/-roll, J/L +pitch/-pitch, U/O +yaw/-yaw\n"
      "  Space: hold measured pose, 0: startup pose, P: print, ?: help, Q/Esc: quit\n"
      "  steps: %.3f m, %.1f deg",
      translation_step_m_, rotation_step_rad_ * 180.0 / kPi);
  }

  void print_target() {
    const auto rpy = quaternion_to_rpy(target_pose_.orientation);
    RCLCPP_INFO(
      get_logger(), "target xyz=[%.3f %.3f %.3f] m, rpy=[%.1f %.1f %.1f] deg",
      target_pose_.position.x, target_pose_.position.y, target_pose_.position.z,
      rpy[0] * 180.0 / kPi,
      rpy[1] * 180.0 / kPi,
      rpy[2] * 180.0 / kPi);
  }

  bool enabled_{true};
  bool terminal_configured_{false};
  bool target_initialized_{false};
  double translation_step_m_{0.01};
  double rotation_step_rad_{0.0523598776};
  double poll_hz_{50.0};
  Vec3 workspace_min_{-1.5, -1.5, 0.05};
  Vec3 workspace_max_{1.5, 1.5, 2.5};
  termios original_terminal_{};
  geometry_msgs::msg::Pose target_pose_;
  geometry_msgs::msg::Pose measured_pose_;
  geometry_msgs::msg::Pose home_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_publisher_;
  rclcpp::Subscription<chr_msgs::msg::ChrState>::SharedPtr state_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KeyboardCommander>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("chr_keyboard_commander"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
