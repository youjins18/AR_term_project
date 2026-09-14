#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class KeyboardCommander final : public rclcpp::Node {
 public:
  KeyboardCommander() : Node("chr_keyboard_commander") {
    enabled_ = declare_parameter("enabled", false);
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 30000,
        "keyboard commander is intentionally blank (enabled=%s)", enabled_ ? "true" : "false");
    });
  }

 private:
  bool enabled_{false};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardCommander>());
  rclcpp::shutdown();
  return 0;
}
