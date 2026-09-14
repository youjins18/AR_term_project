#pragma once

#include <array>

namespace arm_controller {

using JointVector = std::array<double, 3>;

JointVector clamp_joint_position(const JointVector &position);
JointVector select_bias_feedforward(const JointVector &bias_torque, bool enabled);

}  // namespace arm_controller
