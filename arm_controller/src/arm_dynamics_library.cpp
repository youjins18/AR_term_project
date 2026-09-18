// Copyright 2026 mrl_nuc
// SPDX-License-Identifier: Apache-2.0

#include "arm_controller/arm_dynamics_library.hpp"

#include <algorithm>

namespace arm_controller {

JointVector clamp_joint_position(const JointVector &position) {
  constexpr JointVector lower{0.0, -1.5708, -1.5708};
  constexpr JointVector upper{1.5708, 1.5708, 1.5708};
  JointVector output{};
  for (std::size_t index = 0; index < output.size(); ++index) {
    output[index] = std::clamp(position[index], lower[index], upper[index]);
  }
  return output;
}

JointVector select_gravity_feedforward(const JointVector &gravity_torque, bool enabled) {
  return enabled ? gravity_torque : JointVector{};
}

}  // namespace arm_controller
