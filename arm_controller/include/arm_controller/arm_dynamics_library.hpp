// Copyright 2026 mrl_nuc
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>

namespace arm_controller {

using JointVector = std::array<double, 3>;

/// Apply the same mechanical limits used by the MuJoCo J1--J3 joints.
JointVector clamp_joint_position(const JointVector &position);
/// Return MuJoCo bias torque only when gravity compensation is enabled.
JointVector select_bias_feedforward(const JointVector &bias_torque, bool enabled);

}  // namespace arm_controller
