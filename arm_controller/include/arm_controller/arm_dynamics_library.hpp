#pragma once

#include <array>

namespace arm_controller {

using JointVector = std::array<double, 3>;

/// Apply the same mechanical limits used by the MuJoCo J1--J3 joints.
JointVector clamp_joint_position(const JointVector &position);
/// Return the model gravity torque only when compensation is enabled.
JointVector select_gravity_feedforward(const JointVector &gravity_torque, bool enabled);

}  // namespace arm_controller
