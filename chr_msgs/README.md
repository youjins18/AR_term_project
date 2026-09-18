# chr_msgs

Shared contracts for the CHR stack. The high-level controller publishes one
`ChrReference`, containing the full base pose and J1--J3 command. Base attitude is
encoded as a quaternion, but the current CHR invariant fixes roll and pitch to
zero. The independent command is therefore
`[x_b, y_b, z_b, yaw_b, J1, J2, J3]`. Controllers consume only their own fields.

Plant inputs are deliberately separate from references:

- `/chr/reference`: desired base pose/twist and arm joint state
- `/chr/actuator/palletrone`: rotor thrust and tilt-servo angles
- `/chr/actuator/arm`: position-loop arm command
- `/chr/state`: simulator or hardware state estimate
- `/chr/diagnostics/flight`: final force, nominal/final/DOB-estimated torque, allocator,
  calibrated PWM and servo/thrust outputs
- `/chr/diagnostics/ik`: DLS convergence, residual and conditioning metrics

`ChrState` reports four explicit arm torque definitions: ideal joint-sensor
measurement (`joint_torque_meas`), model `M(q)qdd+C(q,dq)+G(q)` inverse dynamics
(`joint_torque_dyn`), gravity only (`joint_torque_grav`), and the saturated
actuator input (`joint_torque_command`). TCP pose is read through MuJoCo frame
sensors rather than directly copied from a controller target.
