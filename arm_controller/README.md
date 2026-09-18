# arm_controller

This node selects J1--J3 from the
whole-body `/chr/reference`, clamps them to the model limits, attaches
position-loop gains, and publishes `/chr/actuator/arm`.

The MuJoCo plant implements the actual PD torque law, mirroring a Dynamixel-style
position-control boundary. Gravity feedforward is enabled in the simulation
configuration to remove steady-state error; disable it before connecting a
hardware driver until signs and units are verified. A stale-reference watchdog
holds the measured arm position.

The state contract exposes four unambiguous torque signals for logging:

- `joint_torque_meas`: ideal J1--J3 `jointactuatorfrc` sensors;
- `joint_torque_dyn`: full-system `M(q)qdd+C(q,dq)+G(q)` at the arm DOFs;
- `joint_torque_grav`: gravity term evaluated at zero velocity/acceleration;
- `joint_torque_command`: clipped motor command applied by MuJoCo.
