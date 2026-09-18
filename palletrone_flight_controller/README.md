# palletrone_flight_controller

Low-level base-pose controller. It consumes the base fields of `/chr/reference`
and publishes only `/chr/actuator/palletrone`.

The node contains:

1. world-frame position PID with gravity feedforward;
2. quaternion attitude PID with bounded integral compensation for the suspended
   arm's static gravity moment;
3. an optional, disabled-by-default torque disturbance observer;
4. X-configuration roll/pitch/yaw thrust allocation followed by tangent-axis
   horizontal-force tilt allocation and actuator saturation.

`/chr/diagnostics/flight` keeps only the final body force, while separating
nominal attitude torque, DOB-estimated disturbance torque and final torque. The
DOB estimate is logged even when compensation is disabled; enabling compensation
applies `-dob_gain * dob_torque` to the nominal torque. The diagnostics also
include thrust, servo commands, allocator health and motor PWM. The PWM
conversion reproduces the custom Palletrone PX4 calibration
`F = 2.0962e-5 PWM^2 + 0.0085 PWM - 36.0347`, including its 0.8 normalized
output limit (1100--1740 us effective range). All coefficients remain ROS
parameters so later propulsion calibration can replace them without code edits.

Rotor order and axes match `chr_description/mujoco/palletrone.xml`: (+x,+y),
(-x,+y), (-x,-y), (+x,-y), with radial tilt axes and a 0.21 m radius.

The default total dynamic mass is 6.474 kg (Palletrone, rotor tilt bodies and the
full arm; the mocap marker is excluded). Gains are initial simulation values,
not flight-tested gains. Keep the DOB disabled until the nominal inertia and
actuator signs are identified from hardware.
