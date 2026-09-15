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

Rotor order and axes match `chr_description/mujoco/palletrone.xml`: (+x,+y),
(-x,+y), (-x,-y), (+x,-y), with radial tilt axes and a 0.21 m radius.

The default total dynamic mass is 6.474 kg (Palletrone, rotor tilt bodies and the
full arm; the mocap marker is excluded). Gains are safe initial simulation values,
not flight-tested gains. Keep the DOB disabled until the nominal inertia and
actuator signs are identified from hardware.
