# Validation record

Validated through 2026-09-18 with Ubuntu 22.04, ROS 2 Humble, MuJoCo Python 3.9.0,
Eigen 3.4 and GCC 11.4.

- All eight ROS package manifests parse and export their correct build type.
- All Python and launch files pass bytecode compilation; all YAML files parse.
- `colcon build --symlink-install --executor sequential` finishes all 8 packages.
- Standard `source install/setup.bash` exposes all 8 packages and 5 executables.
- MuJoCo loads the installed `scene.xml`: `nq=14`, `nv=13`, `nu=11`,
  `nsensor=5`, `nsensordata=10`, `nmesh=8`, dynamic CHR mass 6.474 kg.
- Palletrone `BODY.stl` (15,648 triangles) and `PROP.stl` (1,596 triangles)
  load at their native metre scale. The four rotor sites lie at
  `(+-0.148492, +-0.148492, 0.07) m`, giving a 0.21 m rotor radius, and each
  tilt axis is radial.
- The arm mount is at `(-0.10, 0, -0.13) m`: 10 cm along Palletrone body -X and
  immediately below the BODY mesh's `z=-0.125 m` lower bound.
- The exact C++ nominal FK and MuJoCo FK agree at home:
  `[-0.1000036640, -9.183118e-7, 0.2777500000] m` with the base at `z=1.2 m`.
- A 100-step (0.2 s) direct hover physics test ends at `z=1.199829 m`.
- With the 10 cm arm offset, a 25-second ROS closed-loop hold run ends at
  `[-0.000004, -0.000001, 1.200072] m` with 0.00375 rad roll/pitch error.
  PWM remains approximately 1344--1423 us and the allocator is not saturated.
- DLS recovery of the known configuration `[0.5, 0.3, -0.4]` converges in 14
  iterations to `[0.49926, 0.30136, -0.39991]` with 0.067 mm TCP residual.
- Constrained 6x7 pose DLS recovers a known translated and rotated TCP target in
  7 iterations with 0.0054 mm position and 0.000105 rad attitude residual. A
  deliberately tilted seed still returns a base quaternion with exactly zero
  roll and pitch components.
- Raw-terminal keyboard input initializes from measured `/chr/state` and
  publishes cumulative XYZ plus local-roll/pitch/yaw targets to
  `/chr/target/tcp_pose`; terminal settings are restored on quit.
- DLS teleop maintains exactly zero commanded base roll, pitch and x/y angular
  velocity while solving local-RPY TCP commands.
- External-planner input containing nonzero base roll/pitch and x/y angular
  velocity is projected to `[roll, pitch] = [0, 0]` and `[wx, wy] = [0, 0]` while
  preserving yaw and yaw rate.
- The MuJoCo state path reports ideal joint-sensor torque, full-system
  `M(q)qdd+C(q,dq)+G(q)`, gravity-only torque, and clipped arm command as four
  independent J1--J3 arrays. TCP position and quaternion come from frame
  sensors. A direct runtime snapshot produced finite values for every signal.
- The Palletrone PWM inverse reproduces the supplied PX4 firmware polynomial
  and its 0.8 normalized clamp: a 1900 us-equivalent thrust is reported as the
  effective 1740 us limit. Runtime flight diagnostics publish final force,
  nominal/final torque, DOB torque, thrust, PWM, servo and allocator health.
- A headless full-stack smoke run publishes `/chr/state`,
  `/chr/diagnostics/flight`, and `/chr/diagnostics/ik` without node errors.
- The MATLAB script passes MATLAB R2026a `mlint` with no findings. A synthetic
  five-topic rosbag exports to a 5-row, 96-column CSV inside its bag directory;
  all new flight, arm, TCP and DLS columns are present.
- Offscreen rendering confirms the black/white/orange robot palette, orange
  propellers and measured TCP, plus a blue target sphere and local-+Z attitude
  arrow that follows the target quaternion.
- `colcon test` reports 11 tests, 0 errors, 0 failures and 0 skipped, including
  PWM inversion/clamping, gravity-feedforward selection and constrained DLS.

The GLFW warning seen on the validation host is Wayland/window-position related;
the combined model and passive viewer both initialize. Controller gains and the
DOB remain simulation starting points and require identification before hardware.
