# Validation record

Validated through 2026-09-15 with Ubuntu 22.04, ROS 2 Humble, MuJoCo Python 3.9.0,
Eigen 3.4 and GCC 11.4.

- All eight ROS package manifests parse and export their correct build type.
- All Python and launch files pass bytecode compilation; all YAML files parse.
- `colcon build --symlink-install --executor sequential` finishes all 8 packages.
- Standard `source install/setup.bash` exposes all 8 packages and 5 executables.
- MuJoCo loads the installed `scene.xml`: `nq=14`, `nv=13`, `nu=11`,
  `nmesh=7`, dynamic CHR mass 6.474 kg.
- Palletrone `BODY.stl` (15,648 triangles) and `PROP.stl` (1,596 triangles)
  load at their native metre scale. The four rotor sites lie at
  `(+-0.148492, +-0.148492, 0.07) m`, giving a 0.21 m rotor radius, and each
  tilt axis is radial.
- The arm mount is at `(0, 0, -0.13) m`, immediately below the BODY mesh's
  `z=-0.125 m` lower bound.
- The exact C++ nominal FK and MuJoCo FK agree at home:
  `[-3.664010e-6, -9.183118e-7, 0.2777500000] m` with the base at `z=1.2 m`.
- A 100-step (0.2 s) direct hover physics test ends at `z=1.199829 m`.
- An 8-second ROS closed-loop hold run ends at
  `[-0.000045, -0.000005, 1.200202] m`; base linear speed and arm joint errors
  are on the order of `1e-6`.
- DLS recovery of the known configuration `[0.5, 0.3, -0.4]` converges in 14
  iterations to `[0.49926, 0.30136, -0.39991]` with 0.067 mm TCP residual.
- Weighted 6x9 pose DLS recovers a known translated and rotated TCP target in 6
  iterations with 0.0129 mm position and 0.000133 rad attitude residual.
- Raw-terminal keyboard input initializes from measured `/chr/state` and
  publishes cumulative XYZ plus local-roll/pitch/yaw targets to
  `/chr/target/tcp_pose`; terminal settings are restored on quit.
- An end-to-end teleop test with +10 mm X and +3 degree local roll, pitch and yaw
  converges in the pose solver without warnings. After 15 seconds the simulated
  plant has 1.996 mm TCP position error and 0.00194 rad attitude error; base
  position error is 0.351 mm and arm joint error is `6.2e-7` rad.
- Bounded attitude integral compensation reduces the suspended-arm steady-state
  base attitude error from approximately 0.0165 rad to 0.00188 rad. A separate
  12-second hold test remains at `[-0.000123, -0.000015, 1.200185] m` with
  0.000403 rad/s angular speed.
- With bias feedforward enabled, the measured arm state after 5 seconds is
  `[0.50001, 0.30170, -0.39992]`; base position remains approximately
  `[-0.00191, 0.00011, 1.20071] m`.

The GLFW warning seen on the validation host is Wayland/window-position related;
the combined model and passive viewer both initialize. Controller gains and the
DOB remain simulation starting points and require identification before hardware.
