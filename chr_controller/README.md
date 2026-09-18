# chr_controller

This node owns the complete desired coordinate vector. The message retains all
base pose fields,
but roll and pitch are constrained to zero, leaving seven independent coordinates:

`[x_b, y_b, z_b, yaw_b, J1, J2, J3]`, with `roll_b = pitch_b = 0`.

It publishes that vector as `/chr/reference` (`ChrReference`, quaternion attitude)
so both low-level controllers share one timestamp and source.

## Planner modes

- `hold` (default): publishes the configured base pose and arm home.
- `dls_ik`: solves bounded weighted whole-body pose IK for
  `/chr/target/tcp_pose` in the `world` frame. Its 6x7 numerical Jacobian uses
  base translation, base yaw and J1--J3. Coordinate scales prefer arm motion while
  allowing the Palletrone to translate or yaw when joint limits require it.
- `external`: validates and relays `/chr/target/whole_body`; this is the stable
  insertion point for RL, NMPC or an offline motion planner. Base roll/pitch and
  x/y angular-velocity inputs are projected to zero before publication.

Unknown modes are rejected during startup. An RL policy can use the `external`
input without bypassing the controller constraints or the low-level controllers.

`/chr/diagnostics/ik` publishes the latest DLS target status, convergence flag,
iteration count, position/orientation residual, damping, minimum singular value
and condition number for MATLAB analysis.

`src/chr_dynamics_library.cpp` implements the CHR kinematics, joint-limit
handling and weighted pose DLS-IK. Despite the filename, it does not implement a
whole-body dynamics model.
