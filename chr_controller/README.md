# chr_controller

This is the high-level motion-planning boundary and the only component that owns
the complete desired coordinate vector. The message retains all base pose fields,
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

An unimplemented mode fails explicitly. RL should be introduced as a separate
policy adapter publishing `ChrReference` to the external input, keeping inference,
safety projection and low-level control independently testable.

`libchr_dynamics.so` contains the exact nominal MJCF kinematic frame chain,
numerical position and pose Jacobians, joint limits, position DLS and weighted
whole-body pose DLS. Dynamic model generation lives under `third_party` and is
intentionally not trusted for control until system identification is available.
