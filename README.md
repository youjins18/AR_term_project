# Cone Harvest Robot (CHR) simulation workspace

CHR is the Palletrone plus the cone-harvesting arm mounted below its airframe. This
workspace is a minimal but extensible ROS 2 Humble and MuJoCo control skeleton.

## Architecture

| Package | Owns |
|---|---|
| `chr_description` | composed MuJoCo model and original arm meshes |
| `chr_msgs` | stable state, reference and actuator contracts |
| `chr_mujoco` | physics stepping and the only plant input boundary |
| `chr_controller` | high-level 9-coordinate planning, FK/Jacobian/DLS-IK |
| `palletrone_flight_controller` | base-pose PID, attitude PD, DOB and allocation |
| `arm_controller` | bounded J1--J3 pass-through position command |
| `chr_commander` | intentionally blank keyboard command boundary |
| `bringup` | launch composition |

The data flow is deliberately one-way:

```text
/chr/target/* -> chr_controller -> /chr/reference
                                   |             |
                        flight controller   arm controller
                                   |             |
                    /chr/actuator/palletrone  /chr/actuator/arm
                                   \             /
                                      chr_mujoco
                                          |
                                      /chr/state
```

`ChrReference` is the authoritative high-level output. It contains base position,
base quaternion, base twist and J1--J3; conceptually this is
`[x_b,y_b,z_b,roll_b,pitch_b,yaw_b,J1,J2,J3]`.

## Build

```bash
cd ~/Desktop/ar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The Python `mujoco` package and NumPy must be available to the same Python used by
ROS 2. Native MuJoCo 3.x is not linked by the controllers.

## Run

Safe hold mode:

```bash
ros2 launch bringup simulation.launch.py planner_mode:=hold render:=true
```

DLS arm-position IK mode:

```bash
ros2 launch bringup simulation.launch.py planner_mode:=dls_ik render:=true
ros2 topic pub --once /chr/target/tcp_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.20, y: 0.10, z: 0.45}, orientation: {w: 1.0}}}"
```

Whole-body external planner/RL boundary:

```bash
ros2 launch bringup simulation.launch.py planner_mode:=external render:=true
```

Publish `chr_msgs/msg/ChrReference` on `/chr/target/whole_body`. The controller
normalizes the quaternion and clamps arm joint limits before republishing.

## Safety and extension rules

- Default mode is a stationary hover reference and zero arm joints.
- Watchdogs stop rotor torque and hold/zero arm effort when commands become stale.
- The current gains and inertia are simulation starting values, not hardware gains.
- `arm_mount` in `chr_description/mujoco/chr.xml` is the single calibration point
  for the physical attachment transform.
- RL should publish through the `external` target contract. Do not let a policy
  write MuJoCo actuators directly; keep limits and low-level control in place.
- The DLS solver controls TCP position with the arm at a fixed desired base pose.
  Whole-body IK can extend the same library with a 3x9 or 6x9 weighted Jacobian.
- `third_party/dynamics_gen_chr.py` is a reduced-order reference only. Identify
  coupled inertial parameters before enabling model-based feedforward or NMPC.
