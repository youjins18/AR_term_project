# 🌲 Cone Harvest Robot (CHR) Simulation

ROS 2 Humble and MuJoCo simulation of a Palletrone carrying a three-joint
cone-harvesting arm.

## Architecture

| Package | Owns |
|---|---|
| `chr_description` | composed MuJoCo model and original arm meshes |
| `chr_msgs` | state, reference and actuator messages |
| `chr_mujoco` | physics stepping and the only plant input boundary |
| `chr_controller` | constrained whole-body planning, FK/Jacobian/DLS-IK |
| `palletrone_flight_controller` | base-pose PID, attitude PD, DOB and allocation |
| `arm_controller` | bounded J1--J3 pass-through position command |
| `chr_commander` | interactive TCP position and attitude keyboard commands |
| `bringup` | launch composition |

Controllers write to the plant only through the two actuator topics:

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

`ChrReference` is the common high-level command. It contains base position,
base quaternion, base twist and J1--J3. Palletrone roll and pitch commands are
always zero, so the independent coordinates are
`[x_b,y_b,z_b,yaw_b,J1,J2,J3]`.

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

Hold mode:

```bash
ros2 launch bringup simulation.launch.py planner_mode:=hold render:=true
```

DLS whole-body TCP pose IK mode:

```bash
ros2 launch bringup simulation.launch.py planner_mode:=dls_ik render:=true
ros2 topic pub --once /chr/target/tcp_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.20, y: 0.10, z: 0.45}, orientation: {w: 1.0}}}"
```

Interactive keyboard teleop (starts the full stack in `dls_ik` mode and opens a
dedicated terminal):

```bash
ros2 launch bringup teleop.launch.py render:=true
```

Keys are `W/S` X, `A/D` Y, `R/F` Z, `I/K` roll, `J/L` pitch and `U/O` yaw.
Position increments are expressed in the world frame; attitude increments use
the current TCP-local axes. Press `Space` to hold the measured pose, `0` for the
startup pose, and `Q` or `Esc` to quit.

External planner or RL input:

```bash
ros2 launch bringup simulation.launch.py planner_mode:=external render:=true
```

Publish `chr_msgs/msg/ChrReference` on `/chr/target/whole_body`. The controller
normalizes the quaternion and clamps arm joint limits before republishing.

## Data logging

Simulation launches record the control and diagnostic topics by default. See
[`data_logging/README.md`](data_logging/README.md) for bag conversion and MATLAB
plots. Set `record:=false` to run without recording.

## Design notes

- Default mode is a stationary hover reference and zero arm joints.
- The arm controller holds the measured joint pose if its reference expires.
  The simulator zeros actuator effort if low-level commands expire.
- The current gains and inertia are simulation starting values, not hardware gains.
- `arm_mount` in `chr_description/mujoco/chr.xml` is the physical attachment
  transform. Mirror any change in `ChrKinematics::tcp_in_world` so DLS-IK keeps
  the same nominal frame chain.
- RL policies should publish through the `external` target contract so that
  joint limits and low-level control remain active.
- The pose DLS solver owns all seven independent coordinates and uses a weighted
  6x7 Jacobian. Tune its coordinate scales before aggressive flight motion.
