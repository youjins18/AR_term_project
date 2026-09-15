# bringup

`simulation.launch.py` starts the MuJoCo plant, high-level CHR controller,
Palletrone flight controller, arm controller and automatic rosbag recording.

```bash
ros2 launch bringup simulation.launch.py planner_mode:=hold render:=true
```

Bags are written below `~/Desktop/ar_ws/src/data_logging/bags`. Pass
`record:=false` to disable recording for a run, or override `bag_directory` to
choose another destination.

`teleop.launch.py` starts the same stack in `dls_ik` mode and opens an interactive
keyboard commander in a separate GNOME Terminal. It inherits the same automatic
recording behavior and logging arguments:

```bash
ros2 launch bringup teleop.launch.py render:=true
```

The separate terminal is required because ROS launch subprocesses do not inherit
interactive stdin reliably.
