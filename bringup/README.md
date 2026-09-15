# bringup

`simulation.launch.py` starts the MuJoCo plant, high-level CHR controller,
Palletrone flight controller and arm controller.

```bash
ros2 launch bringup simulation.launch.py planner_mode:=hold render:=true
```

`teleop.launch.py` starts the same stack in `dls_ik` mode and opens an interactive
keyboard commander in a separate GNOME Terminal:

```bash
ros2 launch bringup teleop.launch.py render:=true
```

The separate terminal is required because ROS launch subprocesses do not inherit
interactive stdin reliably.
