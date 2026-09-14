# bringup

`simulation.launch.py` starts the MuJoCo plant, high-level CHR controller,
Palletrone flight controller and arm controller. The keyboard commander is kept in
the separate `teleop.launch.py` because it is intentionally blank at this stage.

```bash
ros2 launch bringup simulation.launch.py planner_mode:=hold render:=true
```
