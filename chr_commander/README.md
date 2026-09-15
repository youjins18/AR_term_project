# chr_commander

Interactive keyboard control of the desired TCP pose. The node waits for
`/chr/state`, initializes its target from the measured TCP pose, and publishes
incremental `geometry_msgs/PoseStamped` commands on `/chr/target/tcp_pose`.

## Key map

| Keys | Command |
|---|---|
| `W` / `S` | world +X / -X |
| `A` / `D` | world +Y / -Y |
| `R` / `F` | world +Z / -Z |
| `I` / `K` | local TCP +roll / -roll |
| `J` / `L` | local TCP +pitch / -pitch |
| `U` / `O` | local TCP +yaw / -yaw |
| `Space` | replace the target with the currently measured TCP pose |
| `0` | restore the TCP pose measured at startup |
| `P`, `?` | print target / help |
| `Q`, `Esc` | quit and restore terminal settings |

Default increments are 10 mm and 3 degrees. Workspace bounds and step sizes are
configured in `config/commander.yaml`. Raw terminal input is kept out of both
controllers; run the executable in a real terminal.

The complete launch opens the keyboard node in a GNOME Terminal:

```bash
ros2 launch bringup teleop.launch.py render:=true
```

When the simulation is already running in `dls_ik` mode, use:

```bash
ros2 run chr_commander keyboard_commander \
  --ros-args --params-file $(ros2 pkg prefix chr_commander)/share/chr_commander/config/commander.yaml
```
