# chr_mujoco

`MuJoCoPlant` owns model IDs and physics;
`ActuatorInterface` is the only write path into `data.ctrl`; `StateInterface`
converts simulation state into ROS messages; `simulator_node` schedules them. This split
keeps the simulation replaceable by hardware without changing either controller.

The node publishes `/chr/state` and `/joint_states`, subscribes to the two actuator
topics, and optionally renders through MuJoCo's passive viewer. A watchdog zeros
rotor and arm torque if low-level commands stop arriving.
