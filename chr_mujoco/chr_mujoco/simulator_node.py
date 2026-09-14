from __future__ import annotations

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from chr_msgs.msg import ChrState

from .actuator_interface import ActuatorInterface
from .mujoco_plant import MuJoCoPlant
from .state_interface import make_chr_state, make_joint_state


class SimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__('chr_mujoco')
        physics_hz = self.declare_parameter('physics_hz', 500.0).value
        publish_hz = self.declare_parameter('publish_hz', 250.0).value
        realtime_factor = self.declare_parameter('realtime_factor', 1.0).value
        render = self.declare_parameter('render', True).value
        timeout_s = self.declare_parameter('command_timeout_s', 0.2).value
        initial_base = self.declare_parameter('initial_base_position', [0.0, 0.0, 1.2]).value
        initial_joints = self.declare_parameter('initial_joint_position', [0.0, 0.0, 0.0]).value

        model_path = os.path.join(
            get_package_share_directory('chr_description'), 'mujoco', 'scene.xml')
        self._plant = MuJoCoPlant(model_path, 1.0 / physics_hz, initial_base, initial_joints)
        self._actuators = ActuatorInterface(self, self._plant, timeout_s)
        self._state_pub = self.create_publisher(ChrState, '/chr/state', 1)
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 1)
        self._marker_sub = self.create_subscription(
            PoseStamped, '/chr/target/tcp_pose', self._on_target_pose, 1)
        self._publish_divisor = max(1, round(physics_hz / publish_hz))
        self._tick = 0
        self._viewer = None

        if render:
            try:
                import mujoco.viewer
                self._viewer = mujoco.viewer.launch_passive(self._plant.model, self._plant.data)
            except Exception as error:  # Rendering is optional in headless operation.
                self.get_logger().warning(f'viewer disabled: {error}')

        wall_hz = physics_hz * max(float(realtime_factor), 1e-6)
        self._timer = self.create_timer(1.0 / wall_hz, self._step)
        self.get_logger().info(
            f'CHR model loaded: nq={self._plant.model.nq}, nv={self._plant.model.nv}, '
            f'nu={self._plant.model.nu}, physics_hz={physics_hz:.1f}')

    def _on_target_pose(self, message: PoseStamped) -> None:
        p = message.pose.position
        q = message.pose.orientation
        self._plant.set_target_marker((p.x, p.y, p.z), (q.w, q.x, q.y, q.z))

    def _step(self) -> None:
        self._actuators.enforce_watchdogs()
        self._plant.step()
        if self._viewer is not None:
            if self._viewer.is_running():
                self._viewer.sync()
            else:
                self._viewer = None
        self._tick += 1
        if self._tick % self._publish_divisor:
            return
        snapshot = self._plant.snapshot()
        stamp = self.get_clock().now().to_msg()
        self._state_pub.publish(make_chr_state(snapshot, stamp))
        self._joint_pub.publish(make_joint_state(snapshot, stamp))

    def destroy_node(self):
        if self._viewer is not None:
            self._viewer.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # ros2 launch may deliver a second SIGINT while cleanup is in progress.
            pass
