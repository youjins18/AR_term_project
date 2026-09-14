from __future__ import annotations

from rclpy.node import Node

from chr_msgs.msg import ArmCommand, PalletroneCommand

from .mujoco_plant import MuJoCoPlant


class ActuatorInterface:
    """ROS callbacks and watchdogs for the two allowed plant command channels."""

    def __init__(self, node: Node, plant: MuJoCoPlant, timeout_s: float):
        self._node = node
        self._plant = plant
        self._timeout_s = timeout_s
        self._last_flight = None
        self._last_arm = None
        self._flight_sub = node.create_subscription(
            PalletroneCommand, '/chr/actuator/palletrone', self._on_flight, 1)
        self._arm_sub = node.create_subscription(
            ArmCommand, '/chr/actuator/arm', self._on_arm, 1)

    def _on_flight(self, message: PalletroneCommand) -> None:
        self._plant.apply_flight_command(message.rotor_thrust, message.servo_angle)
        self._last_flight = self._node.get_clock().now()

    def _on_arm(self, message: ArmCommand) -> None:
        self._plant.apply_arm_command(
            message.joint_position,
            message.joint_velocity,
            message.joint_kp,
            message.joint_kd,
            message.joint_effort_feedforward,
        )
        self._last_arm = self._node.get_clock().now()

    def enforce_watchdogs(self) -> None:
        now = self._node.get_clock().now()
        if self._last_flight is None or (now - self._last_flight).nanoseconds * 1e-9 > self._timeout_s:
            self._plant.stop_flight()
        if self._last_arm is None or (now - self._last_arm).nanoseconds * 1e-9 > self._timeout_s:
            self._plant.hold_arm()
