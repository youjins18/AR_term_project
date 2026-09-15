"""Validated ROS actuator callbacks and command watchdogs."""

from __future__ import annotations

from rclpy.node import Node

from chr_msgs.msg import ArmCommand, PalletroneCommand

from .mujoco_plant import MuJoCoPlant


class ActuatorInterface:
    """ROS callbacks and watchdogs for the two allowed plant command channels."""

    def __init__(self, node: Node, plant: MuJoCoPlant, timeout_s: float):
        if timeout_s <= 0.0:
            raise ValueError('command timeout must be positive')
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
        try:
            self._plant.apply_flight_command(message.rotor_thrust, message.servo_angle)
        except ValueError as error:
            self._node.get_logger().warning(f'rejected flight command: {error}')
            return
        self._last_flight = self._node.get_clock().now()

    def _on_arm(self, message: ArmCommand) -> None:
        try:
            self._plant.apply_arm_command(
                message.joint_position,
                message.joint_velocity,
                message.joint_kp,
                message.joint_kd,
                message.joint_effort_feedforward,
            )
        except ValueError as error:
            self._node.get_logger().warning(f'rejected arm command: {error}')
            return
        self._last_arm = self._node.get_clock().now()

    def enforce_watchdogs(self) -> None:
        now = self._node.get_clock().now()
        if self._is_stale(now, self._last_flight):
            self._plant.stop_flight()
        if self._is_stale(now, self._last_arm):
            self._plant.hold_arm()

    def _is_stale(self, now, last_command) -> bool:
        """Return true until the first command and after the configured deadline."""
        return (
            last_command is None
            or (now - last_command).nanoseconds * 1e-9 > self._timeout_s
        )
