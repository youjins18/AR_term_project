"""ROS-independent MuJoCo plant state and actuator access."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import mujoco
import numpy as np

from .model_names import (
    ACTUATORS,
    ARM_JOINTS,
    BASE_JOINT,
    ROTOR_TILT_JOINTS,
    TARGET_MARKER_BODY,
    TCP_SITE,
)


@dataclass(frozen=True)
class PlantSnapshot:
    time: float
    base_position: np.ndarray
    base_quaternion: np.ndarray
    base_linear_velocity: np.ndarray
    base_angular_velocity: np.ndarray
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    rotor_tilt: np.ndarray
    rotor_thrust: np.ndarray
    joint_bias_torque: np.ndarray
    tcp_position: np.ndarray
    tcp_quaternion: np.ndarray


class MuJoCoPlant:
    """Owns MuJoCo state; contains no ROS-specific behavior."""

    def __init__(self, model_path: str, timestep: float, initial_base_position: Iterable[float],
                 initial_joint_position: Iterable[float]):
        if not np.isfinite(timestep) or timestep <= 0.0:
            raise ValueError('MuJoCo timestep must be finite and positive')
        base_position = self._vector(initial_base_position, 3, 'initial base position')
        joint_position = self._vector(initial_joint_position, len(ARM_JOINTS),
                                      'initial joint position')
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = float(timestep)

        self._base_id = self._id(mujoco.mjtObj.mjOBJ_JOINT, BASE_JOINT)
        self._joint_ids = {name: self._id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in ARM_JOINTS}
        self._tilt_ids = {
            name: self._id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in ROTOR_TILT_JOINTS
        }
        self._actuator_ids = {
            name: self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for group in ACTUATORS.values() for name in group
        }
        self._tcp_id = self._id(mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
        marker_id = self._id(mujoco.mjtObj.mjOBJ_BODY, TARGET_MARKER_BODY)
        self._marker_mocap_id = int(self.model.body_mocapid[marker_id])
        self._joint_qpos_addresses = np.asarray(
            [self._qpos_address(name) for name in ARM_JOINTS], dtype=int)
        self._joint_qvel_addresses = np.asarray(
            [self._qvel_address(name) for name in ARM_JOINTS], dtype=int)
        self._tilt_qpos_addresses = np.asarray([
            self.model.jnt_qposadr[joint_id] for joint_id in self._tilt_ids.values()
        ], dtype=int)
        self._rotor_actuator_ids = np.asarray([
            self._actuator_ids[name] for name in ACTUATORS['rotors']
        ], dtype=int)

        base_qpos = int(self.model.jnt_qposadr[self._base_id])
        self.data.qpos[base_qpos:base_qpos + 3] = base_position
        self.data.qpos[base_qpos + 3:base_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qpos[self._joint_qpos_addresses] = joint_position
        mujoco.mj_forward(self.model, self.data)

    @staticmethod
    def _vector(values: Iterable[float], size: int, label: str) -> np.ndarray:
        """Convert a command vector without silently truncating malformed input."""
        vector = np.asarray(tuple(values), dtype=float)
        if vector.shape != (size,):
            raise ValueError(f'{label} must contain exactly {size} values')
        if not np.all(np.isfinite(vector)):
            raise ValueError(f'{label} must contain only finite values')
        return vector

    def _id(self, object_type, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f'MuJoCo model is missing required name: {name}')
        return int(object_id)

    def _qpos_address(self, joint_name: str) -> int:
        return int(self.model.jnt_qposadr[self._joint_ids[joint_name]])

    def _qvel_address(self, joint_name: str) -> int:
        return int(self.model.jnt_dofadr[self._joint_ids[joint_name]])

    def _clip_and_set(self, actuator_names: Iterable[str], values: Iterable[float]) -> None:
        names = tuple(actuator_names)
        command = self._vector(values, len(names), 'actuator command')
        for name, value in zip(names, command):
            actuator_id = self._actuator_ids[name]
            if self.model.actuator_ctrllimited[actuator_id]:
                low, high = self.model.actuator_ctrlrange[actuator_id]
                value = np.clip(value, low, high)
            self.data.ctrl[actuator_id] = float(value)

    def apply_flight_command(
            self, rotor_thrust: Iterable[float], servo_angle: Iterable[float]) -> None:
        self._clip_and_set(ACTUATORS['rotors'], rotor_thrust)
        self._clip_and_set(ACTUATORS['rotor_servos'], servo_angle)

    def apply_arm_command(self, joint_position: Iterable[float], joint_velocity: Iterable[float],
                          joint_kp: Iterable[float], joint_kd: Iterable[float],
                          effort_feedforward: Iterable[float]) -> None:
        joint_count = len(ARM_JOINTS)
        q_des = self._vector(joint_position, joint_count, 'desired joint position')
        dq_des = self._vector(joint_velocity, joint_count, 'desired joint velocity')
        kp = self._vector(joint_kp, joint_count, 'joint kp')
        kd = self._vector(joint_kd, joint_count, 'joint kd')
        feedforward = self._vector(effort_feedforward, joint_count, 'joint feedforward')
        q = self.data.qpos[self._joint_qpos_addresses]
        dq = self.data.qvel[self._joint_qvel_addresses]
        effort = kp * (q_des - q) + kd * (dq_des - dq) + feedforward
        self._clip_and_set(ACTUATORS['arm'], effort)

    def stop_flight(self) -> None:
        self._clip_and_set(ACTUATORS['rotors'], np.zeros(len(ACTUATORS['rotors'])))

    def hold_arm(self) -> None:
        self._clip_and_set(ACTUATORS['arm'], np.zeros(len(ACTUATORS['arm'])))

    def set_target_marker(
            self, position: Iterable[float], quaternion_wxyz: Iterable[float]) -> None:
        if self._marker_mocap_id < 0:
            return
        marker_position = self._vector(position, 3, 'target marker position')
        quaternion = self._vector(quaternion_wxyz, 4, 'target marker quaternion')
        norm = np.linalg.norm(quaternion)
        if norm <= 1e-12:
            raise ValueError('target marker quaternion must have non-zero norm')
        self.data.mocap_pos[self._marker_mocap_id] = marker_position
        self.data.mocap_quat[self._marker_mocap_id] = quaternion / norm

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def snapshot(self) -> PlantSnapshot:
        base_qpos = int(self.model.jnt_qposadr[self._base_id])
        base_qvel = int(self.model.jnt_dofadr[self._base_id])
        tcp_quaternion = np.empty(4)
        mujoco.mju_mat2Quat(tcp_quaternion, self.data.site_xmat[self._tcp_id])
        return PlantSnapshot(
            time=float(self.data.time),
            base_position=self.data.qpos[base_qpos:base_qpos + 3].copy(),
            base_quaternion=self.data.qpos[base_qpos + 3:base_qpos + 7].copy(),
            base_linear_velocity=self.data.qvel[base_qvel:base_qvel + 3].copy(),
            base_angular_velocity=self.data.qvel[base_qvel + 3:base_qvel + 6].copy(),
            joint_position=self.data.qpos[self._joint_qpos_addresses].copy(),
            joint_velocity=self.data.qvel[self._joint_qvel_addresses].copy(),
            rotor_tilt=self.data.qpos[self._tilt_qpos_addresses].copy(),
            rotor_thrust=self.data.ctrl[self._rotor_actuator_ids].copy(),
            joint_bias_torque=self.data.qfrc_bias[self._joint_qvel_addresses].copy(),
            tcp_position=self.data.site_xpos[self._tcp_id].copy(),
            tcp_quaternion=tcp_quaternion,
        )
