from chr_msgs.msg import ChrState
from sensor_msgs.msg import JointState

from .model_names import ARM_JOINTS
from .mujoco_plant import PlantSnapshot


def make_chr_state(snapshot: PlantSnapshot, stamp) -> ChrState:
    message = ChrState()
    message.header.stamp = stamp
    message.header.frame_id = 'world'
    message.base_pose.position.x, message.base_pose.position.y, message.base_pose.position.z = snapshot.base_position
    message.base_pose.orientation.w, message.base_pose.orientation.x, message.base_pose.orientation.y, message.base_pose.orientation.z = snapshot.base_quaternion
    message.base_twist.linear.x, message.base_twist.linear.y, message.base_twist.linear.z = snapshot.base_linear_velocity
    message.base_twist.angular.x, message.base_twist.angular.y, message.base_twist.angular.z = snapshot.base_angular_velocity
    message.joint_position = snapshot.joint_position.tolist()
    message.joint_velocity = snapshot.joint_velocity.tolist()
    message.rotor_tilt = snapshot.rotor_tilt.tolist()
    message.rotor_thrust = snapshot.rotor_thrust.tolist()
    message.joint_bias_torque = snapshot.joint_bias_torque.tolist()
    message.tcp_pose.position.x, message.tcp_pose.position.y, message.tcp_pose.position.z = snapshot.tcp_position
    message.tcp_pose.orientation.w, message.tcp_pose.orientation.x, message.tcp_pose.orientation.y, message.tcp_pose.orientation.z = snapshot.tcp_quaternion
    return message


def make_joint_state(snapshot: PlantSnapshot, stamp) -> JointState:
    message = JointState()
    message.header.stamp = stamp
    message.name = list(ARM_JOINTS)
    message.position = snapshot.joint_position.tolist()
    message.velocity = snapshot.joint_velocity.tolist()
    message.effort = snapshot.joint_bias_torque.tolist()
    return message
