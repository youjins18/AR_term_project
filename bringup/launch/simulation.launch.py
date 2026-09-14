from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def config(package: str, filename: str):
    return PathJoinSubstitution([FindPackageShare(package), 'config', filename])


def generate_launch_description():
    planner_mode = LaunchConfiguration('planner_mode')
    render = LaunchConfiguration('render')
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_mode', default_value='hold',
            description='CHR planner: hold, dls_ik or external'),
        DeclareLaunchArgument(
            'render', default_value='true',
            description='Open the passive MuJoCo viewer'),
        Node(
            package='chr_mujoco', executable='simulator_node', name='chr_mujoco',
            parameters=[config('chr_mujoco', 'simulator.yaml'), {
                'render': ParameterValue(render, value_type=bool)}], output='screen'),
        Node(
            package='chr_controller', executable='chr_controller_node',
            parameters=[config('chr_controller', 'controller.yaml'), {
                'planner_mode': planner_mode}], output='screen'),
        Node(
            package='palletrone_flight_controller',
            executable='palletrone_flight_controller_node',
            parameters=[config(
                'palletrone_flight_controller', 'palletrone_flight_controller.yaml')],
            output='screen'),
        Node(
            package='arm_controller', executable='arm_controller_node',
            parameters=[config('arm_controller', 'arm_controller.yaml')], output='screen'),
    ])
