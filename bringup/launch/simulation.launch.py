"""Launch the CHR plant and its three-layer control stack."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


RECORDED_TOPICS = (
    '/chr/state',
    '/chr/reference',
    '/chr/target/tcp_pose',
    '/chr/target/whole_body',
    '/chr/actuator/palletrone',
    '/chr/actuator/arm',
    '/chr/diagnostics/flight',
    '/chr/diagnostics/ik',
    '/joint_states',
)


def config(package: str, filename: str):
    """Return the installed path of a package-owned configuration file."""
    return PathJoinSubstitution([
        FindPackageShare(package), 'config', filename])


def start_recorder(context):
    """Create the data folder and start rosbag2 from that directory."""
    configured_path = LaunchConfiguration('bag_directory').perform(context)
    bag_directory = Path(configured_path).expanduser().resolve()
    bag_directory.mkdir(parents=True, exist_ok=True)
    return [
        LogInfo(msg=f'Recording CHR bag under {bag_directory}'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', *RECORDED_TOPICS],
            cwd=str(bag_directory),
            output='screen'),
    ]


def generate_launch_description():
    """Launch simulation, controllers, and optional automatic recording."""
    planner_mode = LaunchConfiguration('planner_mode')
    render = LaunchConfiguration('render')
    record = LaunchConfiguration('record')
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_mode', default_value='hold',
            description='CHR planner: hold, dls_ik or external'),
        DeclareLaunchArgument(
            'render', default_value='true',
            description='Open the passive MuJoCo viewer'),
        DeclareLaunchArgument(
            'record', default_value='true',
            description='Automatically record the maintained CHR topic set'),
        DeclareLaunchArgument(
            'bag_directory',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'Desktop', 'ar_ws', 'src',
                'data_logging', 'bags']),
            description='Directory that receives timestamped CHR bags'),
        Node(
            package='chr_mujoco', executable='simulator_node',
            name='chr_mujoco',
            parameters=[config('chr_mujoco', 'simulator.yaml'), {
                'render': ParameterValue(render, value_type=bool)}],
            output='screen'),
        Node(
            package='chr_controller', executable='chr_controller_node',
            parameters=[config('chr_controller', 'controller.yaml'), {
                'planner_mode': planner_mode}], output='screen'),
        Node(
            package='palletrone_flight_controller',
            executable='palletrone_flight_controller_node',
            parameters=[config(
                'palletrone_flight_controller',
                'palletrone_flight_controller.yaml')],
            output='screen'),
        Node(
            package='arm_controller', executable='arm_controller_node',
            parameters=[config('arm_controller', 'arm_controller.yaml')],
            output='screen'),
        OpaqueFunction(
            function=start_recorder,
            condition=IfCondition(record)),
    ])
