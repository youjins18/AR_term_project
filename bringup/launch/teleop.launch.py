"""Launch CHR pose IK and keyboard input in a dedicated terminal."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch DLS-IK simulation and keyboard input in a new terminal."""
    render = LaunchConfiguration('render')
    record = LaunchConfiguration('record')
    bag_directory = LaunchConfiguration('bag_directory')
    return LaunchDescription([
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(), 'simulation.launch.py'])),
            launch_arguments={
                'planner_mode': 'dls_ik',
                'render': render,
                'record': record,
                'bag_directory': bag_directory,
            }.items()),
        Node(
            package='chr_commander', executable='keyboard_commander',
            parameters=[PathJoinSubstitution([
                FindPackageShare('chr_commander'), 'config',
                'commander.yaml'])],
            prefix='gnome-terminal --wait --',
            output='screen')
    ])
