from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    render = LaunchConfiguration('render')
    return LaunchDescription([
        DeclareLaunchArgument(
            'render', default_value='true',
            description='Open the passive MuJoCo viewer'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ThisLaunchFileDir(), 'simulation.launch.py'])),
            launch_arguments={'planner_mode': 'dls_ik', 'render': render}.items()),
        Node(
            package='chr_commander', executable='keyboard_commander',
            parameters=[PathJoinSubstitution([
                FindPackageShare('chr_commander'), 'config', 'commander.yaml'])],
            prefix='gnome-terminal --wait --',
            output='screen')
    ])
