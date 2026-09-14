from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chr_commander', executable='keyboard_commander',
            parameters=[PathJoinSubstitution([
                FindPackageShare('chr_commander'), 'config', 'commander.yaml'])],
            output='screen')
    ])
