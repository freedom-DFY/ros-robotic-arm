"""Hardware bringup for six arm joints and an independent gripper."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from arm6_ros2.config import load_parameters


def launch_nodes(context):
    parameters = load_parameters(LaunchConfiguration('config').perform(context))
    parameters.update({
        'device': LaunchConfiguration('device').perform(context),
        'baud_rate': int(LaunchConfiguration('baud_rate').perform(context)),
    })
    return [Node(package='arm6_ros2', executable='arm6_node', name='arm6',
                 output='screen', parameters=[parameters])]


def generate_launch_description():
    share = Path(get_package_share_directory('arm6_ros2'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share/'config/arm6.yaml')),
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        OpaqueFunction(function=launch_nodes),
    ])
