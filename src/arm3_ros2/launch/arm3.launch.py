"""ROS 2 three-axis bringup. Real hardware starts in read-only mode."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def launch_nodes(context):
    with open(LaunchConfiguration('config').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    # Merge node sections explicitly, so launch arguments always take priority.
    parameters = {}
    for name in ('/**', 'arm3', '/arm3'):
        parameters.update(config.get(name, {}).get('ros__parameters', {}))
    if not parameters:
        raise ValueError('Expected a ROS 2 YAML section: /**: ros__parameters: ...')
    parameters.update({
        'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
        'device': ParameterValue(LaunchConfiguration('device'), value_type=str),
        'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int),
    })
    controller = Node(
        package='arm3_ros2', executable='arm3_node', name='arm3',
        output='screen', parameters=[parameters],
    )
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason='Three-axis controller exited'))],
        )),
        controller,
        # Keep the ROS 1 Linux/DirectInput mapping, including /dev/input/js0.
        Node(
            package='joy_linux', executable='joy_linux_node', name='arm3_joy',
            output='screen', condition=IfCondition(LaunchConfiguration('use_joy')),
            parameters=[{
                'dev': ParameterValue(LaunchConfiguration('joy_device'), value_type=str),
                'deadzone': 0.1, 'autorepeat_rate': 20.0,
            }],
        ),
    ]


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('arm3_ros2')) / 'config/arm3.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('use_joy', default_value='false'),
        DeclareLaunchArgument('joy_device', default_value='/dev/input/js0'),
        OpaqueFunction(function=launch_nodes),
    ])
