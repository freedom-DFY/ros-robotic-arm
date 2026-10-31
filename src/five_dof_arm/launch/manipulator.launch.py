"""ROS 2 bringup; dry_run permits testing without a serial device."""
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
    # Merge explicitly: ROS gives named YAML sections priority over /** sections,
    # so passing a later anonymous launch dictionary alone cannot override them.
    config_path = LaunchConfiguration('config').perform(context)
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}

    def parameters_for(name):
        result = dict(config.get('/**', {}).get('ros__parameters', {}))
        for key in (name, '/' + name):
            result.update(config.get(key, {}).get('ros__parameters', {}))
        return result

    driver_params = parameters_for('sync_read_write_node')
    driver_params.update({
        'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
        'device_name': LaunchConfiguration('device_name'),
        'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int),
    })
    controller_params = parameters_for('logicool_controller')
    controller_params['joy_layout'] = LaunchConfiguration('joy_layout')
    driver = Node(
        package='five_dof_arm', executable='sync_read_write_node',
        name='sync_read_write_node', output='screen', parameters=[driver_params],
    )
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=driver, on_exit=[EmitEvent(event=Shutdown(reason='Arm driver exited'))])),
        driver,
        Node(package='five_dof_arm', executable='logicool_controller',
             name='logicool_controller', output='screen', parameters=[controller_params]),
        Node(package='joy', executable='game_controller_node', output='screen',
             condition=IfCondition(LaunchConfiguration('use_joy')),
             parameters=[{
                 'device_id': ParameterValue(LaunchConfiguration('joy_device_id'), value_type=int),
                 'deadzone': 0.1, 'autorepeat_rate': 20.0,
             }]),
    ]


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('five_dof_arm')) / 'config' / 'arm.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('device_name', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('use_joy', default_value='false'),
        DeclareLaunchArgument('joy_layout', default_value='sdl'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        OpaqueFunction(function=launch_nodes),
    ])
