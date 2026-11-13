#!/usr/bin/env python3
"""Launch and exercise ROS 2 topics/services with simulated motor feedback only."""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '78')
os.environ.setdefault('ROS_LOG_DIR', '/tmp/arm3_ros2_smoke_logs')

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import SetBool, Trigger
import yaml


def main():
    config_path = Path(__file__).resolve().parents[1] / 'config/arm3.yaml'
    config = yaml.safe_load(config_path.read_text())
    # Named sections normally have higher precedence in ROS. Test that launch
    # arguments still override them, and never attempt serial I/O in this test.
    config['arm3'] = {'ros__parameters': {
        'dry_run': False, 'device': '/invalid/from_yaml', 'baud_rate': 1000000,
    }}
    with tempfile.TemporaryDirectory(prefix='arm3_smoke_') as temp_dir:
        custom_config = Path(temp_dir) / 'named_parameters.yaml'
        custom_config.write_text(yaml.safe_dump(config))
        with tempfile.TemporaryFile(mode='w+') as output:
            process = subprocess.Popen([
                'ros2', 'launch', 'arm3_ros2', 'arm3.launch.py', 'dry_run:=true',
                'device:=/invalid/from_launch', 'baud_rate:=57600',
                'config:=' + str(custom_config),
            ], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('arm3_smoke_test')
            states, points, ticks = [], [], []
            subscriptions = [
                node.create_subscription(JointState, '/arm3/joint_states', states.append, 10),
                node.create_subscription(PointStamped, '/arm3/wrist_point', points.append, 10),
                node.create_subscription(Int32MultiArray, '/arm3/encoder_ticks', ticks.append, 10),
            ]
            joint = node.create_publisher(JointState, '/arm3/joint_target', 1)
            cart = node.create_publisher(PointStamped, '/arm3/cartesian_target', 1)
            legacy = node.create_publisher(Point, '/arm3/auto_transform', 1)
            joy = node.create_publisher(Joy, '/joy', qos_profile_sensor_data)
            enable = node.create_client(SetBool, '/arm3/enable')
            stop = node.create_client(Trigger, '/arm3/stop')
            parameters = node.create_client(GetParameters, '/arm3/get_parameters')

            def wait_until(predicate, timeout=10.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if predicate():
                        return
                    if process.poll() is not None:
                        raise AssertionError('Launch exited unexpectedly')
                raise AssertionError('Timed out waiting for ROS 2 data')

            def spin_for(seconds=0.25):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)

            def call(client, request):
                future = client.call_async(request)
                wait_until(future.done)
                return future.result()

            def point_values():
                p = points[-1].point
                return (p.x, p.y, p.z)

            def send_joy(button=None):
                buttons = [0] * 9
                if button is not None:
                    buttons[button] = 1
                joy.publish(Joy(axes=[0.0] * 6, buttons=buttons))
                spin_for()

            try:
                wait_until(lambda: states and points and ticks and enable.service_is_ready()
                           and stop.service_is_ready() and parameters.service_is_ready())
                wait_until(lambda: all(pub.get_subscription_count() > 0
                                       for pub in (joint, cart, legacy, joy)))
                result = call(parameters, GetParameters.Request(names=['dry_run', 'device', 'baud_rate']))
                assert result.values[0].bool_value
                assert result.values[1].string_value == '/invalid/from_launch'
                assert result.values[2].integer_value == 57600
                assert list(ticks[-1].data) == [2048, 2160, 1910]
                assert list(states[-1].name) == ['joint_1', 'joint_2', 'joint_3']
                assert points[-1].header.frame_id == 'base_link'

                target = (0.02, 0.18, -0.20)
                joint.publish(JointState(name=['joint_3', 'joint_1', 'joint_2'],
                                         position=[target[2], target[0], target[1]]))
                wait_until(lambda: math.dist(states[-1].position, target) < 0.002)
                spin_for()
                held = tuple(states[-1].position)
                for invalid in ([99.0, 0.0, 0.0], [float('nan'), 0.0, 0.0], [0.0]):
                    joint.publish(JointState(position=invalid))
                    spin_for()
                    assert tuple(states[-1].position) == held

                cart.publish(PointStamped(point=Point(x=0.01, y=0.30, z=0.15)))
                spin_for()
                assert tuple(states[-1].position) == held, 'Wrong frame was accepted'
                msg = PointStamped(point=Point(x=0.01, y=0.30, z=0.15))
                msg.header.frame_id = 'base_link'
                cart.publish(msg)
                wait_until(lambda: math.dist(point_values(), (0.01, 0.30, 0.15)) < 0.001)
                spin_for()
                legacy.publish(Point(x=12.0, y=300.0, z=150.0))
                wait_until(lambda: math.dist(point_values(), (0.012, 0.30, 0.15)) < 0.001)
                spin_for()

                send_joy()
                before = tuple(ticks[-1].data)
                send_joy(7)  # RT, J1 +16 ticks, best-effort QoS.
                wait_until(lambda: ticks[-1].data[0] == before[0] + 16)
                send_joy(7)
                assert ticks[-1].data[0] == before[0] + 16, 'Held button repeated a step'
                send_joy()
                send_joy(8)  # BACK to Cartesian mode.
                send_joy()
                before_point = point_values()
                send_joy(3)  # Y, Z +10 mm.
                expected = (before_point[0], before_point[1], before_point[2] + 0.01)
                wait_until(lambda: math.dist(point_values(), expected) < 0.001)
                spin_for()

                joint.publish(JointState(position=[0.4, 0.18, -0.20]))
                spin_for(0.1)
                assert call(stop, Trigger.Request()).success
                spin_for()
                held = tuple(states[-1].position)
                spin_for()
                assert tuple(states[-1].position) == held
                assert abs(held[0] - 0.4) > 0.01, 'Stop did not interrupt the trajectory'
                assert call(enable, SetBool.Request(data=False)).success
                joint.publish(JointState(position=[0.0, 0.0, 0.0]))
                spin_for()
                assert tuple(states[-1].position) == held
                assert call(enable, SetBool.Request(data=True)).success
                assert subscriptions  # Keep subscriptions alive until cleanup.
                print('PASS: launch overrides, encoder/radian/XYZ feedback, named joints, '
                      'invalid-target rejection, Cartesian path, legacy mm, best-effort Joy, '
                      'mode switch, stop and disable/enable')
            except BaseException:
                output.seek(0)
                print(output.read())
                raise
            finally:
                node.destroy_node()
                rclpy.try_shutdown()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
            assert process.returncode == 0, f'Launch failed to stop cleanly: {process.returncode}'
            print('PASS: clean Ctrl+C shutdown')


if __name__ == '__main__':
    main()
