#!/usr/bin/env python3
"""Exercise ROS 2 launch, topics and services without serial I/O.

Run after sourcing install/setup.bash. Uses an isolated ROS domain and closes
all launched nodes when the test finishes.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '73')
os.environ.setdefault('ROS_LOG_DIR', '/tmp/five_dof_arm_ros_logs')

import rclpy
from dynamixel_sdk_custom_interfaces.msg import SyncSetPosition
from dynamixel_sdk_custom_interfaces.srv import SyncGetPosition
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy


def main():
    with tempfile.TemporaryFile(mode='w+') as output:
        process = subprocess.Popen(
            ['ros2', 'launch', 'five_dof_arm', 'manipulator.launch.py', 'dry_run:=true'],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
        )
        rclpy.init()
        node = rclpy.create_node('arm_smoke_test')
        states, points, commands = [], [], []
        subscriptions = [
            node.create_subscription(JointState, 'joint_states', states.append, 10),
            node.create_subscription(Point, 'end_effector_position', points.append, 10),
            node.create_subscription(SyncSetPosition, 'sync_set_position', commands.append, 10),
        ]
        raw = node.create_publisher(SyncSetPosition, 'sync_set_position', 10)
        joy = node.create_publisher(Joy, 'joy', qos_profile_sensor_data)
        target = node.create_publisher(Point, 'auto_transform', 10)
        client = node.create_client(SyncGetPosition, 'sync_get_position')

        def wait_until(predicate, timeout=10):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if predicate():
                    return
                if process.poll() is not None:
                    raise AssertionError('Bringup process exited unexpectedly')
            raise AssertionError('Timed out waiting for ROS data')

        def spin_for(seconds=0.25):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.02)

        def read(ids=(1, 2, 3, 4, 5)):
            req = SyncGetPosition.Request()
            for i, value in enumerate(ids, 1):
                setattr(req, f'id{i}', value)
            future = client.call_async(req)
            wait_until(future.done)
            return future.result()

        def values(response):
            return [getattr(response, f'position{i}') for i in range(1, 6)]

        def command(positions, ids=(1, 2, 3, 4, 5)):
            msg = SyncSetPosition()
            for i, (motor_id, position) in enumerate(zip(ids, positions), 1):
                setattr(msg, f'id{i}', motor_id)
                setattr(msg, f'position{i}', position)
            raw.publish(msg)
            spin_for()

        def send_joy(buttons=None, axes=None):
            msg = Joy()
            msg.axes = axes if axes is not None else [0.0] * 6
            msg.buttons = buttons if buttons is not None else [0] * 15
            joy.publish(msg)
            spin_for()

        try:
            wait_until(lambda: states and points and client.service_is_ready())
            wait_until(lambda: joy.get_subscription_count() > 0 and target.get_subscription_count() > 0)
            response = read()
            assert response.success and response.message == 'dry_run'
            assert values(response) == [2048, 2160, 1910, 1360, 2048]
            command([2050, 2162, 1912, 1362, 2052])
            assert values(read()) == [2050, 2162, 1912, 1362, 2052]
            assert values(read((5, 4, 3, 2, 1))) == [2052, 1362, 1912, 2162, 2050]
            assert not read((1, 1, 3, 4, 5)).success
            command([4096, 2162, 1912, 1362, 2052])
            assert values(read())[0] == 2050, 'Out-of-range command was accepted'
            command([2048] * 5, (1, 1, 3, 4, 5))
            assert values(read())[0] == 2050, 'Duplicate IDs were accepted'
            command([2052, 1362, 1912, 2162, 2050], (5, 4, 3, 2, 1))
            assert values(read()) == [2050, 2162, 1912, 1362, 2052]

            # Empty/malformed Joy must not crash or issue a motor command.
            count = len(commands)
            send_joy([], [])
            assert len(commands) == count
            buttons = [0] * 15
            buttons[3] = 1  # Y: original initial preset.
            send_joy(buttons)
            assert values(read()) == [2048, 2160, 1910, 1360, 2048]
            buttons = [0] * 15
            buttons[2] = 1  # X: joint 5 +16 ticks.
            send_joy(buttons)
            assert values(read())[-1] == 2064
            # SDL triggers are 0 at rest, negative when pressed.
            send_joy(axes=[0.0, 0.0, 0.0, 0.0, -1.0, 0.0])
            assert values(read())[0] == 2032

            # Holding BACK across repeated messages toggles exactly once.
            buttons = [0] * 15
            buttons[4] = 1
            send_joy(buttons)
            send_joy(buttons)
            buttons = [0] * 15
            buttons[9] = 1  # LB in task mode: initial preset.
            send_joy(buttons)
            assert values(read()) == [2048, 2160, 1910, 1360, 2048]

            spin_for()
            previous = values(read())
            point = Point(x=points[-1].x + 10.0, y=points[-1].y, z=points[-1].z)
            target.publish(point)
            wait_until(lambda: values(read()) != previous)
            spin_for()
            error = math.dist((point.x, point.y, point.z),
                              (points[-1].x, points[-1].y, points[-1].z))
            assert error < 1.0, f'Cartesian position error {error} mm'
            previous = values(read())
            target.publish(Point(x=10000.0, y=10000.0, z=10000.0))
            spin_for()
            assert values(read()) == previous, 'Unreachable target was accepted'
            target.publish(Point(x=float('nan'), y=0.0, z=0.0))
            spin_for()
            assert values(read()) == previous, 'NaN target was accepted'
            assert len(states[-1].position) == 5 and all(math.isfinite(x) for x in states[-1].position)
            print('PASS: launch, feedback, sync read/write, ID ordering, limits, joystick, mode toggle, Cartesian IK and invalid targets')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            output.seek(0)
            print(output.read()[-6000:])


if __name__ == '__main__':
    main()
