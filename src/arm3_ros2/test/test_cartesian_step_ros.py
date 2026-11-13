"""Exercise the relative-command CLI against a fake ROS server, with no serial I/O."""

import math
import os
import threading

os.environ.setdefault('ROS_LOG_DIR', '/tmp/arm3_ros2_test_logs')

import pytest
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from arm3_ros2.cartesian_step import main


@pytest.mark.parametrize('valid_bounds', [True, False])
def test_relative_cli_only_publishes_preflighted_target(monkeypatch, valid_bounds):
    monkeypatch.setenv('ROS_DOMAIN_ID', '89')
    context = Context()
    rclpy.init(args=[], context=context, domain_id=89)
    server = Node('arm3', context=context)
    parameters = {
        'dry_run': False, 'calibration_confirmed': True, 'motor_ids': [1, 2, 3],
        'zero_ticks': [3819, 989, 2013], 'joint_directions': [1, -1, -1],
        'position_min_ticks': [3414 if valid_bounds else 3787, 957, 1981],
        'position_max_ticks': [3956, 1108, 2272],
        'upper_arm_m': 0.24606, 'forearm_m': 0.24225, 'base_height_m': 0.0,
        'dh_offsets_rad': [0.0, 0.0, math.pi / 2], 'base_frame': 'arm3_reference',
        'path_resolution_m': 0.001, 'max_path_joint_step_rad': 0.03,
    }
    server.declare_parameters('', list(parameters.items()))
    publisher = server.create_publisher(Int32MultiArray, '/arm3/encoder_ticks', 1)
    timer = server.create_timer(0.02, lambda: publisher.publish(
        Int32MultiArray(data=[3923, 1061, 2105])))
    received = []
    subscription = server.create_subscription(
        PointStamped, '/arm3/cartesian_target', received.append, 1)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        if valid_bounds:
            main(['--dy', '-0.20'])
            assert len(received) == 1
            point = received[0]
            assert point.header.frame_id == 'arm3_reference'
            assert (point.point.x, point.point.y, point.point.z) == pytest.approx(
                [0.2487917826269824, -0.15996906320808624, 0.2692572777163727])
        else:
            with pytest.raises(SystemExit) as result:
                main(['--dy', '-0.20'])
            assert result.value.code == 1
            assert not received
        assert timer is not None and subscription is not None
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        server.destroy_node()
        context.try_shutdown()
