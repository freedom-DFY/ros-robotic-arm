"""Exercise the asynchronous fixed-cycle service and status topic in isolation."""

import os
import threading
import time

os.environ.setdefault('ROS_LOG_DIR', '/tmp/arm3_ros2_test_logs')

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

from arm3_ros2.node import Controller


def test_fixed_cycle_service_reports_both_stages_and_completion():
    context = Context()
    rclpy.init(args=[], context=context, domain_id=92)
    parameters = {
        'dry_run': True, 'zero_ticks': [3819, 989, 2013], 'joint_directions': [1, -1, -1],
        'position_min_ticks': [3414, 957, 1981], 'position_max_ticks': [3956, 1108, 2272],
        'dh_offsets_rad': [0.0, 0.0, 1.5707963267948966],
        'dry_start_ticks': [3446, 1076, 2240], 'fixed_start_ticks': [3923, 1061, 2105],
        'fixed_line_delta_m': [0.0, -0.2, 0.0], 'fixed_settle_time_s': 0.1,
        'fixed_stage_timeout_s': 15.0, 'fixed_arrival_tolerance_ticks': 2,
        'rate_hz': 100.0, 'joint_speed_rad_s': 0.5, 'cartesian_speed_m_s': 0.2,
        'path_resolution_m': 0.001, 'max_path_joint_step_rad': 0.03,
    }
    server = Controller(context=context, parameter_overrides=[
        Parameter(k, value=v) for k, v in parameters.items()])
    assert server.bus is None
    client_node = Node('fixed_cycle_test_client', context=context)
    states = []
    subscription = client_node.create_subscription(
        String, '/arm3/motion_status', lambda msg: states.append(msg.data), 20)
    client = client_node.create_client(Trigger, '/arm3/run_fixed_line')
    server_executor = SingleThreadedExecutor(context=context)
    server_executor.add_node(server)
    client_executor = SingleThreadedExecutor(context=context)
    client_executor.add_node(client_node)
    thread = threading.Thread(target=server_executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_service(timeout_sec=3.0)
        deadline = time.monotonic() + 3.0
        while not states:
            assert time.monotonic() < deadline
            client_executor.spin_once(timeout_sec=0.02)
        future = client.call_async(Trigger.Request())
        client_executor.spin_until_future_complete(future, timeout_sec=3.0)
        assert future.done() and future.result().success
        deadline = time.monotonic() + 12.0
        while 'completed' not in states:
            assert time.monotonic() < deadline, states
            assert 'fault' not in states
            client_executor.spin_once(timeout_sec=0.02)
        for stage in ('moving_to_start', 'settling_start', 'line_motion', 'settling_end', 'completed'):
            assert stage in states
        assert server.measured == (3446, 1076, 2240)
        assert server.fixed_cycle is None and not server.path
        assert subscription is not None
    finally:
        server_executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        client_executor.shutdown(timeout_sec=2.0)
        client_node.destroy_node()
        server.destroy_node()
        context.try_shutdown()
