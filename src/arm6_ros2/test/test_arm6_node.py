"""Hardware controller tests using an injected fake bus; never open a serial port."""

import os
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

os.environ.setdefault('ROS_LOG_DIR', '/tmp/arm6_unit_test_logs')

import numpy as np
import pytest
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import GoalResponse
from rclpy.context import Context
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from arm6_ros2.config import DEFAULTS
from arm6_ros2.model import JOINT_NAMES, quaternion_from_matrix
from arm6_ros2.node import Controller
from arm6_ros2.trajectory import Plan


@pytest.fixture
def controller(monkeypatch):
    context = Context()
    rclpy.init(context=context)
    from arm6_ros2 import dynamixel_bus
    monkeypatch.setattr(dynamixel_bus, 'DynamixelBus',
                        lambda device, baud, ids: FakeBus([3923, 1061, 2105, 2048, 2300, 2048, 2200]))
    node = Controller(context=context)
    node.p['calibration_confirmed'] = True
    node.p['geometry_confirmed'] = True
    node.tick()
    yield node
    node.destroy_node()
    context.shutdown()


def enable(node, enabled=True):
    return node.enable(SetBool.Request(data=enabled), SetBool.Response())


def finish(node):
    node.active['started'] = time.monotonic()-node.active['plan'].duration-0.01
    node.tick()
    node.active['stable'] = time.monotonic()-node.p['settle_time_s']-0.01
    node.tick()
    assert node.active is None
    assert node.state == 'holding'


def test_manual_enable_and_joint_name_order(controller):
    node = controller
    assert not node.enabled and node.state == 'read_only'
    assert not node.has_parameter('dry_run')
    held = node.measured
    node.joint_target(JointState(position=[0.0]*6))
    assert node.active is None and node.measured == held
    assert enable(node).success
    assert node.measured == held
    target = node.model.decode(node.measured)[:6]+[-0.01, 0.01, -0.01, 0.01, 0.01, -0.01]
    node.joint_target(JointState(name=list(reversed(JOINT_NAMES)), position=list(reversed(target.tolist()))))
    assert np.allclose(node.active['plan'].points[-1][:6], target)
    finish(node)
    assert np.allclose(node.model.decode(node.measured)[:6], target, atol=0.001)


@pytest.mark.parametrize('message', [
    JointState(position=[0.0]*5), JointState(position=[float('nan')]*6),
    JointState(position=[10.0]*6), JointState(name=['joint_1']*6, position=[0.0]*6),
])
def test_invalid_targets_do_not_move(controller, message):
    node = controller
    enable(node)
    held = node.measured
    node.joint_target(message)
    assert node.active is None and node.measured == held and node.enabled


def test_gripper_does_not_change_arm_and_busy_commands_do_not_replace_it(controller):
    node = controller
    enable(node)
    original = node.measured
    assert node.open_gripper(None, None).success
    active = node.active
    node.joint_target(JointState(position=[0.0]*6))
    node.gripper_target(Float64(data=0.03))
    assert node.active is active
    finish(node)
    assert node.measured[:6] == original[:6]
    assert node.measured[6] == DEFAULTS['gripper_open_ticks']
    assert node.close_gripper(None, None).success
    finish(node)
    assert node.measured[:6] == original[:6]
    assert node.measured[6] == DEFAULTS['gripper_closed_ticks']


def test_pose_command_rejects_wrong_frame_bad_quaternion_and_accepts_nearby(controller):
    node = controller
    enable(node)
    target = PoseStamped()
    node.pose_target(target)
    assert node.active is None
    target.header.frame_id = node.frame
    node.pose_target(target)
    assert node.active is None
    q = node.model.decode(node.measured)[:6]
    pose = node.model.forward(q+[-0.01, 0.01, -0.01, 0.01, -0.01, 0.01])
    target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, pose[:3, 3])
    (target.pose.orientation.x, target.pose.orientation.y,
     target.pose.orientation.z, target.pose.orientation.w) = map(float, quaternion_from_matrix(pose[:3, :3]))
    node.pose_target(target)
    assert node.active is not None
    finish(node)
    actual = node.model.forward(node.model.decode(node.measured)[:6])
    assert np.allclose(actual[:3, 3], pose[:3, 3], atol=0.001)


def test_stop_disable_stale_feedback_and_fault_recovery(controller):
    node = controller
    enable(node)
    node.open_gripper(None, None)
    node.tick()
    assert node.stop(None, None).success
    held = node.measured
    node.tick()
    assert node.measured == held and node.active is None
    node.last_feedback -= 10
    node.gripper_target(Float64(data=0.01))
    assert node.active is None
    node.tick()
    node.gripper_target(Float64(data=0.01))
    node.last_tick -= 10
    node.tick()
    assert node.state == 'fault' and node.active is None and not node.enabled
    assert not enable(node).success
    assert enable(node, False).success
    assert enable(node).success


def test_hold_enable_independent_of_software_bounds(controller):
    node = controller
    node.bus.raw = (10, 100, 200, 0, 0, 0, 1000)
    node.tick()
    assert enable(node).success
    assert node.measured == (10, 100, 200, 0, 0, 0, 1000)
    node.gripper_target(Float64(data=0.02))
    assert node.active is None and node.enabled


def request(node, seconds=10):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = (node.model.decode(node.measured)[:6]+[-0.01, 0.01, -0.01, 0.01, 0.01, -0.01]).tolist()
    point.time_from_start.sec = seconds
    goal.trajectory.points = [point]
    return goal


def test_trajectory_reservation_rejects_overlapping_requests(controller):
    node = controller
    enable(node)
    assert node.goal(request(node)) == GoalResponse.ACCEPT
    assert node.goal(request(node)) == GoalResponse.REJECT
    node.open_gripper(None, None)
    assert node.active is None
    node.stop(None, None)
    assert node.reservation is None
    assert node.goal(request(node, 0)) == GoalResponse.REJECT


def test_stopped_action_cannot_consume_a_later_goal_reservation(controller):
    import asyncio
    node = controller
    enable(node)
    first = request(node)
    assert node.goal(first) == GoalResponse.ACCEPT
    node.stop(None, None)
    second = request(node)
    assert node.goal(second) == GoalResponse.ACCEPT
    reservation = node.reservation
    handle = SimpleNamespace(request=first, abort=MagicMock())
    result = asyncio.run(node.execute(handle))
    assert result.error_code == -1
    handle.abort.assert_called_once()
    assert node.reservation is reservation and node.reservation[0] is second
    assert node.active is None


@pytest.mark.parametrize('change', ['names', 'velocities', 'stamp', 'time', 'negative_tolerance', 'multi_dof'])
def test_invalid_trajectory_goals_rejected(controller, change):
    node = controller
    enable(node)
    goal = request(node)
    if change == 'names':
        goal.trajectory.joint_names = ['joint_1']*6
    elif change == 'velocities':
        goal.trajectory.points[0].velocities = [0.0]*6
    elif change == 'stamp':
        goal.trajectory.header.stamp.sec = 100
    elif change == 'time':
        goal.trajectory.points[0].time_from_start.sec = 0
    elif change == 'negative_tolerance':
        from control_msgs.msg import JointTolerance
        goal.path_tolerance = [JointTolerance(name='joint_1', position=-1.0)]
    else:
        goal.multi_dof_trajectory.joint_names = ['floating_joint']
    assert node.goal(goal) == GoalResponse.REJECT
    assert node.active is None and node.reservation is None


class FakeBus:
    def __init__(self, raw):
        self.raw = tuple(raw)
        self.position_limits = ((0, 4095),)*7
        self.writes = []
        self.models = tuple(DEFAULTS['expected_model_numbers'])
        self.follow_commands = True

    def arm(self, acceleration, velocity, *, command_rate_hz):
        return self.raw

    def read_positions(self):
        return self.raw

    def write_positions(self, raw):
        if self.follow_commands:
            self.raw = tuple(raw)
        self.writes.append(tuple(raw))

    def read_hardware_errors(self):
        return (0,)*7

    def disarm(self):
        return []

    def close(self):
        pass


def test_real_motion_gates_calibration_and_geometry(controller):
    node = controller
    enable(node)
    node.bus = FakeBus(node.measured)
    node.p['calibration_confirmed'] = False
    node.p['geometry_confirmed'] = False
    with pytest.raises(ValueError, match='calibrations'):
        node._ready()
    node.p['calibration_confirmed'] = True
    node._ready()
    with pytest.raises(ValueError, match='DH model'):
        node._ready(cartesian=True)
    node.p['geometry_confirmed'] = True
    node._ready(cartesian=True)
    node.bus.position_limits = ((0, 4095),)*6+((2000, 2500),)
    with pytest.raises(ValueError, match='register limits'):
        node._ready()


def test_tracking_fault_aborts_action_and_holds_measured_position(controller):
    node = controller
    enable(node)
    current = node.model.decode(node.measured)
    node.bus = FakeBus(node.measured)
    node.p['calibration_confirmed'] = True
    from rclpy.task import Future
    future = Future()
    handle = SimpleNamespace(is_cancel_requested=False, publish_feedback=MagicMock())
    target = current.copy()
    target[0] -= 0.05
    node._start(Plan(node.model, current, [target]), handle=handle, future=future)
    commanded = current.copy()
    commanded[0] -= 0.2
    node.last_written = node.model.encode(commanded)
    raw = node.measured
    node.tick()
    assert node.state == 'fault' and not node.enabled
    assert future.done() and future.result()[0] == -4
    assert node.bus.writes[-1] == raw


def test_goal_timeout_returns_goal_tolerance_error(controller):
    node = controller
    enable(node)
    node.bus = FakeBus(node.measured)
    node.bus.follow_commands = False
    current = node.model.decode(node.measured)
    target = current.copy()
    target[0] -= 0.03  # Below path tolerance but above goal tolerance.
    from rclpy.task import Future
    future = Future()
    node._start(Plan(node.model, current, [target]), future=future)
    node.active['started'] -= node.active['plan'].duration+node.p['arrival_timeout_s']+1
    node.tick()
    assert node.state == 'fault'
    assert future.result()[0] == -5


def test_action_cancel_holds_and_resolves_canceled_result(controller):
    from rclpy.task import Future
    node = controller
    enable(node)
    current = node.model.decode(node.measured)
    target = current.copy()
    target[0] -= 0.1
    future = Future()
    handle = SimpleNamespace(is_cancel_requested=True)
    node._start(Plan(node.model, current, [target]), handle=handle, future=future)
    node.tick()
    assert node.state == 'stopped' and node.active is None
    assert future.result()[2] == 'canceled'
    assert node.bus.writes[-1] == node.measured


def test_startup_model_mismatch_closes_bus_without_enabling(monkeypatch):
    from arm6_ros2 import dynamixel_bus
    context = Context()
    rclpy.init(context=context)
    bus = FakeBus([2048]*7)
    bus.models = (1120,)*7
    bus.arm = MagicMock()
    bus.close = MagicMock()
    monkeypatch.setattr(dynamixel_bus, 'DynamixelBus', lambda device, baud, ids: bus)
    try:
        with pytest.raises(ValueError, match='Model mismatch'):
            Controller(context=context)
        bus.arm.assert_not_called()
        bus.close.assert_called_once()
    finally:
        context.shutdown()
