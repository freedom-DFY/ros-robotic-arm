"""Hardware controller tests using an injected fake bus; never open a serial port."""

import os
from pathlib import Path
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
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from arm6_ros2.config import DEFAULTS, load_parameters
from arm6_ros2.model import ArmModel, JOINT_NAMES, quaternion_from_matrix
from arm6_ros2.node import Controller
from arm6_ros2.trajectory import Plan


@pytest.fixture
def controller(monkeypatch):
    context = Context()
    rclpy.init(context=context, domain_id=183)
    from arm6_ros2 import dynamixel_bus
    monkeypatch.setattr(dynamixel_bus, 'DynamixelBus',
                        lambda device, baud, ids, **options: FakeBus([3923, 1061, 2105, 2048, 2300, 2048, 3550]))
    node = Controller(context=context)
    node.p['home_on_enable'] = False
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


def test_enable_returns_to_taught_home_and_preserves_gripper(controller):
    node = controller
    node.p['home_on_enable'] = True
    # Encoder-taught home is independent of unverified CAD calibration.
    node.p['calibration_confirmed'] = False
    node.p['geometry_confirmed'] = False
    start = node.measured
    assert enable(node).success
    assert node.state == 'moving'
    plan = node.active['plan']
    for elapsed in np.linspace(0, plan.duration, 51):
        raw = node.model.encode(plan.sample(elapsed))
        assert raw[6] == start[6]
        assert all(lo <= tick <= hi for lo, tick, hi in
                   zip(node.model.tick_min, raw[:6], node.model.tick_max))
    # Repeated enable must not replace a running home trajectory.
    assert enable(node).success
    assert node.active['plan'] is plan
    finish(node)
    assert node.measured == tuple(node.p['home_ticks']) + (start[6],)
    assert node.state == 'holding'
    # Each new disabled -> enabled transition returns to the saved pose.
    assert enable(node, False).success
    node.bus.raw = start
    node.tick()
    assert enable(node).success
    finish(node)
    assert node.measured == tuple(node.p['home_ticks']) + (start[6],)


@pytest.mark.parametrize('case', ['unconfirmed', 'arm_outside', 'gripper_outside', 'home_outside'])
def test_invalid_auto_home_is_rejected_before_torque_on(controller, case):
    node = controller
    node.p['home_on_enable'] = True
    node.bus.arm = MagicMock(side_effect=AssertionError('Must not enable torque'))
    if case == 'unconfirmed':
        node.p['home_confirmed'] = False
    elif case == 'arm_outside':
        raw = list(node.bus.raw)
        raw[2] = 1041
        node.bus.raw = tuple(raw)
    elif case == 'gripper_outside':
        raw = list(node.bus.raw)
        raw[6] = 3152
        node.bus.raw = tuple(raw)
    else:
        node.user_origin_ticks = (3372, 992, 1041, 1063, 1250, 2487)
    assert not enable(node).success
    node.bus.arm.assert_not_called()
    assert not node.enabled and node.active is None
    assert not node.bus.writes


def test_auto_home_rechecks_register_bounds_after_enable(controller):
    node = controller
    node.p['home_on_enable'] = True
    node.bus.position_limits = ((0, 4095),)*6 + ((3400, 3700),)
    assert not enable(node).success
    assert node.state == 'fault' and node.active is None
    # Failure after torque-on holds measured feedback and never commands home.
    assert node.bus.writes[-1] == node.measured


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
    assert node.measured[6] == 4376
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


def test_loaded_home_to_trajectory_retains_hold_goal_and_reaches_measured_target(controller):
    node = controller
    assert enable(node).success
    node.bus.load_error[2] = 22
    node.bus.load_error[4] = 24
    assert node.return_home(None, None).success
    finish(node)
    home_goal = node.user_origin_ticks + (node.measured[6],)
    assert node.last_written == home_goal
    assert node.measured[2] == home_goal[2]-22
    assert node.measured[4] == home_goal[4]-24

    goal = request(node)
    target = node.model.encode(np.r_[goal.trajectory.points[0].positions, node.model.decode(node.measured)[6]])
    assert node.goal(goal) == GoalResponse.ACCEPT
    handle = SimpleNamespace(request=goal, is_cancel_requested=False, publish_feedback=MagicMock(),
                             succeed=MagicMock())
    execution = node.execute(handle)
    execution.send(None)
    writes = len(node.bus.writes)
    node._step(node.active['started'])
    assert len(node.bus.writes) == writes  # The first reference equals the existing loaded hold.
    finish(node)
    with pytest.raises(StopIteration) as done:
        execution.send(None)
    assert done.value.value.error_code == 0
    handle.succeed.assert_called_once()
    assert node.measured == target
    assert node.bus.writes[-1][2] == target[2]+22
    assert node.bus.writes[-1][4] == target[4]+24


def test_trajectory_rejects_hold_compensated_goal_outside_motor_limit(controller):
    node = controller
    raw = list(node.bus.raw)
    raw[2] = int(node.model.tick_max[2])-10
    node.bus.raw = tuple(raw)
    assert enable(node).success
    node.bus.load_error[2] = 22
    node.bus.write_positions(node.last_written)
    node.tick()
    goal = request(node)
    target = list(node.measured)
    target[2] = int(node.model.tick_max[2])-5
    goal.trajectory.points[0].positions = node.model.decode(target)[:6].tolist()
    assert node.goal(goal) == GoalResponse.REJECT
    assert node.reservation is None and node.active is None
    assert node.bus.writes == [tuple(raw)]


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
        self.goals = self.raw
        self.position_limits = ((0, 4095),)*6 + (ArmModel().gripper_limits,)
        self.writes = []
        self.models = tuple(DEFAULTS['expected_model_numbers'])
        self.follow_commands = True
        self.load_error = [0]*7

    def arm(self, acceleration, velocity, *, command_rate_hz, gripper_goal_pwm_limit=None):
        self.goals = self.raw
        return self.raw

    def read_positions(self):
        return self.raw

    def write_positions(self, raw, *, indices=None):
        selected = range(len(raw)) if indices is None else indices
        selected = set(selected)
        self.goals = tuple(raw[i] if i in selected else goal for i, goal in enumerate(self.goals))
        if self.follow_commands:
            self.raw = tuple(self.goals[i]-self.load_error[i] if i in selected else tick
                             for i, tick in enumerate(self.raw))
        self.writes.append(self.goals)

    def read_goal_position(self, index):
        return self.goals[index]

    def write_single_position(self, index, tick, *, verify=True):
        goals = list(self.goals)
        goals[index] = tick
        self.goals = tuple(goals)
        if self.follow_commands:
            raw = list(self.raw)
            raw[index] = tick-self.load_error[index]
            self.raw = tuple(raw)
        self.writes.append(self.goals)

    def read_hardware_errors(self):
        return (0,)*7

    def write_verified_positions(self, raw):
        self.write_positions(raw)

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
    node.bus.position_limits = ((0, 4095),)*6+((3400, 3700),)
    with pytest.raises(ValueError, match='register limits'):
        node._ready()


@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('positive', [False, True])
def test_cartesian_jog_translates_wrist_and_preserves_orientation_and_gripper(controller, axis, positive):
    node = controller
    enable(node)
    # The controlled point is frame 6 even after a tool has been calibrated.
    node.model.tcp[:3, 3] = [0.02, -0.01, 0.04]
    current = node.model.decode(node.measured)
    before = node.model.chain(current[:6])[-1]
    result = node.cartesian_jog_request(SetBool.Request(data=positive), None, axis)
    assert result.success, result.message
    assert node.state == 'moving'
    target = node.active['plan'].points[-1]
    after = node.model.chain(target[:6])[-1]
    delta = np.zeros(3)
    delta[axis] = 0.001 * (1 if positive else -1)
    assert np.allclose(after[:3, 3]-before[:3, 3], delta, atol=1e-5)
    assert np.allclose(after[:3, :3], before[:3, :3], atol=1e-4)
    assert target[6] == current[6]
    finish(node)


@pytest.mark.parametrize('case', ['disabled', 'calibration', 'geometry', 'busy', 'ik', 'stale'])
def test_cartesian_jog_rejects_without_replacing_motion_or_writing(controller, case):
    node = controller
    if case != 'disabled':
        enable(node)
    if case == 'calibration':
        node.p['calibration_confirmed'] = False
    elif case == 'geometry':
        node.p['geometry_confirmed'] = False
    elif case == 'busy':
        node.open_gripper(None, None)
    elif case == 'ik':
        node.model.cartesian_path = MagicMock(side_effect=ValueError('No continuous IK solution'))
    elif case == 'stale':
        node.last_feedback = time.monotonic()-1.
    active = node.active
    writes = list(node.bus.writes)
    result = node.cartesian_jog_request(SetBool.Request(data=True), None, 0)
    assert not result.success
    assert node.active is active
    assert node.bus.writes == writes
    assert node.state != 'fault'


def test_cartesian_jog_stop_freezes_existing_goal(controller):
    node = controller
    enable(node)
    assert node.cartesian_jog_request(SetBool.Request(data=True), None, 0).success
    goal = node.last_written
    writes = list(node.bus.writes)
    assert node.stop(None, None).success
    assert node.active is None and node.state == 'stopped'
    assert node.last_written == goal
    assert node.bus.writes == writes


@pytest.mark.parametrize('name,value', [
    ('gamepad_speed_scale', 5.), ('gamepad_speed_scale', float('nan')),
    ('gamepad_cartesian_speed_m_s', 0.1), ('gamepad_command_timeout_s', 0.),
])
def test_invalid_gamepad_speed_settings_are_rejected(controller, name, value):
    controller.p[name] = value
    with pytest.raises(ValueError, match='gamepad'):
        controller._validate_settings()


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
    goal = node.last_written
    writes = list(node.bus.writes)
    node.tick()
    assert node.state == 'stopped' and node.active is None
    assert future.result()[2] == 'canceled'
    assert node.last_written == goal
    assert node.bus.writes == writes


@pytest.mark.parametrize('motion', ['holding', 'cartesian', 'rotation', 'gripper'])
def test_repeated_control_switch_stops_preserve_loaded_goals_and_torque(controller, motion):
    node = controller
    enable(node)
    node.bus.load_error = [0, 18, 22, 0, 12, 0, 0]
    node.bus.write_positions(node.last_written)
    node.tick()
    goal, feedback = node.last_written, node.measured
    assert goal != feedback
    node.bus.read_goal_position = lambda index: node.last_written[index]
    node.bus.arm = MagicMock()
    node.bus.disarm = MagicMock()
    if motion == 'cartesian':
        result = node.cartesian_jog_request(SetBool.Request(data=True), None, 2)
        assert result.success, result.message
    elif motion == 'rotation':
        assert node.calibration_jog_request(SetBool.Request(data=True), None, 5,
                                            require_unconfirmed=False).success
    elif motion == 'gripper':
        assert node.manual_gripper_jog(SetBool.Request(data=True), None).success
    writes = list(node.bus.writes)
    for _ in range(5):
        assert node.stop(None, None).success
        node.tick()
        assert node.last_written == goal
        assert node.measured == feedback
        assert node.bus.writes == writes
        assert node.enabled and node.state == 'stopped'
        assert node.active is None and node.calibration_jog is None
    node.bus.arm.assert_not_called()
    node.bus.disarm.assert_not_called()


def calibrated_pose(node, ticks):
    settings = load_parameters(Path(__file__).resolve().parents[1]/'config/arm6_calibration.yaml')
    node.p.update(settings)
    node.model = ArmModel(settings)
    node.frame = settings['base_frame']
    node.bus = FakeBus(ticks)
    node.tick()
    assert enable(node).success


@pytest.mark.parametrize('ticks', [
    [3449, 1227, 1416, 1117, 1298, 2525, 3548],
    [3339, 1155, 1305, 1064, 1200, 2486, 3627],
])
@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('positive', [False, True])
def test_loaded_calibrated_pose_manual_xyz_steps_and_switches_do_not_drift(controller, ticks, axis, positive):
    node = controller
    calibrated_pose(node, ticks)
    node.bus.load_error = [0, 8, 12, 0, 8, 0, 0]
    node.bus.write_positions(node.last_written)
    node.tick()
    before = node.model.forward(node.model.decode(node.measured)[:6])
    reply = node.cartesian_jog_request(SetBool.Request(data=positive), None, axis)
    assert reply.success, reply.message
    finish(node)
    after = node.model.forward(node.model.decode(node.measured)[:6])
    expected = np.zeros(3)
    expected[axis] = 0.001*(1 if positive else -1)
    # Include encoder quantization, then verify stopping cannot consume the
    # just-achieved Cartesian step by dropping the load-bearing hold offset.
    assert np.allclose(after[:3, 3]-before[:3, 3], expected, atol=0.0005)
    assert np.allclose(after[:3, :3], before[:3, :3], atol=0.003)
    goal, measured = node.last_written, node.measured
    for _ in range(3):
        assert node.stop(None, None).success
        node.tick()
        assert node.last_written == goal and node.measured == measured


def test_collapsed_pose_z_down_rejects_with_limit_details_without_changing_hold(controller):
    node = controller
    calibrated_pose(node, [3608, 744, 1044, 1047, 763, 2591, 4166])
    goal, writes = node.last_written, list(node.bus.writes)
    result = node.cartesian_jog_request(SetBool.Request(data=False), None, 2)
    assert not result.success
    assert 'Wrist z-' in result.message
    assert 'near joint limits' in result.message
    assert 'J2/ID2=744' in result.message and 'J3/ID3=1044' in result.message
    assert node.last_written == goal and node.bus.writes == writes
    assert node.enabled and node.state == 'holding' and node.active is None


def stream_packet(node, token, intent, deadman=True, end=False):
    msg = Joy(axes=[float(v) for v in intent], buttons=[int(deadman), int(end)])
    msg.header.frame_id = token
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg


def stream_tick(node, token, intent, **buttons):
    node.gamepad_velocity(stream_packet(node, token, intent, **buttons))
    if node.gamepad_stream is not None:
        node.gamepad_stream['last_step'] = time.monotonic()-1/30
    node.tick()


def begin_stream(node):
    assert enable(node).success
    response = node.start_gamepad(None, None)
    assert response.success, response.message
    return response.message


def test_continuous_stream_moves_without_active_waypoint_plans(controller):
    node = controller
    calibrated_pose(node, [3449, 1227, 1416, 1117, 1298, 2525, 3548])
    token = begin_stream(node)
    before = node.model.forward(node.model.decode(node.measured)[:6])
    for _ in range(90):
        stream_tick(node, token, [0, 0, 1, 0, 0])
        assert node.state == 'moving' and node.gamepad_stream is not None
        assert node.active is None and node.calibration_jog is None
    node.tick()
    after = node.model.forward(node.model.decode(node.measured)[:6])
    assert after[2, 3]-before[2, 3] > .02
    assert np.linalg.norm(after[:2, 3]-before[:2, 3]) < .001


def test_stream_release_and_switch_retain_loaded_goals_without_torque_cycle(controller):
    node = controller
    assert enable(node).success
    node.bus.load_error = [0, 12, 18, 0, 8, 0, 0]
    node.bus.write_positions(node.last_written)
    node.tick()
    token = begin_stream(node)
    for _ in range(10):
        stream_tick(node, token, [0, 0, 1, 0, 0])
    goal, writes = node.last_written, list(node.bus.writes)
    node.bus.arm, node.bus.disarm = MagicMock(), MagicMock()
    stream_tick(node, token, [0]*5, deadman=False)
    assert node.last_written == goal and node.bus.writes == writes
    assert node.enabled and node.gamepad_stream is not None
    for _ in range(10):
        stream_tick(node, token, [0, 0, 0, 0, 1])
        assert node.last_written[:6] == goal[:6]
    assert node.last_written[6] > goal[6]
    node.bus.arm.assert_not_called()
    node.bus.disarm.assert_not_called()


def test_backend_watchdog_stops_when_gamepad_client_disappears(controller):
    node = controller
    token = begin_stream(node)
    stream_tick(node, token, [0, 0, 0, 1, 0])
    goal, writes = node.last_written, list(node.bus.writes)
    node.gamepad_stream['last_input'] = time.monotonic()-1.
    node.tick()
    assert node.gamepad_stream is None and node.state == 'stopped'
    assert node.enabled and node.last_written == goal and node.bus.writes == writes
    node.gamepad_velocity(stream_packet(node, token, [0, 0, 0, 1, 0]))
    node.tick()
    assert node.gamepad_stream is None and node.last_written == goal


def test_delayed_packet_does_not_refresh_watchdog(controller):
    node = controller
    token = begin_stream(node)
    old_time = node.gamepad_stream['last_input']
    msg = stream_packet(node, token, [0, 0, 1, 0, 0])
    msg.header.stamp.sec -= 1
    node.gamepad_velocity(msg)
    assert node.gamepad_stream['last_input'] == old_time
    assert np.all(node.gamepad_stream['intent'] == 0)


def test_stream_session_cannot_replace_other_motion_or_restart_after_desktop_stop(controller):
    node = controller
    token = begin_stream(node)
    assert not node.start_gamepad(None, None).success
    assert not node.manual_gripper_jog(SetBool.Request(data=True), None).success
    assert node.goal(request(node)) == GoalResponse.REJECT
    assert node.stop(None, None).success
    assert node.gamepad_stream is None
    assert node.open_gripper(None, None).success
    active = node.active
    node.gamepad_velocity(stream_packet(node, token, [0, 0, 1, 0, 0]))
    node.gamepad_velocity(stream_packet(node, token, [0]*5, end=True))
    assert node.active is active  # Late packets cannot stop another operator.


def test_stream_end_releases_ownership_and_invalidates_old_token(controller):
    node = controller
    token = begin_stream(node)
    node.gamepad_velocity(stream_packet(node, token, [0]*5, end=True))
    assert node.gamepad_stream is None and node.enabled
    new_token = begin_stream(node)
    assert new_token != token
    node.gamepad_velocity(stream_packet(node, token, [0, 0, 1, 0, 0]))
    assert np.all(node.gamepad_stream['intent'] == 0)


@pytest.mark.parametrize('kind', ['nan', 'multiple', 'bad_buttons'])
def test_malformed_current_session_packet_stops_without_losing_hold(controller, kind):
    node = controller
    token = begin_stream(node)
    msg = stream_packet(node, token, [0]*5)
    if kind == 'nan':
        msg.axes[0] = float('nan')
    elif kind == 'multiple':
        msg.axes = [1., 1., 0., 0., 0.]
    else:
        msg.buttons = [1]
    goal, writes = node.last_written, list(node.bus.writes)
    node.gamepad_velocity(msg)
    assert node.gamepad_stream is None and node.enabled
    assert node.last_written == goal and node.bus.writes == writes


def test_stream_reaches_j6_limit_and_reverse_works_without_new_session(controller):
    node = controller
    raw = list(node.bus.raw)
    raw[5] = 4090
    node.bus.raw = tuple(raw)
    node.tick()
    token = begin_stream(node)
    for _ in range(30):
        stream_tick(node, token, [0, 0, 0, 1, 0])
        assert node.last_written[5] <= 4095
    assert node.last_written[5] == 4095 and node.state == 'holding'
    assert node.gamepad_stream['token'] == token
    for _ in range(5):
        stream_tick(node, token, [0, 0, 0, -1, 0])
    assert node.last_written[5] < 4095 and node.state == 'moving'


def test_stream_requires_confirmed_geometry(controller):
    node = controller
    assert enable(node).success
    node.p['geometry_confirmed'] = False
    assert not node.start_gamepad(None, None).success
    assert node.gamepad_stream is None


def test_stalled_wrist_ends_stream_before_command_exceeds_tracking_limit(controller):
    node = controller
    token = begin_stream(node)
    node.bus.follow_commands = False
    intent = [0, 0, 0, 1, 0]
    for _ in range(100):
        stream_tick(node, token, intent)
        lag = np.abs(node.model.decode(node.last_written)-node.model.decode(node.measured))
        assert np.all(lag[:6] < node.p['max_tracking_error_rad'])
        assert lag[6] < node.p['gripper_tracking_error_m']
        if node.gamepad_stream is None:
            break
    else:
        pytest.fail('Stalled motor did not stop the stream')
    goal, writes = node.last_written, list(node.bus.writes)
    node.tick()
    assert node.state == 'stopped' and node.enabled
    assert node.last_written == goal and node.bus.writes == writes


@pytest.mark.parametrize('direction', [-1, 1])
def test_slow_gripper_keeps_moving_with_bounded_goal_lead(controller, direction):
    node = controller
    token = begin_stream(node)
    node.bus.follow_commands = False
    start = node.measured
    for _ in range(120):
        stream_tick(node, token, [0, 0, 0, 0, direction])
        assert node.state == 'moving' and node.gamepad_stream['token'] == token
        gap_error = abs(node.model.decode(node.last_written)[6]-node.model.decode(node.measured)[6])
        assert gap_error < node.p['gripper_tracking_error_m']
        measured = node.bus.raw[6]
        step = max(-2, min(2, node.last_written[6]-measured))
        node.bus.raw = node.bus.raw[:6]+(measured+step,)
    assert (node.measured[6]-start[6])*direction > 200
    assert node.last_written[:6] == start[:6]


def test_stalled_gripper_holds_without_ending_session_and_can_reverse(controller):
    node = controller
    token = begin_stream(node)
    node.bus.follow_commands = False
    for _ in range(100):
        stream_tick(node, token, [0, 0, 0, 0, 1])
        gap_error = abs(node.model.decode(node.last_written)[6]-node.model.decode(node.measured)[6])
        assert gap_error < node.p['gripper_tracking_error_m']
        assert node.gamepad_stream['token'] == token
        if node.state == 'holding':
            break
    else:
        pytest.fail('Stalled gripper did not enter hold')
    assert '夹爪未跟随' in node.gamepad_stream['servo'].message
    goal, writes = node.last_written, list(node.bus.writes)
    for _ in range(10):
        stream_tick(node, token, [0, 0, 0, 0, 1])
    assert node.last_written == goal and node.bus.writes == writes
    for _ in range(10):
        stream_tick(node, token, [0, 0, 0, 0, -1])
    assert node.state == 'moving' and node.last_written[6] < goal[6]


@pytest.mark.parametrize('obstruction_tick', [3350, 3184])
def test_closing_contact_relaxes_to_small_preload_and_latches_until_open(controller, obstruction_tick):
    node = controller
    node.bus.raw = node.bus.raw[:6]+(obstruction_tick,)
    token = begin_stream(node)
    node.bus.follow_commands = False
    close = [0, 0, 0, 0, -1]
    opened = [0, 0, 0, 0, 1]
    before = node.last_written[:6]
    lowest = obstruction_tick
    for _ in range(60):
        stream_tick(node, token, close)
        lowest = min(lowest, node.last_written[6])
        if node.gripper_contact:
            break
    assert node.gripper_contact and node.state == 'holding'
    goal = node.last_written
    assert 3180 <= goal[6] <= obstruction_tick <= goal[6]+node.p['gripper_contact_preload_ticks']
    assert goal[6] >= lowest and goal[:6] == before
    if obstruction_tick > 3184:
        assert goal[6] > lowest
    stream_tick(node, token, [0]*5, deadman=False)
    assert node.stop(None, None).success
    response = node.start_gamepad(None, None)
    assert response.success
    for _ in range(30):
        stream_tick(node, response.message, close)
        assert node.last_written == goal
    stream_tick(node, response.message, opened)
    assert not node.gripper_contact and node.last_written[6] > goal[6]


def test_releasing_closing_stick_relaxes_goal_before_contact_timer(controller):
    node = controller
    token = begin_stream(node)
    node.bus.follow_commands = False
    start = node.measured
    for _ in range(6):
        stream_tick(node, token, [0, 0, 0, 0, -1])
    goal = node.last_written
    assert not node.gripper_contact and start[6]-goal[6] > node.p['gripper_contact_preload_ticks']
    stream_tick(node, token, [0]*5, deadman=False)
    after = node.last_written
    assert after[:6] == start[:6]
    assert after[6] == start[6]-node.p['gripper_contact_preload_ticks']
    stream_tick(node, token, [0]*5, deadman=False)
    assert node.last_written == after


@pytest.mark.parametrize('axis,index', [(5, 3), (6, 4)])
def test_wrist_controls_command_only_the_selected_motor(controller, axis, index):
    node = controller
    token = begin_stream(node)
    before = node.last_written
    intent = [0]*7
    intent[axis] = 1
    for _ in range(20):
        stream_tick(node, token, intent)
        assert node.state == 'moving'
    assert node.last_written[index] != before[index]
    assert all(new == old for i, (new, old) in enumerate(zip(node.last_written, before)) if i != index)


def test_gamepad_and_controller_exchange_continuous_commands_over_ros(controller):
    from rclpy.executors import MultiThreadedExecutor
    from arm6_ros2.gamepad_control import GamepadControl, load_profile

    node = controller
    assert enable(node).success
    profile = load_profile(Path(__file__).resolve().parents[3]/'docs/hardware/f710_mapping.json')
    pad = GamepadControl(profile, context=node.context)
    executor = MultiThreadedExecutor(context=node.context, num_threads=2)
    executor.add_node(node)
    executor.add_node(pad)
    controls = Joy(axes=list(profile['baseline_axes']), buttons=[0]*profile['buttons_count'])

    def pump(duration=0., until=None):
        end = time.monotonic()+(duration if until is None else 3.)
        while time.monotonic() < end:
            pad.on_joy(controls)
            executor.spin_once(timeout_sec=0.01)
            if until is not None and until():
                return
        if until is not None:
            assert until(), (pad.notice, node.state)

    try:
        pump(until=lambda: pad.state == 'holding' and pad.feedback_fresh())
        pad.enable_input()
        pump(until=lambda: pad.session is not None)
        token, before = pad.session, node.last_written
        controls.buttons[profile['mapping']['lb']['index']] = 1
        axis = profile['mapping']['left_stick_up']
        controls.axes[axis['index']] = axis['baseline']+axis['sign']
        pump(.5)
        assert node.gamepad_stream is not None and node.gamepad_stream['token'] == token
        assert node.state == 'moving' and node.last_written != before
        assert node.active is None and node.calibration_jog is None
        controls.axes[axis['index']] = axis['baseline']
        pump(until=lambda: node.state == 'holding')
        goal, count = node.last_written, len(node.bus.writes)
        pump(.15)
        assert node.last_written == goal and len(node.bus.writes) == count
        assert pad.enabled and pad.session == token
        pad.disable()
        pump(until=lambda: node.gamepad_stream is None and pad.session is None)
        assert node.enabled and node.last_written == goal
    finally:
        executor.shutdown(timeout_sec=2.)
        executor.remove_node(pad)
        executor.remove_node(node)
        pad.destroy_node()


def test_startup_model_mismatch_closes_bus_without_enabling(monkeypatch):
    from arm6_ros2 import dynamixel_bus
    context = Context()
    rclpy.init(context=context, domain_id=183)
    bus = FakeBus([2048]*7)
    bus.models = (1120,)*7
    bus.arm = MagicMock()
    bus.close = MagicMock()
    monkeypatch.setattr(dynamixel_bus, 'DynamixelBus', lambda device, baud, ids, **options: bus)
    try:
        with pytest.raises(ValueError, match='Model mismatch'):
            Controller(context=context)
        bus.arm.assert_not_called()
        bus.close.assert_called_once()
    finally:
        context.shutdown()
