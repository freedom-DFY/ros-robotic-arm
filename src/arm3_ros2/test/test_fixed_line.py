"""Fixed-start sequencing, using simulated encoders and a deterministic clock."""

import os

os.environ.setdefault('ROS_LOG_DIR', '/tmp/arm3_ros2_test_logs')

import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from arm3_ros2.node import Controller

FIXED = (3923, 1061, 2105)
END = (3446, 1076, 2240)


@pytest.fixture
def make_controller():
    context = Context()
    rclpy.init(args=[], context=context, domain_id=91)
    nodes = []

    def create(start=END, **overrides):
        parameters = {
            'dry_run': True, 'calibration_confirmed': True,
            'zero_ticks': [3819, 989, 2013], 'joint_directions': [1, -1, -1],
            'position_min_ticks': [3414, 957, 1981], 'position_max_ticks': [3956, 1108, 2272],
            'dh_offsets_rad': [0.0, 0.0, 1.5707963267948966],
            'joint_speed_rad_s': 0.02, 'cartesian_speed_m_s': 0.005,
            'path_resolution_m': 0.001, 'max_path_joint_step_rad': 0.03,
            'max_tracking_error_rad': 0.02, 'dry_start_ticks': list(start),
            'fixed_start_ticks': list(FIXED), 'fixed_line_delta_m': [0.0, -0.20, 0.0],
        }
        parameters.update(overrides)
        node = Controller(context=context, parameter_overrides=[
            Parameter(k, value=v) for k, v in parameters.items()])
        node.timer.cancel()
        clock = [0.0]
        node._now = lambda: clock[0]
        node.last_feedback_time = 0.0
        nodes.append(node)

        def tick(count=1):
            for _ in range(count):
                clock[0] += 1.0 / node.hz
                node.tick()

        return node, tick, clock

    yield create
    for node in reversed(nodes):
        node.destroy_node()
    context.try_shutdown()


def start_cycle(node):
    return node.run_fixed_line(Trigger.Request(), Trigger.Response())


def until(node, tick, predicate):
    for _ in range(6000):
        if predicate():
            return
        assert node.motion_state != 'fault'
        tick()
    pytest.fail('Fixed cycle failed to reach the expected stage')


@pytest.mark.parametrize('initial', [END, (3811, 999, 2040), FIXED,
                                     (3300, 1061, 2105), (3300, 900, 1900)])
def test_different_enable_positions_return_to_same_start_then_same_end(make_controller, initial):
    node, tick, clock = make_controller(initial)
    assert start_cycle(node).success
    assert not node.motion_bounds_active
    until(node, tick, lambda: node.motion_state == 'settling_start')
    assert node.measured == FIXED and not node.path
    reached = clock[0]
    until(node, tick, lambda: node.motion_state == 'line_motion')
    assert node.motion_bounds_active
    assert clock[0] - reached >= node.fixed_settle
    assert node.measured == FIXED
    until(node, tick, lambda: node.motion_state == 'completed')
    assert node.measured == END
    assert node.fixed_cycle is None and not node.path


def test_return_is_joint_interpolation_without_cartesian_preflight(make_controller, monkeypatch):
    initial = (3300, 900, 1900)
    node, tick, _ = make_controller(initial)
    calls = []
    original = node.kin.line_path

    def record(start, target, resolution, max_jump):
        calls.append((start, target))
        return original(start, target, resolution, max_jump)

    monkeypatch.setattr(node.kin, 'line_path', record)
    assert start_cycle(node).success
    assert len(calls) == 1  # Only the later Cartesian line uses IK.
    assert calls[0][0] == node.calibration.radians(FIXED)
    assert list(node.path) == [(node.calibration.radians(FIXED), node.joint_speed)]
    tick(5)
    assert node.enabled and not node.motion_bounds_active
    assert not node.calibration.contains_ticks(node.measured)
    assert all(min(a, b) <= p <= max(a, b) for a, b, p in zip(initial, FIXED, node.measured))
    assert node.stop(Trigger.Request(), Trigger.Response()).success
    held = node.measured
    tick(30)
    assert node.measured == held and node.fixed_cycle is None and node.motion_state == 'stopped'


def test_cartesian_leg_uses_software_limits_again(make_controller):
    node, tick, _ = make_controller(FIXED)
    assert start_cycle(node).success
    until(node, tick, lambda: node.motion_state == 'line_motion')
    assert node.motion_bounds_active
    with pytest.raises(ValueError):
        node.calibration.ticks(node.calibration.radians((3300, 1061, 2105)))


def test_return_rejects_invalid_encoder_endpoints(make_controller):
    node, _, _ = make_controller(FIXED)
    node.measured = (-1, 1061, 2105)
    assert not start_cycle(node).success
    assert node.fixed_cycle is None and not node.path


@pytest.mark.parametrize('stage', ['moving_to_start', 'settling_start', 'line_motion', 'settling_end'])
def test_stop_in_any_stage_cancels_the_whole_sequence(make_controller, stage):
    node, tick, _ = make_controller()
    assert start_cycle(node).success
    until(node, tick, lambda: node.motion_state == stage)
    assert node.stop(Trigger.Request(), Trigger.Response()).success
    held = node.measured
    tick(100)
    assert node.motion_state == 'stopped' and node.measured == held
    assert node.fixed_cycle is None and not node.path


def test_start_error_and_motion_reset_the_settle_wait(make_controller):
    node, tick, _ = make_controller(FIXED)
    assert start_cycle(node).success
    until(node, tick, lambda: node.motion_state == 'settling_start')
    node.measured = (FIXED[0] + 11, FIXED[1], FIXED[2])
    tick(30)
    assert node.motion_state == 'settling_start'
    node.measured = FIXED
    tick(5)
    # All points are inside the arrival tolerance, but drift prevents settling.
    for offset in range(1, 10):
        node.measured = (FIXED[0] + offset, FIXED[1], FIXED[2])
        tick(2)
    assert node.motion_state == 'settling_start'
    node.measured = FIXED
    until(node, tick, lambda: node.motion_state == 'line_motion')


@pytest.mark.parametrize('overrides', [
    {'fixed_start_ticks': [4000, 1061, 2105]},
    {'fixed_line_delta_m': [0.0, 0.20, 0.0]},
])
def test_both_legs_preflight_before_any_motion_is_queued(make_controller, overrides):
    node, tick, _ = make_controller(**overrides)
    before = node.measured
    assert not start_cycle(node).success
    tick(10)
    assert node.measured == before and not node.path and node.fixed_cycle is None


def test_duplicate_and_manual_commands_do_not_replace_an_active_cycle(make_controller):
    node, _, _ = make_controller()
    assert start_cycle(node).success
    queued = list(node.path)
    assert not start_cycle(node).success
    node.joint_target(JointState(position=[0.0, 0.0, 0.0]))
    assert list(node.path) == queued and node.fixed_cycle is not None


def test_stale_or_disabled_feedback_cannot_start_cycle(make_controller):
    node, _, _ = make_controller()
    node.last_feedback_time = -1.0
    assert not start_cycle(node).success
    node.enabled = False
    node.last_feedback_time = 0.0
    assert not start_cycle(node).success
    assert not node.path


def test_disable_and_reenable_do_not_resume_sequence(make_controller):
    node, tick, _ = make_controller()
    assert start_cycle(node).success
    tick(10)
    assert node.enable(SetBool.Request(data=False), SetBool.Response()).success
    assert node.enable(SetBool.Request(data=True), SetBool.Response()).success
    held = node.measured
    tick(100)
    assert node.fixed_cycle is None and not node.path and node.measured == held


def test_deadline_fault_cancels_before_next_position_write(make_controller):
    node, tick, clock = make_controller()
    assert start_cycle(node).success
    tick(1)
    last_written = node.last_written
    clock[0] += node.fixed_timeout + 1.0
    tick()
    assert node.motion_state == 'fault' and not node.enabled
    assert node.fixed_cycle is None and not node.path
    assert node.last_written == last_written
