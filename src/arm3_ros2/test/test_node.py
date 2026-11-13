import os
from pathlib import Path

os.environ.setdefault('ROS_LOG_DIR', '/tmp/arm3_ros2_test_logs')

import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import SetBool, Trigger
import yaml

from arm3_ros2 import dynamixel_bus
from arm3_ros2.kinematics import RADIANS_PER_TICK
from arm3_ros2.node import Controller, DEFAULT_PARAMETERS


@pytest.fixture
def make_node():
    context = Context()
    rclpy.init(context=context, domain_id=79)
    nodes = []

    def create(**parameters):
        node = Controller(context=context, parameter_overrides=[
            Parameter(name, value=value) for name, value in parameters.items()
        ])
        node.timer.cancel()
        nodes.append(node)
        return node

    yield create
    for node in reversed(nodes):
        node.destroy_node()
    context.try_shutdown()


@pytest.fixture
def fake_bus(monkeypatch):
    class FakeBus:
        instances = []
        models = (1120, 1120, 1120)

        def __init__(self, device, baud, ids):
            self.ids = ids
            self.positions = (2048, 2160, 1910)
            self.writes = []
            self.armed = False
            self.closed = False
            self.fail = False
            self.position_limits = None
            self.instances.append(self)

        def read_positions(self):
            if self.fail:
                raise OSError('USB disconnected')
            return self.positions

        def arm(self, *args, **kwargs):
            self.armed = True
            self.position_limits = ((0, 4095),) * 3
            return self.positions

        def disarm(self):
            self.armed = False
            return []

        def write_positions(self, ticks):
            self.writes.append(ticks)
            self.positions = ticks

        def close(self):
            self.closed = True

    monkeypatch.setattr(dynamixel_bus, 'DynamixelBus', FakeBus)
    return FakeBus


def enable(node, value):
    return node.enable(SetBool.Request(data=value), SetBool.Response())


def settle(node):
    for _ in range(2000):
        if not node.path:
            return
        node.tick()
    pytest.fail('Motion never completed')


def test_yaml_defaults_match_direct_run():
    config = Path(__file__).resolve().parents[1] / 'config/arm3.yaml'
    assert yaml.safe_load(config.read_text())['/**']['ros__parameters'] == DEFAULT_PARAMETERS


def test_dry_run_never_opens_bus_and_reorders_joints(make_node, fake_bus):
    node = make_node()
    assert not fake_bus.instances
    node.joint_target(JointState(name=['joint_3', 'joint_1', 'joint_2'],
                                 position=[-0.20, 0.02, 0.18]))
    settle(node)
    assert node.calibration.radians(node.measured) == pytest.approx(
        (0.02, 0.18, -0.20), abs=RADIANS_PER_TICK / 2)


def test_bad_target_does_not_replace_existing_path(make_node):
    node = make_node()
    node.joint_target(JointState(position=[0.1, 0.2, -0.2]))
    queued = list(node.path)
    for position in ([9.0, 0.0, 0.0], [0.0], [float('nan'), 0.0, 0.0]):
        node.joint_target(JointState(position=position))
        assert list(node.path) == queued
    node.joint_target(JointState(name=['joint_1'] * 3, position=[0.0] * 3))
    assert list(node.path) == queued


def test_stop_holds_and_disable_rejects_new_motion(make_node):
    node = make_node()
    node.joint_target(JointState(position=[0.4, 0.2, -0.2]))
    node.tick()
    held = node.measured
    assert node.stop(Trigger.Request(), Trigger.Response()).success
    assert not node.path
    for _ in range(10):
        node.tick()
    assert node.measured == held
    assert enable(node, False).success
    node.joint_target(JointState(position=[0.0, 0.0, 0.0]))
    assert not node.path
    assert enable(node, True).success
    assert node.measured == held


def test_joy_moves_once_per_press(make_node):
    node = make_node()
    neutral = Joy(axes=[0.0] * 6, buttons=[0] * 9)
    node.on_joy(neutral)
    rt = Joy(axes=[0.0] * 6, buttons=[0] * 7 + [1, 0])
    initial = node.measured
    node.on_joy(rt)
    settle(node)
    assert node.measured == (initial[0] + 16, initial[1], initial[2])
    node.on_joy(rt)
    assert not node.path
    node.on_joy(neutral)
    node.on_joy(rt)
    settle(node)
    assert node.measured[0] == initial[0] + 32


@pytest.mark.parametrize('confirmed,minimum,maximum', [
    (False, [100] * 3, [3900] * 3),
    (True, [0] * 3, [4095] * 3),
])
def test_hardware_starts_read_only_and_allows_uncalibrated_hold_only(
        make_node, fake_bus, confirmed, minimum, maximum):
    node = make_node(dry_run=False, calibration_confirmed=confirmed,
                     position_min_ticks=minimum, position_max_ticks=maximum)
    bus = fake_bus.instances[-1]
    node.tick()
    assert node.measured == bus.positions
    assert not node.enabled and not bus.armed and not bus.writes
    assert enable(node, True).success
    assert node.enabled and bus.armed and not node.motion_bounds_active
    node.joint_target(JointState(position=[0.0] * 3))
    assert not node.path and not bus.writes
    assert not node.run_fixed_line(Trigger.Request(), Trigger.Response()).success
    assert node.enabled and not node.path


def test_outside_motion_window_can_enable_hold_and_stop(make_node, fake_bus):
    node = make_node(dry_run=False, calibration_confirmed=True,
                     zero_ticks=[3819, 989, 2013], joint_directions=[1, -1, -1],
                     position_min_ticks=[3414, 957, 1981], position_max_ticks=[3956, 1108, 2272])
    bus = fake_bus.instances[-1]
    bus.positions = (3300, 1000, 2050)
    assert enable(node, True).success
    assert node.last_written == bus.positions
    node.tick()
    assert node.enabled and node.measured == bus.positions
    assert not node.motion_bounds_active
    node.joint_target(JointState(position=list(node.calibration.radians((3923, 1061, 2105)))))
    assert node.enabled and not node.path and not bus.writes
    assert not node.run_fixed_line(Trigger.Request(), Trigger.Response()).success
    assert node.stop(Trigger.Request(), Trigger.Response()).success
    assert bus.writes == [(3300, 1000, 2050)]  # A hold writes only the measured position.
    node.tick()
    assert node.enabled and node.measured == (3300, 1000, 2050)


def test_feedback_bounds_are_enforced_after_a_motion_is_accepted(make_node, fake_bus):
    node = make_node(dry_run=False, calibration_confirmed=True,
                     position_min_ticks=[100] * 3, position_max_ticks=[3900] * 3)
    bus = fake_bus.instances[-1]
    assert enable(node, True).success
    node.joint_target(JointState(position=[0.02, 0.18, -0.20]))
    assert node.motion_bounds_active
    bus.positions = (99, 2160, 1910)
    node.tick()
    assert not node.enabled and node.motion_state == 'fault' and not bus.writes


def test_servo_limit_compatibility_is_checked_when_motion_is_requested(make_node, fake_bus):
    node = make_node(dry_run=False, calibration_confirmed=True,
                     position_min_ticks=[100] * 3, position_max_ticks=[3900] * 3)
    bus = fake_bus.instances[-1]
    assert enable(node, True).success
    bus.position_limits = ((0, 3000),) * 3
    node.joint_target(JointState(position=[0.02, 0.18, -0.20]))
    assert node.enabled and not node.path and not bus.writes


def test_calibrated_enable_usb_fault_and_shutdown(make_node, fake_bus):
    node = make_node(dry_run=False, calibration_confirmed=True,
                     position_min_ticks=[100] * 3, position_max_ticks=[3900] * 3)
    bus = fake_bus.instances[-1]
    assert enable(node, True).success
    assert node.last_written == bus.positions
    node.joint_target(JointState(position=[0.02, 0.18, -0.20]))
    node.tick()
    assert bus.writes
    bus.fail = True
    node.tick()
    assert not node.enabled and node.measured is None and not node.path
    count = len(bus.writes)
    node.joint_target(JointState(position=[0.0] * 3))
    assert len(bus.writes) == count
    assert enable(node, False).success
    node.close()
    assert bus.closed and not bus.armed


def test_wrong_model_closes_port_and_invalid_ids_fail_early(make_node, fake_bus):
    fake_bus.models = (1000, 1120, 1120)
    with pytest.raises(ValueError, match='Expected DYNAMIXEL model'):
        make_node(dry_run=False)
    assert fake_bus.instances[-1].closed
    count = len(fake_bus.instances)
    with pytest.raises(ValueError, match='distinct'):
        make_node(dry_run=False, motor_ids=[1, 1, 3])
    assert len(fake_bus.instances) == count


def documented_parameters(**overrides):
    config = (Path(__file__).resolve().parents[3] / 'calibration' /
              'arm3_horizontal_up_20cm' / 'arm3_20cm_EXECUTE.yaml')
    parameters = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    parameters.update(overrides)
    return parameters


def test_documented_full_turn_base_accepts_real_motion(make_node, fake_bus):
    node = make_node(**documented_parameters())
    bus = fake_bus.instances[-1]
    assert node.calibration.minimum == (0, 740, 1047)
    assert node.calibration.maximum == (4095, 2700, 3347)
    assert enable(node, True).success
    node.joint_target(JointState(position=list(node.calibration.radians((2049, 2160, 1910)))))
    assert node.path and node.motion_bounds_active
    for _ in range(3):
        node.tick()  # At 0.02 rad/s the first step may round to the held encoder tick.
    assert node.enabled and bus.writes


@pytest.mark.parametrize('target', [(0, 740, 1047), (4095, 2700, 3347)])
def test_documented_endpoint_targets_are_accepted(make_node, fake_bus, target):
    node = make_node(**documented_parameters())
    assert enable(node, True).success
    node.joint_target(JointState(position=list(node.calibration.radians(target))))
    assert node.path


@pytest.mark.parametrize('target', [(2048, 739, 1910), (2048, 2701, 1910),
                                  (2048, 2160, 1046), (2048, 2160, 3348)])
def test_documented_outside_targets_are_rejected_without_replacing_path(
        make_node, fake_bus, target):
    node = make_node(**documented_parameters())
    bus = fake_bus.instances[-1]
    assert enable(node, True).success
    node.joint_target(JointState(position=list(node.calibration.radians((2049, 2160, 1910)))))
    queued = list(node.path)
    assert queued
    node.joint_target(JointState(position=list(node.calibration.radians(target))))
    assert list(node.path) == queued and not bus.writes


@pytest.mark.parametrize('current', [(3923, 2701, 2105), (3923, 1061, 1046)])
def test_outside_documented_limits_can_hold_but_cannot_start_return(
        make_node, fake_bus, current):
    node = make_node(**documented_parameters())
    bus = fake_bus.instances[-1]
    bus.positions = current
    assert enable(node, True).success
    node.tick()
    assert node.enabled and not node.motion_bounds_active
    response = node.run_fixed_line(Trigger.Request(), Trigger.Response())
    assert not response.success and 'mechanical' in response.message
    assert not node.path and node.fixed_cycle is None and not bus.writes
    assert node.enabled


def test_return_bypasses_trial_window_but_enforces_documented_feedback_limits(make_node, fake_bus):
    node = make_node(**documented_parameters(
        position_min_ticks=[3414, 957, 1981], position_max_ticks=[3956, 1108, 2272]))
    bus = fake_bus.instances[-1]
    bus.positions = (3300, 900, 1900)  # Outside the old trial window, inside mechanical bounds.
    assert enable(node, True).success
    assert node.run_fixed_line(Trigger.Request(), Trigger.Response()).success
    assert not node.motion_bounds_active
    for _ in range(3):
        node.tick()
    assert node.enabled and bus.writes
    count = len(bus.writes)
    bus.positions = (3300, 2701, 1900)
    node.tick()
    assert node.motion_state == 'fault' and not node.enabled
    assert len(bus.writes) == count and node.fixed_cycle is None


def test_software_limits_cannot_extend_beyond_documented_mechanical_limits(make_node, fake_bus):
    with pytest.raises(ValueError, match='inside the mechanical limits'):
        make_node(**documented_parameters(position_max_ticks=[4095, 3323, 3347]))
    assert not fake_bus.instances  # Configuration rejected before touching a serial port.


def test_existing_20cm_line_completes_under_documented_limits(make_node):
    node = make_node(**documented_parameters(dry_run=True))
    clock = [0.0]
    node._now = lambda: clock[0]
    node.last_feedback_time = 0.0
    assert node.run_fixed_line(Trigger.Request(), Trigger.Response()).success
    for _ in range(2000):
        clock[0] += 1.0 / node.hz
        node.tick()
        assert node.motion_state != 'fault'
        assert node.mechanical.contains_ticks(node.measured)
        if node.motion_state == 'completed':
            break
    assert node.motion_state == 'completed'
    assert node.measured == (3446, 1076, 2240)
