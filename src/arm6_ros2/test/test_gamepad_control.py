"""Measured F710 mapping and streaming-session ownership; no hardware."""

from concurrent.futures import Future
import json
import os
from pathlib import Path
import time
from unittest.mock import MagicMock

os.environ.setdefault('ROS_LOG_DIR', '/tmp/arm6_unit_test_logs')

import pytest
import rclpy
from rclpy.context import Context
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import Trigger

from arm6_ros2.gamepad_control import GamepadControl, load_profile
from arm6_ros2.jog import INTENT_SIZE

TOKEN = '1'*32


def client():
    result = MagicMock()
    result.service_is_ready.return_value = True
    result.replies = []

    def send(request):
        future = Future()
        result.replies.append(future)
        return future

    result.call_async.side_effect = send
    return result


def joy(node, **controls):
    axes = list(node.profile['baseline_axes'])
    buttons = [0]*node.profile['buttons_count']
    for name, value in controls.items():
        entry = node.mapping[name]
        if entry['kind'] == 'axis':
            axes[entry['index']] = entry['baseline']+value*entry['sign']
        else:
            buttons[entry['index']] = int(value)
    node.on_joy(Joy(axes=axes, buttons=buttons))


def feedback(node, status, token=TOKEN):
    node.on_stream_feedback(String(data=json.dumps(dict(session=token, status=status, message=status))))


def connect(node, token=TOKEN):
    node.update()
    node.start_client.replies[-1].set_result(Trigger.Response(success=True, message=token))
    node.update()
    assert node.session == token


def last_packet(node):
    return node.velocity_pub.publish.call_args.args[0]


@pytest.fixture
def pad():
    context = Context()
    rclpy.init(context=context, domain_id=183)
    profile = load_profile(Path(__file__).resolve().parents[3]/'docs/hardware/f710_mapping.json')
    node = GamepadControl(profile, context=context)
    node.start_client, node.stop_client = client(), client()
    node.velocity_pub = MagicMock()
    joy(node)
    node.on_state(String(data='holding'))
    node.on_ticks(Int32MultiArray(data=[2048]*7))
    node.enable_input()
    assert node.enabled
    yield node
    node.destroy_node()
    context.shutdown()


@pytest.mark.parametrize('controls,index,value', [
    ({'left_stick_up': 1.}, 0, 1.), ({'left_stick_up': -1.}, 0, -1.),
    ({'left_stick_right': 1.}, 1, 1.), ({'left_stick_right': -1.}, 1, -1.),
    ({'a': 1, 'left_stick_up': 1.}, 2, 1.), ({'a': 1, 'left_stick_up': -1.}, 2, -1.),
    ({'right_stick_up': 1.}, 4, 1.), ({'right_stick_up': -1.}, 4, -1.),
    ({'dpad_up': 1.}, 3, 1.), ({'dpad_down': 1.}, 3, -1.),
    ({'left_stick_up': .5, 'left_stick_right': -1.}, 1, -1.),
])
def test_measured_inputs_publish_correct_velocity_axis(pad, controls, index, value):
    connect(pad)
    joy(pad, lb=1, **controls)
    packet = last_packet(pad)
    expected = [0.]*INTENT_SIZE
    expected[index] = value
    assert list(packet.axes) == expected
    assert list(packet.buttons) == [1, 0]
    assert packet.header.frame_id == TOKEN
    pad.stop_client.call_async.assert_not_called()


@pytest.mark.parametrize('controls', [
    {'left_stick_up': .2}, {'right_stick_right': 1.}, {'a': 1, 'left_stick_right': 1.},
    {'left_stick_up': 1., 'right_stick_up': 1.}, {'dpad_up': 1., 'right_stick_up': -1.},
])
def test_deadzone_unused_axis_and_conflicting_inputs_publish_zero(pad, controls):
    connect(pad)
    joy(pad, lb=1, **controls)
    assert list(last_packet(pad).axes) == [0.]*INTENT_SIZE
    assert list(last_packet(pad).buttons) == [0, 0]


def test_held_input_streams_during_motion_without_arrival_service_calls(pad):
    connect(pad)
    joy(pad, lb=1, left_stick_up=1.)
    pad.on_state(String(data='moving'))
    pad.velocity_pub.reset_mock()
    for _ in range(100):
        pad.update()
    assert pad.velocity_pub.publish.call_count == 100
    assert all(list(c.args[0].axes) == [1.]+[0.]*(INTENT_SIZE-1)
               for c in pad.velocity_pub.publish.call_args_list)
    assert pad.start_client.call_async.call_count == 1
    pad.stop_client.call_async.assert_not_called()


def test_release_and_reversal_keep_the_same_enabled_session(pad):
    connect(pad)
    joy(pad, lb=1, left_stick_up=1.)
    pad.on_state(String(data='moving'))
    joy(pad, lb=1)
    assert list(last_packet(pad).axes) == [0.]*INTENT_SIZE
    joy(pad, lb=1, left_stick_up=-1.)
    assert last_packet(pad).axes[0] == -1.
    joy(pad, left_stick_up=-1.)
    assert list(last_packet(pad).axes) == [0.]*INTENT_SIZE
    assert pad.session == TOKEN and pad.enabled
    pad.stop_client.call_async.assert_not_called()


def test_analog_stick_deflection_controls_speed(pad):
    connect(pad)
    joy(pad, lb=1, left_stick_up=.6)
    assert last_packet(pad).axes[0] == pytest.approx(.5)
    joy(pad, lb=1, left_stick_up=1.)
    assert last_packet(pad).axes[0] == 1.


def test_a_tap_selects_each_wrist_joint_without_sending_motion(pad):
    connect(pad)
    for index in (5, 6, 3):  # J4, J5, J6
        joy(pad, a=1)
        joy(pad)
        assert not any(last_packet(pad).axes)
        joy(pad, lb=1, dpad_up=1.)
        values = list(last_packet(pad).axes)
        assert values[index] == 1. and sum(abs(v) for v in values) == 1.
        joy(pad, lb=1, dpad_down=1.)
        assert last_packet(pad).axes[index] == -1.
        joy(pad)


def test_holding_a_for_z_does_not_change_the_selected_wrist_joint(pad):
    connect(pad)
    joy(pad, a=1)  # A can be held before LB for Z movement.
    joy(pad, a=1, lb=1, left_stick_up=1.)
    assert last_packet(pad).axes[2] == 1.
    joy(pad)
    assert pad.wrist_axis == 3


@pytest.mark.parametrize('input_time', ['joy_time', 'state_time', 'tick_time'])
def test_stale_feedback_ends_only_own_session_then_reconnects_after_neutral(pad, input_time):
    connect(pad)
    joy(pad, lb=1, right_stick_up=1.)
    setattr(pad, input_time, time.monotonic()-1.)
    pad.update()
    assert pad.enabled and pad.neutral_gate and pad.end_requested
    assert list(last_packet(pad).buttons) == [0, 1]
    pad.stop_client.call_async.assert_not_called()
    feedback(pad, 'ended')
    pad.on_state(String(data='stopped'))
    pad.on_ticks(Int32MultiArray(data=[2048]*7))
    joy(pad, lb=1, right_stick_up=1.)
    pad.update()
    assert pad.neutral_gate and pad.session is None
    joy(pad, lb=1)
    connect(pad, '2'*32)
    assert not pad.neutral_gate


def test_late_start_acceptance_after_disable_can_only_end_that_session(pad):
    pad.update()
    pad.disable()
    pad.start_client.replies[0].set_result(Trigger.Response(success=True, message=TOKEN))
    pad.update()
    assert not pad.enabled
    assert list(last_packet(pad).axes) == [0.]*INTENT_SIZE
    assert list(last_packet(pad).buttons) == [0, 1]
    pad.stop_client.call_async.assert_not_called()


def test_rejected_start_does_not_stop_other_operators_motion(pad):
    pad.update()
    pad.start_client.replies[0].set_result(Trigger.Response(success=False, message='Another motion is active'))
    pad.update()
    assert pad.enabled and pad.neutral_gate and pad.session is None
    pad.stop_client.call_async.assert_not_called()


def test_b_is_an_explicit_global_stop_even_while_start_is_pending(pad):
    pad.update()
    joy(pad, b=1)
    assert not pad.enabled
    assert pad.stop_client.call_async.call_count == 1


def test_blocked_direction_feedback_does_not_disable_input(pad):
    connect(pad)
    feedback(pad, 'blocked')
    assert pad.enabled and pad.session == TOKEN and not pad.neutral_gate
    joy(pad, lb=1, left_stick_up=-1.)
    assert last_packet(pad).axes[0] == -1.


def test_unknown_session_feedback_cannot_end_current_session(pad):
    connect(pad)
    feedback(pad, 'ended', '0'*32)
    assert pad.session == TOKEN


def test_missing_stream_feedback_ends_session_without_losing_enabled_input(pad):
    connect(pad)
    pad.last_stream_feedback = time.monotonic()-1.
    pad.update()
    assert pad.enabled and pad.neutral_gate and pad.end_requested
    assert list(last_packet(pad).buttons) == [0, 1]


def test_velocity_transport_failure_keeps_supervisor_running_and_requires_neutral(pad):
    connect(pad)
    pad.velocity_pub.publish.side_effect = RuntimeError('transport error')
    joy(pad, lb=1, left_stick_up=1.)
    assert pad.enabled and pad.neutral_gate and pad.session is None
    pad.update()
    pad.stop_client.call_async.assert_not_called()


def test_start_transport_failure_pauses_without_stopping_other_motion(pad):
    pad.start_client.call_async.side_effect = RuntimeError('transport error')
    pad.update()
    assert pad.enabled and pad.neutral_gate and pad.session is None
    pad.stop_client.call_async.assert_not_called()


def test_b_retries_transport_failure_without_losing_stop_request(pad):
    pad.stop_client.call_async.side_effect = RuntimeError('transport error')
    joy(pad, b=1)
    assert not pad.enabled and pad.force_stop
    pad.stop_client.call_async.side_effect = lambda request: Future()
    pad.stop_retry_at = 0
    pad.update()
    assert pad.stop_future is not None
