"""Continuous integration, speed limits and endpoint stops without hardware."""

from pathlib import Path

import numpy as np
import pytest

from arm6_ros2.config import load_parameters
from arm6_ros2.jog import ContinuousJog, intent_key
from arm6_ros2.model import ArmModel, RADIANS_PER_TICK


def setup_servo(ticks=None, load=None):
    p = load_parameters(Path(__file__).resolve().parents[1]/'config/arm6_calibration.yaml')
    model = ArmModel(p)
    actual = model.decode(ticks or [3449, 1227, 1416, 1117, 1298, 2525, 3548])
    load = np.array(load or [0, 12, 18, 0, 8, 0, 0])
    goals = tuple(np.asarray(model.encode(actual))+load)
    return model, ContinuousJog(model, p), actual, goals, load


@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('sign', [-1, 1])
def test_held_cartesian_input_moves_every_period_without_arrival_pauses(axis, sign):
    model, servo, actual, goals, load = setup_servo()
    origin = model.forward(actual[:6])
    intent = np.zeros(5)
    intent[axis] = sign
    initial_wrist_goals = goals[3:]
    previous_velocity = np.zeros(7)
    writes, halfway = 0, None
    for tick in range(100):
        target, motor, status = servo.step(intent, actual, goals, 1/30)
        assert status == 'moving' and motor is None
        assert np.all(np.abs(servo.velocity) <= model.velocity*model.p['gamepad_speed_scale']+1e-9)
        assert np.all(np.abs(servo.velocity-previous_velocity) <=
                      model.acceleration*model.p['gamepad_speed_scale']**2/30+1e-9)
        previous_velocity = servo.velocity.copy()
        if target is not None:
            goals = target
            if axis < 2:
                assert goals[3:] == initial_wrist_goals
            actual = model.decode(np.asarray(goals)-load)
            writes += 1
        if tick == 49:
            halfway = (model.forward(actual[:6])[axis, 3]-origin[axis, 3])*sign
    after = model.forward(actual[:6])
    delta = after[:3, 3]-origin[:3, 3]
    assert writes >= 90
    assert delta[axis]*sign > 0.003
    assert delta[axis]*sign > halfway*1.6
    assert np.linalg.norm(np.delete(delta, axis)) < 0.001
    if axis == 2:
        assert np.allclose(after[:3, :3], origin[:3, :3], atol=0.003)


def test_zero_input_stops_immediately_without_rebasing_loaded_goals():
    model, servo, actual, goals, load = setup_servo()
    for _ in range(20):
        target, _, _ = servo.step([0, 0, 1, 0, 0], actual, goals, 1/30)
        if target is not None:
            goals = target
            actual = model.decode(np.asarray(goals)-load)
    saved = goals
    for _ in range(30):
        target, _, status = servo.step([0]*5, actual, goals, 1/30)
        assert target is None and status == 'holding'
        assert np.all(servo.velocity == 0)
        assert goals == saved


@pytest.mark.parametrize('axis,index,sign', [(3, 5, 1), (3, 5, -1), (4, 6, 1), (4, 6, -1)])
def test_stream_stops_at_encoder_endpoint_and_allows_reverse(axis, index, sign):
    ticks = [3449, 1227, 1416, 1117, 1298, 2525, 3548]
    ticks[index] = (4090 if sign > 0 else 5) if index == 5 else (4370 if sign > 0 else 3186)
    model, servo, actual, goals, load = setup_servo(ticks, [0]*7)
    intent = np.zeros(5)
    intent[axis] = sign
    for _ in range(100):
        target, motor, status = servo.step(intent, actual, goals, 1/30)
        if target is not None:
            assert motor == index
            goals = target
            actual = model.decode(goals)
        if status == 'blocked':
            break
    else:
        pytest.fail('Did not stop at the endpoint')
    endpoint = (4095 if sign > 0 else 0) if index == 5 else (4376 if sign > 0 else 3180)
    assert goals[index] == endpoint
    for _ in range(10):
        assert servo.step(intent, actual, goals, 1/30)[0] is None
    intent[axis] *= -1
    moved = False
    for _ in range(10):
        target, _, status = servo.step(intent, actual, goals, 1/30)
        assert status == 'moving'
        if target is not None:
            moved = moved or target[index] != endpoint
            goals, actual = target, model.decode(target)
    assert moved


@pytest.mark.parametrize('intent', [[float('nan')]*5, [2, 0, 0, 0, 0], [1, 1, 0, 0, 0], [0]*4])
def test_invalid_intent_is_rejected(intent):
    with pytest.raises(ValueError):
        intent_key(intent)


def test_analog_deflection_changes_velocity_without_quantized_step_counts():
    distances = []
    for level in (.4, 1.):
        model, servo, actual, goals, load = setup_servo()
        start = actual[6]
        for _ in range(10):
            target, _, status = servo.step([0, 0, 0, 0, level], actual, goals, 1/30)
            assert status == 'moving'
            if target is not None:
                goals, actual = target, model.decode(np.asarray(target)-load)
        distances.append(actual[6]-start)
    assert distances[1] > distances[0]*1.5


def test_compensated_joint_limit_is_respected_with_load_offset():
    ticks = [3449, 1227, 1416, 1117, 1298, 4080, 3548]
    model, servo, actual, goals, load = setup_servo(ticks, [0, 0, 0, 0, 0, 10, 0])
    for _ in range(50):
        target, _, status = servo.step([0, 0, 0, 1, 0], actual, goals, 1/30)
        if target is not None:
            assert target[5] <= 4095
            goals, actual = target, model.decode(np.asarray(target)-load)
    assert status == 'blocked' and goals[5] == 4095
    assert np.abs(servo.offset[5]*RADIANS_PER_TICK) > 0


def test_extended_cartesian_hold_stops_at_workspace_singularity_instead_of_creeping_forever():
    model, servo, actual, goals, load = setup_servo(load=[0]*7)
    for _ in range(3000):
        target, _, status = servo.step([0, 0, 1, 0, 0], actual, goals, 1/30)
        if target is not None:
            goals, actual = target, model.decode(target)
        if status == 'blocked':
            break
    else:
        pytest.fail('Held Cartesian input failed to stop at the workspace boundary')
    for _ in range(10):
        assert servo.step([0, 0, 1, 0, 0], actual, goals, 1/30)[0] is None
    assert '奇异点' in servo.message or '限位' in servo.message
