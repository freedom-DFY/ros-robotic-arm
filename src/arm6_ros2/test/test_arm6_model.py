"""Numerical checks independent of ROS or physical motors."""

from pathlib import Path
import math

import numpy as np
import pytest
import yaml

from arm6_ros2.config import DEFAULTS, load_parameters
from arm6_ros2.model import (ArmModel, pose_matrix, quaternion_from_matrix,
                            rotation_exp, rotation_vector)
from arm6_ros2.trajectory import Plan

REFERENCE_TICKS = [3923, 1061, 2105, 2048, 2300, 2048, 2200]


def test_first_three_axes_match_recorded_calibration_and_original_fk():
    model = ArmModel()
    assert DEFAULTS['zero_ticks'][:3] == [3819, 989, 2013]
    assert DEFAULTS['joint_directions'][:3] == [1, -1, -1]
    assert DEFAULTS['position_min_ticks'][:3] == [0, 740, 1047]
    assert DEFAULTS['position_max_ticks'][:3] == [4095, 2700, 3347]
    assert DEFAULTS['motor_ids'][:3] == [1, 2, 3]
    assert DEFAULTS['joint_velocity_limits_rad_s'][:3] == [0.02]*3
    random = np.random.default_rng(34)
    for _ in range(100):
        joints = random.uniform(model.lower, model.upper)
        t1, t2, t3 = joints[:3] + np.array([0, 0, math.pi/2])
        radial = 0.24606*math.cos(t2) + 0.24225*math.cos(t2-t3)
        expected = [radial*math.cos(t1), radial*math.sin(t1),
                    -0.24606*math.sin(t2)-0.24225*math.sin(t2-t3)]
        assert np.allclose(model.chain(joints)[3][:3, 3], expected, atol=1e-12)


def test_yaml_matches_defaults():
    path = Path(__file__).resolve().parents[1]/'config/arm6.yaml'
    assert yaml.safe_load(path.read_text())['/**']['ros__parameters'] == DEFAULTS


def test_configuration_sections_and_obsolete_options(tmp_path):
    path = tmp_path/'arm6.yaml'
    path.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {'baud_rate': 57600}},
        'arm6': {'ros__parameters': {'baud_rate': 1000000}},
        '/arm6': {'ros__parameters': {'device': '/test/only'}},
    }))
    parameters = load_parameters(path)
    assert parameters['baud_rate'] == 1000000
    assert parameters['device'] == '/test/only'
    path.write_text('/**:\n  ros__parameters:\n    dry_run: true\n')
    with pytest.raises(ValueError, match='Unknown arm6 parameters'):
        load_parameters(path)
    path.write_text('unrelated: {}\n')
    with pytest.raises(ValueError, match='Expected ROS 2 YAML'):
        load_parameters(path)


def test_encoding_roundtrip_and_reversed_gripper():
    for parameters in ({}, {'gripper_open_ticks': 1800, 'gripper_closed_ticks': 2600}):
        model = ArmModel(parameters)
        for raw in ((0, 740, 1047, 1024, 1024, 1024, 1800),
                    (4095, 2700, 3347, 3072, 3072, 3072, 2600), REFERENCE_TICKS):
            assert model.encode(model.decode(raw)) == tuple(raw)
        assert model.encode(np.r_[np.zeros(6), 0])[6] == model.closed
        assert model.encode(np.r_[np.zeros(6), model.width])[6] == model.opened
        for state in (np.r_[np.zeros(6), -0.001], np.zeros(6), np.full(7, np.nan)):
            with pytest.raises(ValueError):
                model.encode(state)


@pytest.mark.parametrize('settings', [
    {'motor_ids': [1, 2, 3, 4, 5, 6, 6]}, {'expected_model_numbers': [1120]*6+[999]},
    {'dh_a_m': [0]*5}, {'dh_offsets_rad': [0, 0, 0, 0, 0, float('nan')]},
    {'joint_directions': [1, 1, 1, 1, 1, 0]}, {'position_min_ticks': [0]*5+[4096]},
    {'gripper_open_ticks': 1800}, {'gripper_speed_m_s': float('nan')},
])
def test_invalid_configuration(settings):
    with pytest.raises(ValueError):
        ArmModel(settings)


def test_jacobian_matches_numerical_pose_derivative():
    model = ArmModel({'tcp_xyz_m': [0.01, -0.02, 0.12], 'tcp_rpy_rad': [0.2, -0.1, 0.3]})
    q = model.decode(REFERENCE_TICKS)[:6]
    current = model.forward(q)
    numeric = np.zeros((6, 6))
    for i in range(6):
        moved = q.copy()
        moved[i] += 1e-7
        pose = model.forward(moved)
        numeric[:, i] = np.r_[(pose[:3, 3]-current[:3, 3])/1e-7,
                               rotation_vector(pose[:3, :3] @ current[:3, :3].T)/1e-7]
    assert np.allclose(numeric, model.jacobian(q), atol=2e-7)


def test_ik_roundtrip_nearby_and_unreachable_pose():
    model = ArmModel()
    seed = model.decode(REFERENCE_TICKS)[:6]
    random = np.random.default_rng(21)
    for _ in range(35):
        target_q = seed+random.uniform(-0.05, 0.05, 6)
        target = model.forward(target_q)
        result = model.inverse(target, seed)
        actual = model.forward(result)
        assert np.linalg.norm(actual[:3, 3]-target[:3, 3]) < 1e-5
        assert np.linalg.norm(rotation_vector(actual[:3, :3] @ target[:3, :3].T)) < 1e-4
    target = np.eye(4)
    target[0, 3] = 5
    with pytest.raises(ValueError, match='IK solution'):
        model.inverse(target, seed)


@pytest.mark.parametrize('angle', [0, 1e-8, 0.3, np.pi-1e-8, np.pi])
def test_quaternion_and_rotation_roundtrip(angle):
    axis = np.array([1, -2, 3])/np.sqrt(14)
    rotation = rotation_exp(angle*axis)
    q = quaternion_from_matrix(rotation)
    assert np.allclose(pose_matrix([1, 2, 3], q)[:3, :3], rotation, atol=1e-7)
    with pytest.raises(ValueError):
        pose_matrix([0, 0, 0], [0, 0, 0, 0])


def test_cartesian_path_position_and_orientation_are_preflighted():
    model = ArmModel()
    start = model.decode(REFERENCE_TICKS)[:6]
    target = model.forward(start+np.array([-0.02, 0.02, -0.01, 0.02, -0.02, 0.01]))
    path = model.cartesian_path(start, target)
    origin = model.forward(start)
    for i, q in enumerate(path, 1):
        pose = model.forward(q)
        expected = origin[:3, 3]+(target[:3, 3]-origin[:3, 3])*i/len(path)
        assert np.linalg.norm(pose[:3, 3]-expected) < 1e-5
    assert np.allclose(model.forward(path[-1]), target, atol=1e-4)


def test_quintic_speed_acceleration_and_bounds():
    model = ArmModel()
    start = model.decode(REFERENCE_TICKS)
    end = start+np.array([-0.05, 0.03, 0.02, 0.1, -0.1, 0.1, 0.01])
    plan = Plan(model, start, [end])
    times = np.linspace(0, plan.duration, 2001)
    samples = np.stack([plan.sample(t) for t in times])
    velocity = np.diff(samples, axis=0)/(times[1]-times[0])
    acceleration = np.diff(velocity, axis=0)/(times[1]-times[0])
    assert np.all(np.max(np.abs(velocity), axis=0) <= model.velocity+1e-7)
    assert np.all(np.max(np.abs(acceleration), axis=0) <= model.acceleration+1e-7)
    assert np.array_equal(plan.sample(-10), start)
    assert np.array_equal(plan.sample(plan.duration+1), end)
    for sample in samples[::50]:
        model.encode(sample)
    for times in ([0], [0.001], [-1], [float('nan')]):
        with pytest.raises(ValueError):
            Plan(model, start, [end], times)

