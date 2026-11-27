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


def test_legacy_encoder_defaults_retained_pending_cad_alignment():
    assert DEFAULTS['zero_ticks'][:3] == [3819, 989, 2013]
    assert DEFAULTS['joint_directions'][:3] == [1, -1, -1]
    assert DEFAULTS['position_min_ticks'][:3] == [0, 740, 1047]
    assert DEFAULTS['position_max_ticks'][:3] == [4095, 2700, 3347]
    assert DEFAULTS['motor_ids'][:3] == [1, 2, 3]
    assert DEFAULTS['joint_velocity_limits_rad_s'][:3] == [0.02]*3
    assert DEFAULTS['calibration_confirmed'] is False
    assert DEFAULTS['geometry_confirmed'] is False


def test_cad_saved_pose_matches_assembly_adapter_origin():
    model = ArmModel()
    # Independent assembly reference, in mm; allow 1 micrometre for rounded DH rows.
    assert np.allclose(model.forward(np.zeros(6))[:3, 3] * 1000,
                       [9.2073, 469.0624, 154.4533], atol=0.001, rtol=0)
    assert np.allclose(model.tcp, np.eye(4))


def test_cad_spherical_wrist_axes_and_adapter_distance():
    model = ArmModel()
    saved = model.chain(np.zeros(6))
    assert np.allclose(saved[4][:3, 2], saved[6][:3, 2], atol=1e-12)
    random = np.random.default_rng(34)
    for _ in range(100):
        chain = model.chain(random.uniform(model.lower, model.upper))
        wrist = chain[4][:3, 3]
        for index in (4, 5, 6):
            # All three wrist axis lines pass through the same point.
            assert np.linalg.norm(np.cross(wrist-chain[index][:3, 3],
                                           chain[index][:3, 2])) < 1e-12
        assert np.allclose(chain[6][:3, 3]-wrist, 0.0735*chain[6][:3, 2], atol=1e-12)


@pytest.mark.parametrize('tool', [{}, {'tcp_xyz_m': [0.01, -0.02, 0.08],
                                     'tcp_rpy_rad': [0.2, -0.1, 0.3]}])
def test_modified_dh_matches_original_standard_cad_model(tool):
    model = ArmModel(tool)
    # Frozen original CAD standard-DH rows, independent of the new MDH defaults.
    rows = [
        (0, 0.0495, 90, 87.4126),
        (0.2489377, 0, 0, 60.5269),
        (0.0000077, -0.011977, 90, 11.5950),
        (0, 0.2905981, 90, -90),
        (0, 0, -90, 0),
        (0, 0.0735, 0, 0),
    ]
    random = np.random.default_rng(62)
    for joints in [np.zeros(6), model.lower, model.upper,
                   *random.uniform(model.lower, model.upper, (100, 6))]:
        reference = np.eye(4)
        for q, (a, d, alpha_deg, offset_deg) in zip(joints, rows):
            theta, alpha = q + math.radians(offset_deg), math.radians(alpha_deg)
            c, s, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
            standard = np.array([[c, -s*ca, s*sa, a*c], [s, c*ca, -c*sa, a*s],
                                 [0, sa, ca, d], [0, 0, 0, 1]])
            reference = reference @ standard
        # Original a6=alpha6=0, so both frame 6 definitions coincide.
        assert np.allclose(model.forward(joints), reference @ model.tcp, atol=1e-12, rtol=0)


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

