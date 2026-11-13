import math

import pytest

from arm3_ros2.cartesian_step import prepare_relative_line


@pytest.fixture
def parameters():
    return {
        'dry_run': False, 'calibration_confirmed': True, 'motor_ids': [1, 2, 3],
        'zero_ticks': [3819, 989, 2013], 'joint_directions': [1, -1, -1],
        'position_min_ticks': [3414, 957, 1981], 'position_max_ticks': [3956, 1108, 2272],
        'upper_arm_m': 0.24606, 'forearm_m': 0.24225, 'base_height_m': 0.0,
        'dh_offsets_rad': [0.0, 0.0, math.pi / 2], 'base_frame': 'arm3_reference',
        'path_resolution_m': 0.001, 'max_path_joint_step_rad': 0.03,
    }


def test_twenty_cm_line_starts_from_current_feedback(parameters):
    plan = prepare_relative_line(parameters, [3923, 1061, 2105], (0.0, -0.2, 0.0))
    assert plan['expected_end_ticks'] == [3446, 1076, 2240]
    assert plan['length_mm'] == pytest.approx(200.0)
    delta = [b - a for a, b in zip(plan['start_xyz_m'], plan['target_xyz_m'])]
    assert delta == pytest.approx([0.0, -0.2, 0.0])
    changed = prepare_relative_line(parameters, [3921, 1060, 2104], (0.0, -0.2, 0.0))
    assert changed['target_xyz_m'] != plan['target_xyz_m']
    assert [b - a for a, b in zip(changed['start_xyz_m'], changed['target_xyz_m'])] == pytest.approx(delta)


@pytest.mark.parametrize('field,value', [('dry_run', True), ('calibration_confirmed', False),
                                       ('motor_ids', [4, 5])])
def test_rejects_simulated_unconfirmed_or_wrong_motor_node(parameters, field, value):
    parameters[field] = value
    with pytest.raises(ValueError):
        prepare_relative_line(parameters, [3923, 1061, 2105], (0.0, -0.2, 0.0))


@pytest.mark.parametrize('delta', [(0.0, 0.0, 0.0), (0.0, -0.21, 0.0),
                                  (0.0, float('nan'), 0.0), (0.0, 0.16, 0.12)])
def test_rejects_invalid_or_unreachable_displacement(parameters, delta):
    with pytest.raises(ValueError):
        prepare_relative_line(parameters, [3923, 1061, 2105], delta)


def test_refuses_path_outside_loaded_bounds(parameters):
    parameters['position_min_ticks'][0] = 3787
    with pytest.raises(ValueError):
        prepare_relative_line(parameters, [3923, 1061, 2105], (0.0, -0.2, 0.0))
