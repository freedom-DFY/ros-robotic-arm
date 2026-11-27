"""CAD geometry converted to Craig modified DH; encoder alignment is still pending."""

import math
from pathlib import Path

import yaml

DEFAULTS = {
    'device': '/dev/ttyUSB0',
    'baud_rate': 1000000,
    'base_frame': 'arm6_cad',
    'calibration_confirmed': False,
    'geometry_confirmed': False,
    # J1..J6 then gripper. ID7 is unmeasured: replace after an isolated scan.
    'motor_ids': [1, 2, 3, 5, 4, 11, 7],
    'expected_model_numbers': [1120, 1120, 1120, 1000, 1000, 1000, 1000],
    # Legacy encoder values, not yet aligned to the CAD saved pose/local +Z axes.
    'zero_ticks': [3819, 989, 2013, 2048, 2048, 2048],
    'joint_directions': [1, -1, -1, 1, 1, 1],
    'position_min_ticks': [0, 740, 1047, 1024, 1024, 1024],
    'position_max_ticks': [4095, 2700, 3347, 3072, 3072, 3072],
    # CRAIG MODIFIED DH: Rx(alpha_prev) Tx(a_prev) Rz(q + offset) Tz(d).
    # CAD assembly frame; q=0 is the saved pose, positive about servo local +Z.
    # Per row i: a[i-1], alpha[i-1], d[i], theta_offset[i].
    'dh_a_m': [0.0, 0.0, 0.2489377, 0.0000077, 0.0, 0.0],
    'dh_d_m': [0.0495, 0.0, -0.011977, 0.2905981, 0.0, 0.0735],
    'dh_alpha_rad': [0.0, math.pi / 2, 0.0, math.pi / 2, math.pi / 2, -math.pi / 2],
    'dh_offsets_rad': [math.radians(87.4126), math.radians(60.5269),
                       math.radians(11.5950), -math.pi / 2, 0.0, 0.0],
    # Frame 6 is the gripper adapter CAD origin; d6 already includes 73.5 mm.
    # Add the measured grasp-point transform here after tool calibration.
    'tcp_xyz_m': [0.0, 0.0, 0.0],
    'tcp_rpy_rad': [0.0, 0.0, 0.0],
    'gripper_closed_ticks': 1800,
    'gripper_open_ticks': 2600,
    'gripper_max_width_m': 0.06,
    'gripper_speed_m_s': 0.01,
    'joint_velocity_limits_rad_s': [0.02, 0.02, 0.02, 0.10, 0.10, 0.10],
    'joint_acceleration_limits_rad_s2': [0.04, 0.04, 0.04, 0.20, 0.20, 0.20],
    'gripper_acceleration_m_s2': 0.02,
    'profile_acceleration': 5,
    'profile_velocity': 12,
    'rate_hz': 30.0,
    'feedback_timeout_s': 0.25,
    'max_tracking_error_rad': 0.12,
    'gripper_tracking_error_m': 0.006,
    'arrival_tolerance_rad': 0.008,
    'gripper_arrival_tolerance_m': 0.001,
    'settle_time_s': 0.2,
    'arrival_timeout_s': 5.0,
    'cartesian_resolution_m': 0.005,
    'cartesian_resolution_rad': 0.05,
    'max_ik_joint_step_rad': 0.25,
    'max_trajectory_duration_s': 300.0,
    'max_waypoints': 2000,
}


def load_parameters(path):
    """Load wildcard and named ROS 2 sections; reject obsolete/unknown options."""
    content = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    result = dict(DEFAULTS)
    found = False
    for name in ('/**', 'arm6', '/arm6'):
        if name in content:
            result.update(content[name].get('ros__parameters', {}))
            found = True
    if not found:
        raise ValueError('Expected ROS 2 YAML section /**, arm6 or /arm6')
    unknown = set(result)-set(DEFAULTS)
    if unknown:
        raise ValueError(f'Unknown arm6 parameters: {sorted(unknown)}')
    return result
