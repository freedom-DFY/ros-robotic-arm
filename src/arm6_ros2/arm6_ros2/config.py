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
    # User-taught encoder pose, independent of CAD zero_ticks. Gripper is held.
    'home_on_enable': True,
    'home_confirmed': True,
    'home_ticks': [3449, 1227, 1416, 1117, 1298, 2525],
    'home_arrival_tolerance_rad': math.radians(3.0),
    'calibration_jog_step_ticks': 20,
    'gamepad_speed_scale': 3.0,
    'gamepad_cartesian_speed_m_s': 0.01,
    'gamepad_command_timeout_s': 0.25,
    # J1..J6 then gripper; the gripper ID is confirmed by the user.
    'motor_ids': [1, 2, 3, 5, 4, 11, 12],
    'expected_model_numbers': [1120, 1120, 1120, 1000, 1000, 1000, 1000],
    # Legacy encoder values, not yet aligned to the CAD saved pose/local +Z axes.
    'zero_ticks': [3819, 989, 2013, 2048, 2048, 2048],
    'joint_directions': [1, -1, -1, 1, 1, 1],
    # J1/J2 retain legacy bounds; ID3 lower bound is the measured holding pose.
    # ID5: -145..45 deg about tick 2048, sign +1;
    # round the lower tick inward (ceil(2048 - 145*4096/360) = 399).
    # ID4 and ID11: bounded 0..4095; endpoints never wrap into another turn.
    'position_min_ticks': [0, 740, 1043, 399, 0, 0],
    'position_max_ticks': [4095, 2700, 3347, 2560, 4095, 4095],
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
    # ID12 opens across encoder zero: 3180 -> 4095 -> 0 -> 280 (1196 ticks).
    # Only this motor uses bounded Extended Position Mode 4; J1..6 stay Mode 3.
    'gripper_closed_ticks': 3180,
    'gripper_open_ticks': 280,
    'gripper_wraps_zero': True,
    'gripper_max_width_m': 0.06,
    'gripper_speed_m_s': 0.01,
    'gripper_goal_pwm_limit': 132,  # About 15% duty, not a calibrated grip force.
    'gripper_contact_preload_ticks': 5,
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

# Old trial YAMLs may still contain these settings. Continuous control never
# used them, so ignore only these names while rejecting other unknown keys.
RETIRED_GAMEPAD_PARAMETERS = frozenset((
    'gamepad_cartesian_step_m', 'gamepad_jog_step_ticks', 'gamepad_settle_time_s',
))

def load_parameters(path):
    """Load wildcard and named ROS 2 sections, rejecting unknown options."""
    content = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    result = dict(DEFAULTS)
    found = False
    for name in ('/**', 'arm6', '/arm6'):
        if name in content:
            result.update({key: value for key, value in content[name].get('ros__parameters', {}).items()
                           if key not in RETIRED_GAMEPAD_PARAMETERS})
            found = True
    if not found:
        raise ValueError('Expected ROS 2 YAML section /**, arm6 or /arm6')
    unknown = set(result)-set(DEFAULTS)
    if unknown:
        raise ValueError(f'Unknown arm6 parameters: {sorted(unknown)}')
    return result
