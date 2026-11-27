"""Configurable standard-DH kinematics, encoder mapping and bounded pose IK."""

import math

import numpy as np

from arm6_ros2.config import DEFAULTS

RADIANS_PER_TICK = 2 * math.pi / 4096

JOINT_NAMES = tuple(f'joint_{i}' for i in range(1, 7))


def vector(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain {size} finite numbers')
    return result


def rotation_vector(rotation):
    """SO(3) logarithm, including rotations close to pi."""
    angle = math.acos(float(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))
    skew = np.array([rotation[2, 1] - rotation[1, 2],
                     rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])
    if angle < 1e-7:
        return skew / 2
    if math.pi - angle < 1e-5:
        values, axes = np.linalg.eigh((rotation + rotation.T) / 2)
        axis = axes[:, np.argmax(values)]
        if np.dot(axis, skew) < 0:
            axis = -axis
        return angle * axis
    return angle * skew / (2 * math.sin(angle))


def rotation_exp(value):
    angle = np.linalg.norm(value)
    if angle < 1e-12:
        return np.eye(3)
    x, y, z = value / angle
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def pose_matrix(position, quaternion):
    """ROS quaternion order is x,y,z,w; accept normalized quaternions only."""
    x, y, z, w = vector(quaternion, 4, 'quaternion')
    if abs(np.linalg.norm([x, y, z, w]) - 1) > 1e-3:
        raise ValueError('Quaternion must have unit norm')
    x, y, z, w = np.array([x, y, z, w]) / np.linalg.norm([x, y, z, w])
    transform = np.eye(4)
    transform[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    transform[:3, 3] = vector(position, 3, 'position')
    return transform


def quaternion_from_matrix(rotation):
    rotvec = rotation_vector(rotation)
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(rotvec * (math.sin(angle / 2) / angle)) + (math.cos(angle / 2),)


def dh_transform(a, d, alpha, theta):
    c, s, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[c, -s*ca, s*sa, a*c], [s, c*ca, -c*sa, a*s],
                     [0, sa, ca, d], [0, 0, 0, 1]], dtype=float)


class ArmModel:
    def __init__(self, parameters=None):
        self.p = {**DEFAULTS, **(parameters or {})}
        p = self.p
        self.ids = tuple(p['motor_ids'])
        if (len(self.ids) != 7 or len(set(self.ids)) != 7
                or any(type(i) is not int or not 0 <= i <= 252 for i in self.ids)):
            raise ValueError('motor_ids must contain seven distinct IDs (six joints then gripper)')
        self.models = tuple(p['expected_model_numbers'])
        # Only the verified common X-series control table is supported by this driver.
        if len(self.models) != 7 or any(m not in (1120, 1020, 1000) for m in self.models):
            raise ValueError('Supported model numbers: XM540-W270=1120, XM430-W350=1020, '
                             'XH430-W350=1000')
        self.zero = vector(p['zero_ticks'], 6, 'zero_ticks')
        self.sign = vector(p['joint_directions'], 6, 'joint_directions')
        self.tick_min = vector(p['position_min_ticks'], 6, 'position_min_ticks')
        self.tick_max = vector(p['position_max_ticks'], 6, 'position_max_ticks')
        if (not np.all(np.isin(self.sign, [-1, 1]))
                or any(np.any(v != np.round(v)) for v in (self.zero, self.tick_min, self.tick_max))
                or np.any(self.zero < 0) or np.any(self.zero > 4095)
                or np.any(self.tick_min < 0) or np.any(self.tick_max > 4095)
                or np.any(self.tick_min >= self.tick_max)):
            raise ValueError('Invalid single-turn calibration or motion limits')
        ends = np.stack([(self.tick_min-self.zero)*self.sign*RADIANS_PER_TICK,
                         (self.tick_max-self.zero)*self.sign*RADIANS_PER_TICK])
        self.lower, self.upper = ends.min(axis=0), ends.max(axis=0)
        self.a = vector(p['dh_a_m'], 6, 'dh_a_m')
        self.d = vector(p['dh_d_m'], 6, 'dh_d_m')
        self.alpha = vector(p['dh_alpha_rad'], 6, 'dh_alpha_rad')
        self.offset = vector(p['dh_offsets_rad'], 6, 'dh_offsets_rad')
        self.tcp = np.eye(4)
        self.tcp[:3, 3] = vector(p['tcp_xyz_m'], 3, 'tcp_xyz_m')
        roll, pitch, yaw = vector(p['tcp_rpy_rad'], 3, 'tcp_rpy_rad')
        self.tcp[:3, :3] = (rotation_exp(np.array([0, 0, yaw]))
                           @ rotation_exp(np.array([0, pitch, 0]))
                           @ rotation_exp(np.array([roll, 0, 0])))
        closed, opened = p['gripper_closed_ticks'], p['gripper_open_ticks']
        self.width = p['gripper_max_width_m']
        if (any(type(t) is not int or not 0 <= t <= 4095 for t in (closed, opened))
                or closed == opened or not math.isfinite(self.width) or self.width <= 0):
            raise ValueError('Invalid gripper endpoint calibration')
        self.closed, self.opened = closed, opened
        self.velocity = np.r_[vector(p['joint_velocity_limits_rad_s'], 6, 'velocity limits'),
                              p['gripper_speed_m_s']]
        self.acceleration = np.r_[vector(p['joint_acceleration_limits_rad_s2'], 6, 'acceleration limits'),
                                  p['gripper_acceleration_m_s2']]
        if (not np.all(np.isfinite(self.velocity)) or not np.all(np.isfinite(self.acceleration))
                or np.any(self.velocity <= 0) or np.any(self.acceleration <= 0)):
            raise ValueError('Velocity and acceleration limits must be positive and finite')

    def decode(self, ticks):
        ticks = vector(ticks, 7, 'encoder feedback')
        return np.r_[(ticks[:6]-self.zero)*self.sign*RADIANS_PER_TICK,
                     (ticks[6]-self.closed)*self.width/(self.opened-self.closed)]

    def encode(self, state):
        state = vector(state, 7, 'joint and gripper target')
        if (np.any(state[:6] < self.lower-1e-10) or np.any(state[:6] > self.upper+1e-10)
                or not -1e-10 <= state[6] <= self.width+1e-10):
            raise ValueError('Joint or gripper target is outside calibrated motion limits')
        ticks = np.r_[self.zero + state[:6]*self.sign/RADIANS_PER_TICK,
                      self.closed + state[6]*(self.opened-self.closed)/self.width]
        return tuple(int(t) for t in np.rint(ticks))

    def chain(self, joints):
        q = vector(joints, 6, 'arm joints')
        transforms = [np.eye(4)]
        for a, d, alpha, theta in zip(self.a, self.d, self.alpha, q+self.offset):
            transforms.append(transforms[-1] @ dh_transform(a, d, alpha, theta))
        return transforms

    def forward(self, joints):
        return self.chain(joints)[-1] @ self.tcp

    def jacobian(self, joints):
        chain = self.chain(joints)
        end = (chain[-1] @ self.tcp)[:3, 3]
        return np.stack([np.r_[np.cross(t[:3, 2], end-t[:3, 3]), t[:3, 2]]
                         for t in chain[:-1]], axis=1)

    def inverse(self, target, seed):
        target = np.asarray(target, dtype=float)
        if (target.shape != (4, 4) or not np.all(np.isfinite(target))
                or not np.allclose(target[3], [0, 0, 0, 1])
                or not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-6)
                or abs(np.linalg.det(target[:3, :3])-1) > 1e-6):
            raise ValueError('Target must be a rigid 4x4 pose')
        q = vector(seed, 6, 'IK seed').copy()
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError('IK seed is outside joint limits')
        for _ in range(250):
            current = self.forward(q)
            error = np.r_[target[:3, 3]-current[:3, 3],
                          rotation_vector(target[:3, :3] @ current[:3, :3].T)]
            if np.linalg.norm(error[:3]) < 1e-5 and np.linalg.norm(error[3:]) < 1e-4:
                return q
            # Metres and radians receive a characteristic-length weighting.
            weights = np.array([1, 1, 1, 0.2, 0.2, 0.2])
            jac = self.jacobian(q) * weights[:, None]
            rhs = error * weights
            step = jac.T @ np.linalg.solve(jac @ jac.T + 1e-6*np.eye(6), rhs)
            step *= min(1.0, 0.15 / max(np.max(np.abs(step)), 1e-12))
            improved = False
            for scale in (1.0, 0.5, 0.25, 0.1):
                candidate = np.clip(q+scale*step, self.lower, self.upper)
                pose = self.forward(candidate)
                new_error = np.r_[target[:3, 3]-pose[:3, 3],
                                  rotation_vector(target[:3, :3] @ pose[:3, :3].T)]
                if np.linalg.norm(new_error*weights) < np.linalg.norm(rhs):
                    q = candidate
                    improved = True
                    break
            if not improved:
                break
        raise ValueError('No continuous IK solution from this seed within joint limits')

    def cartesian_path(self, seed, target):
        """Preflight a TCP line with shortest-path orientation interpolation."""
        q = vector(seed, 6, 'seed').copy()
        start = self.forward(q)
        delta = target[:3, 3] - start[:3, 3]
        rotation = rotation_vector(target[:3, :3] @ start[:3, :3].T)
        count = max(1, math.ceil(np.linalg.norm(delta)/self.p['cartesian_resolution_m']),
                    math.ceil(np.linalg.norm(rotation)/self.p['cartesian_resolution_rad']))
        if count > self.p['max_waypoints']:
            raise ValueError('Cartesian path has too many waypoints')
        result = []
        for i in range(1, count+1):
            fraction = i/count
            pose = np.eye(4)
            pose[:3, 3] = start[:3, 3]+fraction*delta
            pose[:3, :3] = rotation_exp(fraction*rotation) @ start[:3, :3]
            following = self.inverse(pose, q)
            if np.max(np.abs(following-q)) > self.p['max_ik_joint_step_rad']:
                raise ValueError('Cartesian path crosses an IK branch or singularity')
            result.append(following)
            q = following
        return result
