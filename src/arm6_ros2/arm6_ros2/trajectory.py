"""Rest-to-rest quintic segments with analytic speed and acceleration checks."""

import math

import numpy as np

from arm6_ros2.model import JOINT_NAMES, vector


class Plan:
    def __init__(self, model, start, targets, times=None):
        self.start = vector(start, 7, 'trajectory start').copy()
        model.encode(self.start)
        if not targets or len(targets) > model.p['max_waypoints']:
            raise ValueError('Expected a nonempty bounded list of waypoints')
        if times is not None and len(times) != len(targets):
            raise ValueError('Each trajectory point needs a time')
        self.points, self.times = [self.start], [0.0]
        for index, target in enumerate(targets):
            target = vector(target, 7, 'waypoint').copy()
            model.encode(target)
            # Joints are bounded coordinates, including ID11 at 0..4095 ticks.
            # Never wrap a delta modulo 2*pi: 4095 -> 0 traverses the interval.
            delta = np.abs(target-self.points[-1])
            # Max derivatives of 10s^3-15s^4+6s^5 on [0,1].
            minimum = max(float(np.max(1.875*delta/model.velocity)),
                          float(np.max(np.sqrt((10/math.sqrt(3))*delta/model.acceleration))),
                          1/model.p['rate_hz'])
            if times is None:
                end_time = self.times[-1]+minimum*1.001
            else:
                end_time = float(times[index])
                if not math.isfinite(end_time) or end_time < 0:
                    raise ValueError('Trajectory times must be finite and nonnegative')
                if index == 0 and end_time == 0:
                    if np.max(delta) > 1e-9:
                        raise ValueError('Point at t=0 must equal the measured start')
                    continue
                if end_time <= self.times[-1] or end_time-self.times[-1] < minimum-1e-8:
                    raise ValueError('Trajectory times violate order, velocity or acceleration limits')
            self.times.append(end_time)
            self.points.append(target)
        self.duration = self.times[-1]
        if self.duration > model.p['max_trajectory_duration_s']:
            raise ValueError('Trajectory exceeds maximum duration')

    def sample(self, elapsed):
        if elapsed >= self.duration:
            return self.points[-1].copy()
        index = max(1, int(np.searchsorted(self.times, max(0, elapsed), side='right')))
        fraction = (max(0, elapsed)-self.times[index-1])/(self.times[index]-self.times[index-1])
        smooth = fraction**3*(10 + fraction*(-15 + fraction*6))
        return self.points[index-1]+smooth*(self.points[index]-self.points[index-1])


def trajectory_request(model, start, message):
    """Validate ROS JointTrajectory without depending on ROS in the math layer."""
    if len(message.joint_names) != 6 or set(message.joint_names) != set(JOINT_NAMES):
        raise ValueError('Trajectory must name all six arm joints exactly once')
    if message.header.stamp.sec or message.header.stamp.nanosec:
        raise ValueError('Use a zero header stamp (start on acceptance)')
    order = [message.joint_names.index(name) for name in JOINT_NAMES]
    targets, times = [], []
    for point in message.points:
        if point.velocities or point.accelerations or point.effort:
            raise ValueError('Provide positions only; each waypoint uses rest-to-rest quintic interpolation')
        q = vector(point.positions, 6, 'trajectory positions')[order]
        if point.time_from_start.sec < 0 or not 0 <= point.time_from_start.nanosec < 10**9:
            raise ValueError('Invalid ROS duration')
        targets.append(np.r_[q, start[6]])
        times.append(point.time_from_start.sec+point.time_from_start.nanosec*1e-9)
    return Plan(model, start, targets, times)
