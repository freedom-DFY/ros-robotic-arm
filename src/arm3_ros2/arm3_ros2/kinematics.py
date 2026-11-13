"""Position of the original J4 mounting point, in metres."""

import math

TAU = 2 * math.pi
RADIANS_PER_TICK = TAU / 4096


class Calibration:
    def __init__(self, zero, direction, minimum, maximum):
        self.zero, self.direction = tuple(zero), tuple(direction)
        self.minimum, self.maximum = tuple(minimum), tuple(maximum)
        for values in (self.zero, self.direction, self.minimum, self.maximum):
            if len(values) != 3 or any(type(value) is not int for value in values):
                raise ValueError("Calibration arrays must contain three integers")
        for z, sign, lo, hi in zip(self.zero, self.direction, self.minimum, self.maximum):
            if not (0 <= z <= 4095 and sign in (-1, 1) and 0 <= lo < hi <= 4095):
                raise ValueError("Invalid encoder zero, direction or mechanical limit")
        self.lower, self.upper = zip(*(
            sorted(((lo - z) * sign * RADIANS_PER_TICK, (hi - z) * sign * RADIANS_PER_TICK))
            for z, sign, lo, hi in zip(self.zero, self.direction, self.minimum, self.maximum)
        ))

    def radians(self, ticks):
        return tuple((t - z) * sign * RADIANS_PER_TICK
                     for t, z, sign in zip(ticks, self.zero, self.direction))

    def ticks(self, radians):
        if len(radians) != 3 or not all(
            math.isfinite(q) and lo - 1e-9 <= q <= hi + 1e-9
            for q, lo, hi in zip(radians, self.lower, self.upper)
        ):
            raise ValueError("Joint target is outside the calibrated joint limits")
        return tuple(z + int(round(q * sign / RADIANS_PER_TICK))
                     for q, z, sign in zip(radians, self.zero, self.direction))

    def contains_ticks(self, ticks):
        return len(ticks) == 3 and all(
            lo <= tick <= hi for tick, lo, hi in zip(ticks, self.minimum, self.maximum)
        )


class WristKinematics:
    def __init__(self, upper_arm, forearm, base_height, offsets, calibration):
        self.a2, self.a3, self.base_height = upper_arm, forearm, base_height
        self.offsets = tuple(offsets)
        self.calibration = calibration
        if (len(self.offsets) != 3
                or not all(map(math.isfinite, (upper_arm, forearm, base_height, *self.offsets)))
                or upper_arm <= 0 or forearm <= 0):
            raise ValueError("Invalid link geometry")

    def forward(self, joints):
        # The original modified DH table uses alpha_3 = pi.
        t1, t2, t3 = (q + offset for q, offset in zip(joints, self.offsets))
        radial = self.a2 * math.cos(t2) + self.a3 * math.cos(t2 - t3)
        return (radial * math.cos(t1), radial * math.sin(t1),
                self.base_height - self.a2 * math.sin(t2) - self.a3 * math.sin(t2 - t3))

    def inverse(self, point, seed):
        if len(point) != 3 or not all(map(math.isfinite, point)):
            raise ValueError("Expected three finite Cartesian coordinates")
        x, y, z = point
        z -= self.base_height
        radius = math.hypot(x, y)
        distance = math.hypot(radius, z)
        if not abs(self.a2 - self.a3) - 1e-10 <= distance <= self.a2 + self.a3 + 1e-10:
            raise ValueError("Wrist target is outside the geometric workspace")

        c3 = (distance**2 - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)
        elbow = math.acos(max(-1.0, min(1.0, c3)))
        if radius < 1e-10:
            base_options = ((seed[0] + self.offsets[0], 0.0),)
        else:
            yaw = math.atan2(y, x)
            base_options = ((yaw, radius), (yaw + math.pi, -radius))
        candidates = []
        for t1, radial in base_options:
            for t3 in (elbow, -elbow):
                t2 = (math.atan2(self.a3 * math.sin(t3), self.a2 + self.a3 * math.cos(t3))
                      - math.atan2(z, radial))
                if distance < 1e-10:
                    # Equal lengths folded on themselves leave shoulder angle free.
                    t2 = seed[1] + self.offsets[1]
                candidate = self._fit_limits((t1, t2, t3))
                if candidate is not None and math.dist(self.forward(candidate), point) < 1e-8:
                    candidates.append(candidate)
        if not candidates:
            raise ValueError("No wrist solution within the calibrated joint limits")
        return min(candidates, key=lambda q: math.dist(q, seed))

    def _fit_limits(self, dh_angles):
        angles = []
        for t, offset, lo, hi in zip(
            dh_angles, self.offsets, self.calibration.lower, self.calibration.upper
        ):
            angle = t - offset
            angle += TAU * math.ceil((lo - angle - 1e-10) / TAU)
            if angle > hi + 1e-10:
                return None
            angles.append(max(lo, min(hi, angle)))
        return tuple(angles)

    def line_path(self, start, target, resolution, max_joint_jump):
        """Preflight a line and reject it if any waypoint breaks continuity or limits."""
        self.inverse(target, start)
        origin = self.forward(start)
        distance = math.dist(origin, target)
        if distance < 1e-9:
            return []
        count = math.ceil(distance / resolution)
        joints = start
        path = []
        for index in range(1, count + 1):
            fraction = index / count
            point = tuple(a + (b - a) * fraction for a, b in zip(origin, target))
            next_joints = self.inverse(point, joints)
            jump = max(abs(a - b) for a, b in zip(joints, next_joints))
            if jump > max_joint_jump:
                raise ValueError("Straight path crosses a wrist-position singularity or joint limit")
            path.append((next_joints, distance / count, jump))
            joints = next_joints
        return path
