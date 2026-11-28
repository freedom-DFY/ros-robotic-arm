"""Continuous, bounded gamepad servo references; independent of ROS and I/O."""

import numpy as np

from arm6_ros2.model import RADIANS_PER_TICK, rotation_vector, vector

GRIPPER_AXIS = 4
GRIPPER_CLOSE = (GRIPPER_AXIS, False)
WRIST_AXES = {3: 5, 5: 3, 6: 4}  # Wire axis -> joint index; retain the original J6 slot.
INTENT_SIZE = 7
GRIPPER_CONTACT_MESSAGE = '夹爪闭合受阻，已轻夹保持；向上推杆可松开'


def normalize_intent(intent):
    if len(intent) == 5:
        intent = np.r_[intent, 0., 0.]  # Original clients can still command J6 and the gripper.
    return vector(intent, INTENT_SIZE, 'gamepad intent')


def intent_key(intent):
    values = normalize_intent(intent)
    active = np.flatnonzero(np.abs(values) > 1e-6)
    if np.any(np.abs(values) > 1) or len(active) > 1:
        raise ValueError('Gamepad intent must select at most one normalized axis')
    return (int(active[0]), bool(values[active[0]] > 0)) if len(active) else None


class ContinuousJog:
    """Integrate velocity without waypoint arrivals or rest-to-rest segments.

    Intent axes: base X/Y/Z, J6, full gripper gap, J4, J5. Holding offsets
    are carried in encoder ticks; a stop never substitutes feedback for goals.
    """

    def __init__(self, model, settings, *, gripper_contact=False):
        self.model, self.p = model, settings
        self.reference = self.velocity = None
        self.offset = None
        self.key = self.blocked_key = None
        self.anchor = None
        self.gripper_progress = self.gripper_stall_time = 0.
        self.gripper_contact = gripper_contact
        self.gripper_closing = False
        self.message = '回中待命'

    def stop(self):
        self.key = None
        if self.velocity is not None:
            self.velocity[:] = 0

    def _anchor(self, actual, goals):
        self.reference = vector(actual, 7, 'servo feedback').copy()
        self.velocity = np.zeros(7)
        self.offset = np.asarray(goals, dtype=int)-self.model.encode(self.reference)
        if (np.any(np.abs(self.offset[:6])*RADIANS_PER_TICK > self.p['max_tracking_error_rad'])
                or abs(self.offset[6])*self.model.width/abs(self.model.opened-self.model.closed)
                > self.p['gripper_tracking_error_m']):
            raise ValueError('Existing hold error exceeds the tracking limit')
        self.anchor = self.model.forward(self.reference[:6])
        self.gripper_progress, self.gripper_stall_time = actual[6], 0.

    def _gripper_pace(self, actual_gap, goal_gap, direction, dt, lead_limit):
        lead = direction*(goal_gap-actual_gap)
        min_progress = 2*self.model.width/abs(self.model.opened-self.model.closed)
        threshold = min(lead_limit*0.6, min_progress) if goal_gap == 0 else lead_limit*0.6
        if lead < threshold or direction*(actual_gap-self.gripper_progress) >= min_progress:
            self.gripper_progress, self.gripper_stall_time = actual_gap, 0.
        else:
            self.gripper_stall_time += dt
        # A slow jaw can keep moving without tripping the hard tracking limit.
        # Reduce command speed smoothly as lead grows, with a separate stall check.
        return float(np.clip((lead_limit-lead)/(lead_limit*2/3), 0., 1.))

    def _relaxed_gripper_goal(self, actual, goals):
        measured = self.model.encode(actual)[6]
        direction = 1 if self.model.closed > self.model.opened else -1
        candidate = measured+direction*self.p['gripper_contact_preload_ticks']
        # Back off when the old target adds more preload. A contact event
        # must never send a target farther closed than the previous goal.
        target = candidate if direction*(candidate-goals[6]) < 0 else goals[6]
        return (*goals[:6], target) if target != goals[6] else None

    def _hold_gripper_contact(self, key, actual, goals):
        self.gripper_closing = False
        self.gripper_contact = True
        self._block(key, GRIPPER_CONTACT_MESSAGE)
        return self._relaxed_gripper_goal(actual, goals), 6, 'blocked'

    def _bounds(self):
        m = self.model
        ends = np.stack([(m.tick_min-self.offset[:6]-m.zero)*m.sign*RADIANS_PER_TICK,
                         (m.tick_max-self.offset[:6]-m.zero)*m.sign*RADIANS_PER_TICK])
        gap = (np.asarray(m.gripper_limits)-self.offset[6]-m.closed)*m.width/(m.opened-m.closed)
        return (np.r_[np.maximum(m.lower, ends.min(axis=0)), max(0., gap.min())],
                np.r_[np.minimum(m.upper, ends.max(axis=0)), min(m.width, gap.max())])

    def step(self, intent, actual, goals, dt):
        """Return (encoder goals or None, active motor or None, status)."""
        key = intent_key(intent)
        if key != GRIPPER_CLOSE and self.gripper_closing:
            self.gripper_closing = False
            relaxed = self._relaxed_gripper_goal(actual, goals)
            if relaxed is not None:
                self.stop()
                self.message = '夹爪已停止闭合，保留少量预紧'
                return relaxed, 6, 'holding'
        if key is None:
            self.stop()
            self.blocked_key = None
            self.message = GRIPPER_CONTACT_MESSAGE if self.gripper_contact else '摇杆回中，保持当前目标'
            return None, None, 'holding'
        if key == self.blocked_key:
            return None, None, 'blocked'
        if key == GRIPPER_CLOSE and self.gripper_contact:
            return self._block(key, GRIPPER_CONTACT_MESSAGE)
        if key != self.key:
            self.stop()
            self._anchor(actual, goals)
            if key[0] == GRIPPER_AXIS:
                # Preserve the last jaw target on reversal, but don't carry
                # jaw tracking lag as an offset into the new direction.
                self.reference[6] = self.model.decode(goals)[6]
                self.offset[6] = 0
            self.key = key
            self.blocked_key = None
        if not np.isfinite(dt) or not 0 < dt <= self.p['feedback_timeout_s']:
            raise ValueError('Servo period is stale or invalid')
        scale = self.p['gamepad_speed_scale']
        vmax = self.model.velocity*scale
        amax = self.model.acceleration*scale**2
        desired = np.zeros(7)
        axis, positive = key
        motor = None
        if axis < 3:
            pose = self.model.forward(self.reference[:6])
            drift = pose[:3, 3]-self.anchor[:3, 3]
            drift[axis] = 0
            if np.linalg.norm(drift) > 0.003:
                return self._block(key, '直线偏差达到限值，已保持；请改变方向')
            linear = -4.*drift
            linear[axis] = intent[axis]*self.p['gamepad_cartesian_speed_m_s']*scale
            full_jacobian = self.model.jacobian(self.reference[:6])
            if axis < 2:
                # XY has three position constraints and only the first three
                # motors available. Keep the height, leave orientation free.
                joints = slice(0, 3)
                jacobian = jac = full_jacobian[:3, :3]
                target = linear
            else:
                joints = slice(0, 6)
                angular_error = rotation_vector(self.anchor[:3, :3] @ pose[:3, :3].T)
                if np.linalg.norm(angular_error) > 0.03:
                    return self._block(key, '姿态偏差达到限值，已保持；请改变方向')
                weights = np.array([1., 1., 1., .2, .2, .2])
                jacobian = full_jacobian
                jac = jacobian*weights[:, None]
                target = np.r_[linear, 4.*angular_error]*weights
            desired[joints] = jac.T @ np.linalg.solve(jac @ jac.T+1e-6*np.eye(len(target)), target)
            progress = (jacobian @ desired[joints])[axis]*(1 if positive else -1)
            if progress < 1e-6:
                return self._block(key, '该方向接近奇异点，已保持；请改变方向')
        elif axis == GRIPPER_AXIS:
            motor = 6
            direction = 1 if positive else -1
            goal_gap = self.model.decode(goals)[6]
            lead_limit = self.p['gripper_tracking_error_m']*0.75
            if not positive:
                self.gripper_closing = True
                lead_limit = min(lead_limit, 0.0012)
            pace = self._gripper_pace(actual[6], goal_gap, direction, dt, lead_limit)
            if not positive and self.gripper_stall_time >= 0.25:
                return self._hold_gripper_contact(key, actual, goals)
            if self.gripper_stall_time >= 0.75:
                return self._block(key, '夹爪未跟随，已保持；请检查是否夹住物体或反向操作')
            desired[motor] = intent[axis]*vmax[motor]*pace
        else:
            motor = WRIST_AXES[axis]
            desired[motor] = intent[axis]*vmax[motor]
        # A common factor keeps the Cartesian velocity direction when one
        # joint reaches its speed or acceleration limit.
        desired *= min(1., float(np.min(vmax/np.maximum(np.abs(desired), 1e-12))))
        if axis < 3:
            achievable = abs((jacobian @ desired[joints])[axis])
            if achievable < max(1e-5, abs(linear[axis])*0.005):
                return self._block(key, '该方向接近工作空间边界/奇异点，已保持；请改变方向')
        change = desired-self.velocity
        blend = min(1., float(np.min(amax*dt/np.maximum(np.abs(change), 1e-12))))
        velocity = self.velocity+blend*change
        delta = velocity*dt
        if axis == GRIPPER_AXIS:
            # Enforce the soft tracking bound even while ramping down. The
            # encoder can round by half a tick, still below the hard limit.
            lead = direction*(self.reference[6]-actual[6])
            lead_room = max(0., lead_limit-lead)
            delta[6] = direction*min(abs(delta[6]), lead_room)
            velocity[6] = delta[6]/dt
        lower, upper = self._bounds()
        if np.any(self.reference < lower-1e-9) or np.any(self.reference > upper+1e-9):
            return self._block(key, '承重保持目标已接近关节限位，已保持；请调整姿态')
        fraction = 1.
        for q, step, lo, hi in zip(self.reference, delta, lower, upper):
            if step > 0:
                fraction = min(fraction, max(0., (hi-q)/step))
            elif step < 0:
                fraction = min(fraction, max(0., (lo-q)/step))
        self.reference += fraction*delta
        self.velocity = velocity
        result = tuple(int(t) for t in np.asarray(self.model.encode(self.reference))+self.offset)
        if (any(not lo <= t <= hi for t, lo, hi in
                zip(result[:6], self.model.tick_min, self.model.tick_max))
                or not self.model.gripper_limits[0] <= result[6] <= self.model.gripper_limits[1]):
            raise ValueError('Servo target exceeds the compensated encoder limits')
        if key == (GRIPPER_AXIS, True) and result[6] != goals[6]:
            self.gripper_contact = False
        if fraction < 1.-1e-9:
            # Closing at the calibrated endpoint still checks for contact
            # until feedback gets there or the jaw stalls against an object.
            if key == GRIPPER_CLOSE and actual[6] >= 2*self.model.width/abs(self.model.opened-self.model.closed):
                return result if result != tuple(goals) else None, motor, 'moving'
            self._block(key, '已到关节/夹爪限位，保持；反向或换轴可继续')
            return result if result != tuple(goals) else None, motor, 'blocked'
        self.message = '持续移动：'+('XYZ'[axis] if axis < 3 else '夹爪开合' if motor == 6 else f'J{motor+1}')
        return result if result != tuple(goals) else None, motor, 'moving'

    def _block(self, key, message):
        self.stop()
        self.blocked_key = key
        self.message = message
        return None, None, 'blocked'
