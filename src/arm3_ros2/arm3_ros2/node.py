"""ROS 2 joystick and position commands for the remaining three arm joints."""

import math
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import SetBool, Trigger

from arm3_ros2.kinematics import Calibration, RADIANS_PER_TICK, WristKinematics

JOINT_NAMES = ('joint_1', 'joint_2', 'joint_3')
DEFAULT_PARAMETERS = {
    'dry_run': True,
    'device': '/dev/ttyUSB0',
    'baud_rate': 1000000,
    'calibration_confirmed': False,
    'base_frame': 'base_link',
    'motor_ids': [1, 2, 3],
    'expected_model_number': 1120,
    'zero_ticks': [2048, 2048, 2048],
    'joint_directions': [1, 1, 1],
    'position_min_ticks': [0, 0, 0],
    'position_max_ticks': [4095, 4095, 4095],
    'mechanical_min_ticks': [0, 0, 0],
    'mechanical_max_ticks': [4095, 4095, 4095],
    'upper_arm_m': 0.24606,
    'forearm_m': 0.24225,
    'base_height_m': 0.0,
    'dh_offsets_rad': [math.pi / 2, -1.444, -1.444],
    'dry_start_ticks': [2048, 2160, 1910],
    'rate_hz': 30.0,
    'joint_speed_rad_s': 0.15,
    'cartesian_speed_m_s': 0.025,
    'path_resolution_m': 0.005,
    'max_path_joint_step_rad': 0.30,
    'max_tracking_error_rad': 0.12,
    'joy_joint_steps_ticks': [16, 8, 8],
    'joy_cartesian_step_m': 0.01,
    'profile_acceleration': 5,
    'profile_velocity': 12,
    'fixed_start_ticks': [2048, 2160, 1910],
    'fixed_line_delta_m': [0.0, -0.20, 0.0],
    'fixed_arrival_tolerance_ticks': 10,
    'fixed_settle_time_s': 0.5,
    'fixed_stage_timeout_s': 120.0,
}


class Controller(Node):
    def __init__(self, **node_options):
        super().__init__('arm3', **node_options)
        self.lock = threading.Lock()
        self.path = deque()
        self.bus = None
        self.closed = False
        self.fixed_cycle = None
        self.motion_state = 'idle'
        self.last_feedback_time = None
        self.motion_bounds_active = False
        try:
            self._initialize()
        except Exception:
            self.close()
            super().destroy_node()
            raise

    def _initialize(self):
        for name, default in DEFAULT_PARAMETERS.items():
            # Calibration and serial settings are applied on startup only.
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        param = lambda name: self.get_parameter(name).value
        self.dry_run = param('dry_run')
        self.confirmed = param('calibration_confirmed')
        self.frame = param('base_frame')
        self.ids = tuple(param('motor_ids'))
        self.expected_model = param('expected_model_number')
        if (len(self.ids) != 3 or len(set(self.ids)) != 3
                or any(type(i) is not int or not 0 <= i <= 252 for i in self.ids)):
            raise ValueError('motor_ids must contain three distinct DYNAMIXEL IDs')
        if not self.frame or not 0 < self.expected_model <= 65535 or param('baud_rate') <= 0:
            raise ValueError('Invalid base_frame, expected_model_number or baud_rate')
        self.calibration = Calibration(param('zero_ticks'), param('joint_directions'),
                                       param('position_min_ticks'), param('position_max_ticks'))
        self.mechanical = Calibration(param('zero_ticks'), param('joint_directions'),
                                      param('mechanical_min_ticks'), param('mechanical_max_ticks'))
        if any(not hard_lo <= lo < hi <= hard_hi
               for lo, hi, hard_lo, hard_hi in zip(
                   self.calibration.minimum, self.calibration.maximum,
                   self.mechanical.minimum, self.mechanical.maximum)):
            raise ValueError('Software motion bounds must lie inside the mechanical limits')
        self.kin = WristKinematics(param('upper_arm_m'), param('forearm_m'), param('base_height_m'),
                                   param('dh_offsets_rad'), self.calibration)
        self.hz = param('rate_hz')
        self.joint_speed = param('joint_speed_rad_s')
        self.cartesian_speed = param('cartesian_speed_m_s')
        self.resolution = param('path_resolution_m')
        self.max_jump = param('max_path_joint_step_rad')
        self.max_lag = param('max_tracking_error_rad')
        self.jog = tuple(param('joy_joint_steps_ticks'))
        self.cartesian_jog = param('joy_cartesian_step_m')
        self.profile_accel = param('profile_acceleration')
        self.profile_speed = param('profile_velocity')
        self.fixed_start = tuple(param('fixed_start_ticks'))
        self.fixed_delta = tuple(param('fixed_line_delta_m'))
        self.fixed_tolerance = param('fixed_arrival_tolerance_ticks')
        self.fixed_settle = param('fixed_settle_time_s')
        self.fixed_timeout = param('fixed_stage_timeout_s')
        self.validate_settings()

        self.enabled = self.dry_run
        self.measured = None
        self.last_written = None
        self.q_cmd = None
        self.cartesian_mode = False
        self.previous_joy = None
        if self.dry_run:
            self.measured = tuple(param('dry_start_ticks'))
            if (len(self.measured) != 3
                    or any(type(tick) is not int or not 0 <= tick <= 4095 for tick in self.measured)):
                raise ValueError('Invalid dry_start_ticks')
        else:
            # Dry-run needs no SDK import and never opens a serial port.
            from arm3_ros2.dynamixel_bus import DynamixelBus
            self.bus = DynamixelBus(param('device'), param('baud_rate'), self.ids)
            if any(model != self.expected_model for model in self.bus.models):
                raise ValueError(f'Expected DYNAMIXEL model {self.expected_model}, '
                                 f'got {self.bus.models}')
            self.get_logger().info(f'Found IDs {self.ids}, models {self.bus.models}; '
                                   'READ ONLY until enable service after calibration')

        self.joint_pub = self.create_publisher(JointState, '~/joint_states', 1)
        self.point_pub = self.create_publisher(PointStamped, '~/wrist_point', 1)
        self.ticks_pub = self.create_publisher(Int32MultiArray, '~/encoder_ticks', 1)
        self.status_pub = self.create_publisher(String, '~/motion_status', 1)
        self.create_subscription(JointState, '~/joint_target', self.joint_target, 1)
        self.create_subscription(PointStamped, '~/cartesian_target', self.cartesian_target, 1)
        self.create_subscription(Point, '~/auto_transform', self.legacy_target, 1)
        # Best effort also accepts the QoS used by ROS 2 joystick publishers.
        self.create_subscription(Joy, '/joy', self.on_joy, qos_profile_sensor_data)
        self.create_service(SetBool, '~/enable', self.enable)
        self.create_service(Trigger, '~/stop', self.stop)
        self.create_service(Trigger, '~/run_fixed_line', self.run_fixed_line)
        # Hardware interpolation follows wall time even if use_sim_time is set.
        self.control_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / self.hz, self.tick, clock=self.control_clock)
        if self.measured is not None:
            self.last_feedback_time = self._now()
        self.get_logger().info(f'arm3 ready; dry_run={self.dry_run}; '
                               'joystick starts in joint mode')

    def validate_settings(self):
        positive = (self.hz, self.joint_speed, self.cartesian_speed, self.resolution,
                    self.max_jump, self.max_lag, self.cartesian_jog)
        if not all(math.isfinite(value) and value > 0 for value in positive):
            raise ValueError('Motion settings must be positive and finite')
        if not 5 <= self.hz <= 100 or not 0.001 <= self.resolution <= 0.010:
            raise ValueError('Use 5–100 Hz and 1–10 mm Cartesian waypoints')
        if len(self.jog) != 3 or any(type(tick) is not int or tick <= 0 for tick in self.jog):
            raise ValueError('Expected three positive integer joystick tick increments')
        if any(type(value) is not int or not 0 < value <= 32767
               for value in (self.profile_accel, self.profile_speed)):
            raise ValueError('DYNAMIXEL profiles must be nonzero integers')
        if (len(self.fixed_start) != 3 or any(type(t) is not int or not 0 <= t <= 4095
                                             for t in self.fixed_start)):
            raise ValueError('fixed_start_ticks must contain three single-turn encoder integers')
        if (len(self.fixed_delta) != 3 or not all(math.isfinite(d) for d in self.fixed_delta)
                or not 0 < math.sqrt(sum(d*d for d in self.fixed_delta)) <= 0.2 + 1e-10):
            raise ValueError('fixed_line_delta_m must define a finite line of 0–200 mm')
        if (type(self.fixed_tolerance) is not int or not 1 <= self.fixed_tolerance <= 32
                or not all(math.isfinite(v) and v > 0 for v in (self.fixed_settle, self.fixed_timeout))
                or self.fixed_timeout <= self.fixed_settle):
            raise ValueError('Invalid fixed-cycle arrival tolerance, settle time or stage timeout')

    def _now(self):
        return time.monotonic()

    def _set_motion_state(self, state):
        if state != self.motion_state:
            self.motion_state = state
            self.get_logger().info(f'Motion state: {state}')

    def _cartesian_path(self, start, point):
        return deque((q, min(self.joint_speed, self.cartesian_speed * joint_delta / segment))
                     for q, segment, joint_delta in self.kin.line_path(
                         start, point, self.resolution, self.max_jump))

    def _require_motion_ready(self, *, check_current=True):
        """Apply calibration and software-range rules to motion, not initial holding."""
        if self.bus is not None:
            # Joint 1 may use its full single-turn range; joints 2/3 still
            # require calibrated ranges before real motion is accepted.
            if (not self.confirmed or any(
                    lo == 0 and hi == 4095
                    for lo, hi in zip(self.calibration.minimum[1:], self.calibration.maximum[1:]))):
                raise ValueError('Calibrate and confirm motion bounds before requesting movement')
            hardware_limits = self.bus.position_limits
            if hardware_limits is None or len(hardware_limits) != 3 or any(
                    not motor_lo <= lo < hi <= motor_hi
                    for lo, hi, (motor_lo, motor_hi) in zip(
                        self.calibration.minimum, self.calibration.maximum, hardware_limits)):
                raise ValueError('Software motion bounds must lie inside the servo position limits')
        if not self.mechanical.contains_ticks(self.measured):
            raise ValueError(f'Current position is outside the mechanical motion limits: {self.measured}; '
                             'holding remains enabled')
        if check_current and not self.calibration.contains_ticks(self.measured):
            raise ValueError('Current position is outside the motion bounds; holding remains enabled')

    def _accept_target(self, *, joints=None, point=None):
        if self.fixed_cycle is not None:
            raise ValueError('Fixed-start cycle active; use stop before sending another target')
        if self.closed or not self.enabled or self.measured is None:
            raise ValueError('Motion disabled or no fresh joint feedback')
        self._require_motion_ready()
        start = self.calibration.radians(self.measured)
        if joints is not None:
            self.calibration.ticks(joints)
            path = [(joints, self.joint_speed)]
        else:
            path = self._cartesian_path(start, point)
        self.path = deque(path)
        self.q_cmd = start
        self.motion_bounds_active = True
        self._set_motion_state('manual_motion')
        if not self.path:
            self._hold()

    def _command(self, **target):
        with self.lock:
            try:
                self._accept_target(**target)
            except ValueError as exc:
                self.get_logger().warning(f'Rejected command: {exc}')
            except Exception as exc:
                self._fault(exc)

    def joint_target(self, msg):
        if msg.name:
            if len(msg.name) != 3 or set(msg.name) != set(JOINT_NAMES) or len(msg.position) != 3:
                self.get_logger().warning('Joint target must name joint_1, joint_2 '
                                          'and joint_3 exactly once')
                return
            ordered = dict(zip(msg.name, msg.position))
            self._command(joints=tuple(ordered[name] for name in JOINT_NAMES))
        else:
            self._command(joints=tuple(msg.position))

    def cartesian_target(self, msg):
        if msg.header.frame_id != self.frame:
            self.get_logger().warning(f'Cartesian target frame must be {self.frame}')
            return
        self._command(point=(msg.point.x, msg.point.y, msg.point.z))

    def legacy_target(self, msg):
        self._command(point=(msg.x / 1000, msg.y / 1000, msg.z / 1000))

    def on_joy(self, msg):
        if len(msg.buttons) < 9 or len(msg.axes) < 6:
            self.get_logger().warning('Joystick expects the original Logitech DirectInput layout',
                                      throttle_duration_sec=5.0)
            return
        current = (bool(msg.buttons[8]), bool(msg.buttons[6]), bool(msg.buttons[7]),
                   bool(msg.buttons[4]), bool(msg.buttons[5]),
                   msg.axes[5] > 0.5, msg.axes[5] < -0.5,
                   msg.axes[4] > 0.5, msg.axes[4] < -0.5,
                   bool(msg.buttons[3]), bool(msg.buttons[1]))
        with self.lock:
            previous, self.previous_joy = self.previous_joy, current
            if previous is None:
                return
            back, lt, rt, lb, rb, up, down, left, right, y, a = (
                pressed and not was_pressed for pressed, was_pressed in zip(current, previous)
            )
            if back:
                self.cartesian_mode = not self.cartesian_mode
                self.get_logger().info('Joystick mode: ' +
                                       ('Cartesian' if self.cartesian_mode else 'joint'))
                return
            if self.closed or not self.enabled or self.measured is None:
                return
            try:
                start = self.calibration.radians(self.measured)
                if self.cartesian_mode:
                    delta = (int(up) - int(down), int(left) - int(right), int(y) - int(a))
                    if any(delta):
                        self._accept_target(point=tuple(
                            value + sign * self.cartesian_jog
                            for value, sign in zip(self.kin.forward(start), delta)
                        ))
                else:
                    delta = (int(rt) - int(lt), int(rb) - int(lb), int(up) - int(down))
                    if any(delta):
                        target_ticks = tuple(tick + sign * increment
                                             for tick, sign, increment in zip(self.measured, delta, self.jog))
                        self._accept_target(joints=self.calibration.radians(target_ticks))
            except ValueError as exc:
                self.get_logger().warning(f'Rejected joystick command: {exc}')
            except Exception as exc:
                self._fault(exc)

    def enable(self, request, response):
        with self.lock:
            if self.closed:
                return SetBool.Response(success=False, message='Controller is shutting down')
            if not request.data:
                self.path.clear()
                self.fixed_cycle = None
                self.motion_bounds_active = False
                self._set_motion_state('disabled')
                self.enabled = False
                failures = self.bus.disarm() if self.bus is not None else []
                return SetBool.Response(success=not failures,
                                        message='; '.join(failures) or 'Motion disabled')
            if self.enabled:
                return SetBool.Response(success=True, message='Already enabled')
            if self.bus is not None:
                try:
                    self.measured = self.bus.arm(
                        self.profile_accel, self.profile_speed, command_rate_hz=self.hz
                    )
                except Exception as exc:
                    return SetBool.Response(success=False, message=str(exc))
            if self.measured is None:
                return SetBool.Response(success=False, message='No joint feedback; restart the controller')
            self.last_written = self.measured
            self.q_cmd = self.calibration.radians(self.measured)
            self.enabled = True
            self.motion_bounds_active = False
            self.last_feedback_time = self._now()
            self._set_motion_state('idle')
            return SetBool.Response(success=True, message='Enabled; holding the current position')

    def run_fixed_line(self, _request, response):
        """Return in joint space, then run the preflighted software-bounded line."""
        with self.lock:
            try:
                if self.closed or not self.enabled or self.measured is None:
                    raise ValueError('Enable motion and obtain fresh feedback first')
                if self.fixed_cycle is not None or self.path:
                    raise ValueError('Another motion is active; stop it before starting a fixed cycle')
                if (self.last_feedback_time is None
                        or self._now() - self.last_feedback_time > max(0.25, 3 / self.hz)):
                    raise ValueError('Joint feedback is stale')
                # The current pose may be outside the application's test window.
                # Only the subsequent Cartesian line uses that window.
                self._require_motion_ready(check_current=False)
                if not self.calibration.contains_ticks(self.fixed_start):
                    raise ValueError('Fixed start is outside the loaded trial bounds')
                if self.fixed_tolerance * RADIANS_PER_TICK > self.max_lag:
                    raise ValueError('Fixed arrival tolerance exceeds the tracking-error threshold')
                fixed_q = self.calibration.radians(self.fixed_start)
                fixed_point = self.kin.forward(fixed_q)
                end_point = tuple(a + d for a, d in zip(fixed_point, self.fixed_delta))
                start_q = self.calibration.radians(self.measured)
                hardware_limits = (self.bus.position_limits if self.bus is not None
                                   else ((0, 4095),) * 3)
                # Returning bypasses the trial window, but never the separate
                # documented mechanical limits or the servo's register limits.
                return_calibration = Calibration(
                    self.calibration.zero, self.calibration.direction,
                    [max(lo, hard_lo) for (lo, _), hard_lo in zip(
                        hardware_limits, self.mechanical.minimum)],
                    [min(hi, hard_hi) for (_, hi), hard_hi in zip(
                        hardware_limits, self.mechanical.maximum)])
                if (not return_calibration.contains_ticks(self.measured)
                        or not return_calibration.contains_ticks(self.fixed_start)):
                    raise ValueError('Return start or target is outside the mechanical/servo position limits')
                # Each streamed encoder value stays between current and fixed
                # ticks. Cartesian IK and the trial window do not govern this leg.
                return_path = (deque([(fixed_q, self.joint_speed)])
                               if tuple(self.measured) != self.fixed_start else deque())
                line_path = self._cartesian_path(fixed_q, end_point)
                end_ticks = self.calibration.ticks(line_path[-1][0])
                # Do not start unless the encoder return endpoints are valid
                # and the complete subsequent Cartesian line passes preflight.
                self.fixed_cycle = {
                    'fixed_q': fixed_q, 'line_path': line_path, 'end_ticks': end_ticks,
                    'return_calibration': return_calibration,
                    'stage_started': self._now(), 'stable_since': None, 'previous_ticks': None,
                    'settle_anchor': None,
                }
                self.path = return_path
                self.q_cmd = start_q
                self.motion_bounds_active = False
                self._set_motion_state('moving_to_start')
                return Trigger.Response(success=True, message=
                                        f'Accepted: return to {self.fixed_start}, settle, then '
                                        f'line {self.fixed_delta} m; end ticks {end_ticks}')
            except ValueError as exc:
                return Trigger.Response(success=False, message=str(exc))
            except Exception as exc:
                self._fault(exc)
                return Trigger.Response(success=False, message=str(exc))

    def _advance_fixed_cycle(self):
        cycle = self.fixed_cycle
        if cycle is None:
            return
        now = self._now()
        if now - cycle['stage_started'] > self.fixed_timeout:
            raise RuntimeError(f'Fixed cycle timed out during {self.motion_state}')
        if self.motion_state in ('moving_to_start', 'line_motion'):
            if self.path:
                return
            self._set_motion_state('settling_start' if self.motion_state == 'moving_to_start'
                                   else 'settling_end')
            cycle['stable_since'] = None
            cycle['previous_ticks'] = None
            cycle['settle_anchor'] = None
        target = self.fixed_start if self.motion_state == 'settling_start' else cycle['end_ticks']
        previous = cycle['previous_ticks']
        cycle['previous_ticks'] = self.measured
        within = max(abs(a-b) for a,b in zip(self.measured, target)) <= self.fixed_tolerance
        quiet = previous is not None and max(abs(a-b) for a,b in zip(self.measured, previous)) <= 3
        if not within or not quiet:
            cycle['stable_since'] = None
            cycle['settle_anchor'] = None
            return
        if cycle['stable_since'] is None:
            cycle['stable_since'] = now
            cycle['settle_anchor'] = self.measured
        elif max(abs(a-b) for a,b in zip(self.measured, cycle['settle_anchor'])) > 3:
            cycle['stable_since'] = now
            cycle['settle_anchor'] = self.measured
        if now - cycle['stable_since'] < self.fixed_settle:
            return
        if self.motion_state == 'settling_start':
            # Application bounds and feedback checks resume for the 20 cm leg.
            self._require_motion_ready()
            self.path = cycle['line_path']
            # Keep the commanded start and endpoint fixed across repeated cycles.
            self.q_cmd = cycle['fixed_q']
            self.motion_bounds_active = True
            cycle['stage_started'] = now
            cycle['stable_since'] = None
            cycle['previous_ticks'] = None
            cycle['settle_anchor'] = None
            self._set_motion_state('line_motion')
        else:
            self.fixed_cycle = None
            self._set_motion_state('completed')

    def stop(self, _request, response):
        with self.lock:
            if self.closed:
                return Trigger.Response(success=False, message='Controller is shutting down')
            try:
                self._hold()
            except Exception as exc:
                self._fault(exc)
                return Trigger.Response(success=False, message=str(exc))
            return Trigger.Response(success=True, message='Queue cleared; holding position if enabled')

    def _hold(self):
        self.fixed_cycle = None
        self._set_motion_state('stopped')
        self.path.clear()
        if not self.enabled:
            return
        if self.bus is not None:
            present = self.bus.read_positions()
            if self.motion_bounds_active and not self.calibration.contains_ticks(present):
                raise RuntimeError('Cannot hold: current ticks are outside calibrated limits')
            if len(present) != 3 or any(type(tick) is not int or not 0 <= tick <= 4095
                                        for tick in present):
                raise RuntimeError('Cannot hold: invalid single-turn encoder feedback')
            self.bus.write_positions(present)
            self.measured = present
        self.q_cmd = self.calibration.radians(self.measured)
        self.last_written = self.measured

    def _fault(self, exc):
        self.path.clear()
        self.fixed_cycle = None
        self._set_motion_state('fault')
        self.enabled = False
        self.motion_bounds_active = False
        self.measured = None
        self.get_logger().error(f'Motion inhibited: {exc}. '
                                'Clear the fault and use enable false/true.',
                                throttle_duration_sec=2.0)

    def tick(self):
        with self.lock:
            if self.closed:
                return
            try:
                if self.bus is not None:
                    raw = self.bus.read_positions()
                    if (self.enabled and (len(raw) != 3 or any(
                            type(tick) is not int or not 0 <= tick <= 4095 for tick in raw))):
                        raise RuntimeError('Invalid single-turn encoder feedback')
                    if (self.enabled and self.motion_bounds_active
                            and not self.calibration.contains_ticks(raw)):
                        raise RuntimeError(f'Joint feedback outside calibrated limits: {raw}')
                    self.measured = raw if self.enabled else tuple(tick % 4096 for tick in raw)
                if (self.enabled and (self.motion_bounds_active or self.fixed_cycle is not None)
                        and not self.mechanical.contains_ticks(self.measured)):
                    raise RuntimeError(f'Joint feedback outside mechanical limits: {self.measured}')
                if (self.fixed_cycle is not None
                        and self._now() - self.fixed_cycle['stage_started'] > self.fixed_timeout):
                    raise RuntimeError(f'Fixed cycle timed out during {self.motion_state}')
                if self.enabled and self.path:
                    self._step()
                if self.measured is not None:
                    self.last_feedback_time = self._now()
                    if self.enabled:
                        self._advance_fixed_cycle()
                    if self.fixed_cycle is None and not self.path and self.motion_state == 'manual_motion':
                        self._set_motion_state('idle')
                    self._publish()
            except Exception as exc:
                self._fault(exc)
            finally:
                if self.context.ok():
                    self.status_pub.publish(String(data=self.motion_state))

    def _step(self):
        measured = self.calibration.radians(self.measured)
        if max(abs(a - b) for a, b in zip(measured, self.q_cmd)) > self.max_lag:
            raise RuntimeError('Actuator is not following the commanded position')
        target, speed = self.path[0]
        error = max(abs(a - b) for a, b in zip(self.q_cmd, target))
        allowed = speed / self.hz
        if error <= max(allowed, 1e-12):
            self.q_cmd = target
            self.path.popleft()
        else:
            self.q_cmd = tuple(a + (b - a) * (allowed / error) for a, b in zip(self.q_cmd, target))
        conversion = (self.fixed_cycle['return_calibration']
                      if self.fixed_cycle is not None
                      and self.motion_state in ('moving_to_start', 'settling_start')
                      else self.calibration)
        ticks = conversion.ticks(self.q_cmd)
        if not self.mechanical.contains_ticks(ticks):
            raise RuntimeError(f'Command outside mechanical limits: {ticks}')
        if ticks != self.last_written:
            if self.bus is not None:
                self.bus.write_positions(ticks)
            else:
                self.measured = ticks
            self.last_written = ticks

    def _publish(self):
        q = self.calibration.radians(self.measured)
        joints = JointState()
        joints.header.stamp = self.get_clock().now().to_msg()
        joints.name = list(JOINT_NAMES)
        joints.position = list(q)
        point = PointStamped()
        point.header.stamp = joints.header.stamp
        point.header.frame_id = self.frame
        point.point.x, point.point.y, point.point.z = self.kin.forward(q)
        self.joint_pub.publish(joints)
        self.point_pub.publish(point)
        # Single-turn encoder ticks in motor_ids order, including read-only mode.
        self.ticks_pub.publish(Int32MultiArray(data=list(self.measured)))

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.fixed_cycle = None
            self.path.clear()
            if hasattr(self, 'timer'):
                self.timer.cancel()
            if self.bus is not None:
                try:
                    failures = self.bus.disarm()
                    if failures:
                        self.get_logger().error('Torque-off could not be confirmed: ' + '; '.join(failures))
                finally:
                    self.bus.close()

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = None
    try:
        controller = Controller()
        rclpy.spin(controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if controller is not None:
            controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
