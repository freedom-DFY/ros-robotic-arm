"""ROS 2 six-axis controller with an independent seventh gripper motor."""

import math
import threading
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int32MultiArray, String
from std_srvs.srv import SetBool, Trigger

from arm6_ros2.config import DEFAULTS
from arm6_ros2.model import ArmModel, JOINT_NAMES, RADIANS_PER_TICK, pose_matrix, quaternion_from_matrix, vector
from arm6_ros2.trajectory import Plan, trajectory_request


class Controller(Node):
    def __init__(self, **options):
        super().__init__('arm6', **options)
        self.lock = threading.RLock()
        self.bus = None
        self.closed = False
        self.active = None
        self.calibration_jog = None
        self.reservation = None
        self.enabled = False
        self.state = 'read_only'
        self.measured = None
        self.last_feedback = None
        self.last_tick = None
        self.last_written = None
        self.last_health_check = 0.0
        self.action_server = None
        try:
            self._initialize()
        except Exception:
            self.close()
            super().destroy_node()
            raise

    def _initialize(self):
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.user_origin_ticks = tuple(self.p['home_ticks'])
        self.model = ArmModel(self.p)
        self._validate_settings()
        self.frame = self.p['base_frame']
        from arm6_ros2.dynamixel_bus import DynamixelBus
        self.bus = DynamixelBus(
            self.p['device'], self.p['baud_rate'], self.model.ids,
            gripper_wrap_limits=self.model.gripper_limits if self.model.gripper_wraps_zero else None)
        if tuple(self.bus.models) != self.model.models:
            raise ValueError(f'Model mismatch in motor_ids order: expected {self.model.models}, '
                             f'found {self.bus.models}')
        self.joint_pub = self.create_publisher(JointState, '~/joint_states', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '~/tcp_pose', 10)
        self.tick_pub = self.create_publisher(Int32MultiArray, '~/encoder_ticks', 10)
        self.origin_pub = self.create_publisher(Int32MultiArray, '~/user_origin_ticks', 10)
        self.width_pub = self.create_publisher(Float64, '~/gripper_width', 10)
        self.status_pub = self.create_publisher(String, '~/motion_status', 10)
        self.create_subscription(JointState, '~/joint_target', self.joint_target, 1)
        self.create_subscription(PoseStamped, '~/pose_target', self.pose_target, 1)
        self.create_subscription(Float64, '~/gripper_target', self.gripper_target, 1)
        self.create_service(SetBool, '~/enable', self.enable)
        for index in range(6):
            self.create_service(SetBool, f'~/calibration/joint_{index+1}/jog',
                                lambda request, response, index=index:
                                self.calibration_jog_request(request, response, index))
            self.create_service(SetBool, f'~/manual/joint_{index+1}/jog',
                                lambda request, response, index=index:
                                self.calibration_jog_request(request, response, index, require_unconfirmed=False))
        self.create_service(Trigger, '~/set_user_origin', self.set_user_origin)
        self.create_service(Trigger, '~/home', self.return_home)
        self.create_service(Trigger, '~/manual/gripper/open',
                            lambda request, response: self.manual_gripper_request(request, response, True))
        self.create_service(Trigger, '~/manual/gripper/close',
                            lambda request, response: self.manual_gripper_request(request, response, False))
        self.create_service(SetBool, '~/manual/gripper/jog', self.manual_gripper_jog)
        self.create_service(Trigger, '~/stop', self.stop)
        self.create_service(Trigger, '~/gripper/open', self.open_gripper)
        self.create_service(Trigger, '~/gripper/close', self.close_gripper)
        self.action_server = ActionServer(
            self, FollowJointTrajectory, '~/follow_joint_trajectory', self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup())
        self.timer = self.create_timer(1/self.p['rate_hz'], self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Six-axis arm + ID{self.model.ids[-1]} gripper ready; manual enable required. '
                               f'device={self.p["device"]}, state={self.state}')
        if self.p['home_on_enable']:
            self.get_logger().info(f'Enable will return to taught home ticks={self.p["home_ticks"]}; '
                                   'gripper opening will be held')

    def _validate_settings(self):
        positive = ('rate_hz', 'feedback_timeout_s', 'max_tracking_error_rad',
                    'gripper_tracking_error_m', 'arrival_tolerance_rad',
                    'gripper_arrival_tolerance_m', 'settle_time_s', 'arrival_timeout_s',
                    'cartesian_resolution_m', 'cartesian_resolution_rad',
                    'max_ik_joint_step_rad', 'max_trajectory_duration_s', 'home_arrival_tolerance_rad')
        if any(not math.isfinite(self.p[k]) or self.p[k] <= 0 for k in positive):
            raise ValueError('Timing, tolerance and path settings must be positive and finite')
        if (not 5 <= self.p['rate_hz'] <= 100
                or self.p['feedback_timeout_s'] < 3/self.p['rate_hz']
                or not self.p['base_frame'] or self.p['baud_rate'] <= 0
                or type(self.p['max_waypoints']) is not int or not 1 <= self.p['max_waypoints'] <= 10000
                or self.p['arrival_timeout_s'] <= self.p['settle_time_s']
                or self.p['arrival_tolerance_rad'] > self.p['max_tracking_error_rad']
                or self.p['gripper_arrival_tolerance_m'] > self.p['gripper_tracking_error_m']):
            raise ValueError('Invalid frame, rate, serial, waypoint or settling settings')
        if any(type(self.p[k]) is not int or not 1 <= self.p[k] <= 32767
               for k in ('profile_velocity', 'profile_acceleration')):
            raise ValueError('Register profile values must be positive integers')
        home = vector(self.p['home_ticks'], 6, 'home_ticks')
        if (type(self.p['calibration_jog_step_ticks']) is not int
                or not 1 <= self.p['calibration_jog_step_ticks'] <= 20):
            raise ValueError('calibration_jog_step_ticks must be an integer from 1 to 20')
        if (np.any(home != np.round(home)) or np.any(home < self.model.tick_min)
                or np.any(home > self.model.tick_max)):
            raise ValueError('home_ticks must be six integer positions inside joint motion limits')

    def _valid_raw(self, raw):
        gripper_max = 8191 if self.model.gripper_wraps_zero else 4095
        if (len(raw) != 7 or any(type(t) is not int or not 0 <= t <= 4095 for t in raw[:6])
                or type(raw[6]) is not int or not 0 <= raw[6] <= gripper_max):
            raise ValueError('Expected six single-turn joint integers and a valid gripper encoder')
        return tuple(raw)

    def _unarmed_positions(self, raw):
        return tuple(t % 4096 for t in raw[:6]) + (
            raw[6] if self.model.gripper_wraps_zero else raw[6] % 4096,)

    def _feedback(self):
        if self.closed or self.measured is None or self.last_feedback is None:
            raise ValueError('No joint feedback')
        if time.monotonic()-self.last_feedback > self.p['feedback_timeout_s']:
            raise ValueError('Joint feedback is stale')
        return self.model.decode(self.measured)

    def _ready(self, *, cartesian=False):
        if not self.enabled or self.state == 'fault':
            raise ValueError('Enable motion first; faults require disable then enable')
        if self.active is not None or self.calibration_jog is not None or self.reservation is not None:
            raise ValueError('Another motion is active; stop or cancel before a new command')
        current = self._feedback()
        if not self.p['calibration_confirmed']:
            raise ValueError('Motor calibrations await manual confirmation (calibration_confirmed=false): '
                             'verify encoder zeros, directions and limits; independent of home tolerance')
        if cartesian and not self.p['geometry_confirmed']:
            raise ValueError('DH model and reference TCP await manual confirmation '
                             '(geometry_confirmed=false); independent of home tolerance')
        self._check_motion_bounds(current)
        return current

    def _check_motion_bounds(self, current):
        if self.bus.position_limits is None:
            raise ValueError('Servo limits have not been read by enable')
        minima = list(self.model.tick_min)+[min(self.model.closed, self.model.opened)]
        maxima = list(self.model.tick_max)+[max(self.model.closed, self.model.opened)]
        if any(not lo <= a < b <= hi for a, b, (lo, hi) in
               zip(minima, maxima, self.bus.position_limits)):
            raise ValueError('Configured motion limits must lie inside servo register limits')
        self.model.encode(current)  # Motion checks apply after enable-at-current-position.

    def _home_plan(self, raw):
        if not self.p['home_confirmed']:
            raise ValueError('Confirm the taught home pose before automatic homing')
        current = self.model.decode(raw)
        # Preserve the gripper's measured opening, including its reversed direction.
        target = self.model.decode(self.user_origin_ticks + (raw[6],))
        return Plan(self.model, current, [target])

    def set_user_origin(self, _request, _response):
        with self.lock:
            try:
                if (not self.enabled or self.state not in ('holding', 'stopped')
                        or self.active is not None or self.calibration_jog is not None
                        or self.reservation is not None):
                    raise ValueError('Set user origin only while enabled and stationary')
                self._check_motion_bounds(self._feedback())
                self.user_origin_ticks = tuple(self.measured[:6])
                self.origin_pub.publish(Int32MultiArray(data=list(self.user_origin_ticks)))
                return Trigger.Response(success=True, message=str(list(self.user_origin_ticks)))
            except ValueError as exc:
                return Trigger.Response(success=False, message=str(exc))

    def return_home(self, _request, _response):
        with self.lock:
            try:
                if (not self.enabled or self.state not in ('holding', 'stopped')
                        or self.active is not None or self.calibration_jog is not None
                        or self.reservation is not None):
                    raise ValueError('Return home requires an enabled stationary arm')
                current = self._feedback()
                self._check_motion_bounds(current)
                plan = self._home_plan(self.measured)
                tolerance = self.p['home_arrival_tolerance_rad']
                if np.all(np.abs(current[:6]-plan.points[-1][:6]) <= tolerance):
                    self.state = 'holding'
                    return Trigger.Response(success=True, message='Already within user-origin tolerance')
                self._start(plan, tolerances={'goal': np.full(6, tolerance)})
                return Trigger.Response(success=True, message='Returning to user origin; gripper opening held')
            except ValueError as exc:
                return Trigger.Response(success=False, message=str(exc))
            except Exception as exc:
                self._fault(exc)
                return Trigger.Response(success=False, message=str(exc))

    def calibration_jog_request(self, request, _response, index, require_unconfirmed=True):
        with self.lock:
            proposed = None
            try:
                if require_unconfirmed and self.p['calibration_confirmed']:
                    raise ValueError('Calibration jog is only available while encoder calibration is unconfirmed')
                if (not self.enabled or self.state not in ('holding', 'stopped')
                        or self.active is not None or self.calibration_jog is not None
                        or self.reservation is not None):
                    raise ValueError('Jog requires an enabled, stationary arm without another motion')
                self._check_motion_bounds(self._feedback())
                proposed = {'axis': index}
                self.calibration_jog = proposed
                # Increment the existing hold goal, so repeated small jogs do not
                # lose every step to an unmodified P-only controller's load error.
                initial = self.bus.read_goal_position(index)
                step = self.p['calibration_jog_step_ticks'] * (1 if request.data else -1)
                target = initial+step
                if not (self.model.tick_min[index] <= initial <= self.model.tick_max[index]
                        and self.model.tick_min[index] <= target <= self.model.tick_max[index]):
                    raise ValueError('Selected jog would cross the measured motor limits')
                velocity = min(0.02, self.model.velocity[index])
                acceleration = min(0.04, self.model.acceleration[index])
                self._start_manual_axis(index, initial, target, velocity, acceleration)
                return SetBool.Response(success=True, message=(
                    f'ID{self.model.ids[index]} goal {initial}->{target}; other hold goals unchanged'))
            except ValueError as exc:
                if proposed is not None and self.calibration_jog is proposed:
                    self.calibration_jog = None
                return SetBool.Response(success=False, message=str(exc))
            except Exception as exc:
                self._fault(exc)
                return SetBool.Response(success=False, message=str(exc))

    def _start_manual_axis(self, index, initial, target, velocity, acceleration):
        distance = abs(target-initial)*RADIANS_PER_TICK
        duration = max(1.875*distance/velocity,
                       math.sqrt((10/math.sqrt(3))*distance/acceleration), 0.1)*1.01
        self.calibration_jog = dict(axis=index, initial=initial, target=target, duration=duration,
                                    started=time.monotonic(), last=initial)
        self.last_tick = time.monotonic()
        self.state = 'calibrating'

    def manual_gripper_jog(self, request, _response):
        result = self.manual_gripper_request(None, None, request.data, step_only=True)
        return SetBool.Response(success=result.success, message=result.message)

    def manual_gripper_request(self, _request, _response, opened, step_only=False):
        with self.lock:
            proposed = None
            try:
                if (not self.enabled or self.state not in ('holding', 'stopped')
                        or self.active is not None or self.calibration_jog is not None
                        or self.reservation is not None):
                    raise ValueError('Gripper control requires an enabled stationary arm')
                self._check_motion_bounds(self._feedback())
                proposed = {'axis': 6}
                self.calibration_jog = proposed
                initial = self.bus.read_goal_position(6)
                target = self.model.opened if opened else self.model.closed
                lo, hi = self.model.gripper_limits
                if not lo <= initial <= hi:
                    raise ValueError('Gripper hold goal is outside the measured gripper range')
                if step_only:
                    step = self.p['calibration_jog_step_ticks']
                    target = initial+max(-step, min(step, target-initial))
                    if target == initial:
                        raise ValueError('Gripper already at the measured endpoint')
                self._start_manual_axis(6, initial, target, 0.05, 0.05)
                return Trigger.Response(success=True, message=(
                    f'Gripper goal {initial}->{target}; six arm hold goals unchanged'))
            except ValueError as exc:
                if proposed is not None and self.calibration_jog is proposed:
                    self.calibration_jog = None
                return Trigger.Response(success=False, message=str(exc))
            except Exception as exc:
                self._fault(exc)
                return Trigger.Response(success=False, message=str(exc))

    def _step_calibration_jog(self, now):
        jog = self.calibration_jog
        index = jog['axis']
        measured = self.measured[index]
        lo, hi = (self.model.gripper_limits if index == 6 else
                  (self.model.tick_min[index], self.model.tick_max[index]))
        if not lo <= measured <= hi:
            raise RuntimeError('Calibration feedback crossed the measured motor limit')
        if abs(measured-jog['last'])*RADIANS_PER_TICK > self.p['max_tracking_error_rad']:
            raise RuntimeError('Calibration tracking error exceeded the configured limit')
        elapsed = now-jog['started']
        fraction = min(1.0, max(0.0, elapsed/jog['duration']))
        smooth = fraction**3*(10+fraction*(-15+fraction*6))
        tick = round(jog['initial']+smooth*(jog['target']-jog['initial']))
        if tick != jog['last']:
            self.bus.write_single_position(index, tick)
            jog['last'] = tick
        if elapsed >= jog['duration']+0.3:
            self.bus.write_single_position(index, jog['target'])
            self.get_logger().info(f'Calibration jog ID{self.model.ids[index]} finished: '
                                   f'goal={jog["target"]}, feedback={measured}; record actual feedback')
            self.calibration_jog = None
            self.state = 'holding'

    def _start(self, plan, *, handle=None, future=None, tolerances=None):
        # Planning can take longer than one feedback period. Never execute a
        # preflighted path from stale feedback or from a changed starting pose.
        current = self._feedback()
        if np.max(np.abs(current-plan.start)) > 0.002:
            raise ValueError('Measured start changed while planning')
        self.active = {
            'plan': plan, 'started': time.monotonic(), 'stable': None,
            'handle': handle, 'future': future,
            'path_tolerance': (tolerances or {}).get('path', np.full(6, self.p['max_tracking_error_rad'])),
            'goal_tolerance': (tolerances or {}).get('goal', np.full(6, self.p['arrival_tolerance_rad'])),
            'timeout': (tolerances or {}).get('timeout', self.p['arrival_timeout_s']),
        }
        self.last_written = tuple(self.measured)
        self.last_tick = time.monotonic()
        self.state = 'moving'

    def _reject_or_fault(self, operation):
        with self.lock:
            try:
                operation()
            except ValueError as exc:
                self.get_logger().warning(f'Rejected command: {exc}')
            except Exception as exc:
                self._fault(exc)

    def joint_target(self, msg):
        def command():
            current = self._ready()
            q = vector(msg.position, 6, 'joint target')
            if msg.name:
                if len(msg.name) != 6 or set(msg.name) != set(JOINT_NAMES):
                    raise ValueError('Joint target must name all six arm joints exactly once')
                q = q[[msg.name.index(name) for name in JOINT_NAMES]]
            self._start(Plan(self.model, current, [np.r_[q, current[6]]]))
        self._reject_or_fault(command)

    def pose_target(self, msg):
        def command():
            current = self._ready(cartesian=True)
            if msg.header.frame_id != self.frame:
                raise ValueError(f'Pose target frame must be {self.frame}')
            pos, quat = msg.pose.position, msg.pose.orientation
            target = pose_matrix([pos.x, pos.y, pos.z], [quat.x, quat.y, quat.z, quat.w])
            points = self.model.cartesian_path(current[:6], target)
            # Preflight all IK and time bounds before accepting any movement.
            self._start(Plan(self.model, current, [np.r_[q, current[6]] for q in points]))
        self._reject_or_fault(command)

    def _gripper(self, width):
        current = self._ready()
        target = current.copy()
        target[6] = width
        self._start(Plan(self.model, current, [target]))

    def gripper_target(self, msg):
        self._reject_or_fault(lambda: self._gripper(msg.data))

    def _gripper_service(self, width):
        with self.lock:
            try:
                self._gripper(width)
                return Trigger.Response(success=True, message=f'Accepted gripper width {width} m')
            except ValueError as exc:
                return Trigger.Response(success=False, message=str(exc))
            except Exception as exc:
                self._fault(exc)
                return Trigger.Response(success=False, message=str(exc))

    def open_gripper(self, _request, _response):
        return self._gripper_service(self.model.width)

    def close_gripper(self, _request, _response):
        return self._gripper_service(0.0)

    def _action_tolerances(self, request):
        result = {}
        for key, entries, default in (
            ('path', request.path_tolerance, self.p['max_tracking_error_rad']),
            ('goal', request.goal_tolerance, self.p['arrival_tolerance_rad']),
        ):
            values = np.full(6, default)
            seen = set()
            for entry in entries:
                if (entry.name not in JOINT_NAMES or entry.name in seen
                        or not math.isfinite(entry.position) or entry.position < 0
                        or entry.velocity != 0 or entry.acceleration != 0):
                    raise ValueError('Tolerances must name unique arm joints with nonnegative position only')
                seen.add(entry.name)
                if entry.position > 0:
                    values[JOINT_NAMES.index(entry.name)] = min(default, entry.position)
            result[key] = values
        duration = request.goal_time_tolerance
        if duration.sec < 0 or not 0 <= duration.nanosec < 10**9:
            raise ValueError('Invalid goal time tolerance')
        timeout = duration.sec+duration.nanosec*1e-9
        if timeout and timeout <= self.p['settle_time_s']:
            raise ValueError('Goal time tolerance must exceed settle_time_s')
        result['timeout'] = min(timeout or self.p['arrival_timeout_s'], self.p['max_trajectory_duration_s'])
        # This controller supports single-DOF position trajectories only.
        if (request.multi_dof_trajectory.joint_names or request.multi_dof_trajectory.points
                or request.component_path_tolerance or request.component_goal_tolerance):
            raise ValueError('Multi-DOF trajectories and component tolerances are unsupported')
        return result

    def goal(self, request):
        with self.lock:
            try:
                current = self._ready()
                plan = trajectory_request(self.model, current, request.trajectory)
                tolerance = self._action_tolerances(request)
                self.reservation = (request, plan, tolerance)
                return GoalResponse.ACCEPT
            except ValueError as exc:
                self.get_logger().warning(f'Rejected trajectory: {exc}')
                return GoalResponse.REJECT

    def cancel(self, _handle):
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        with self.lock:
            reservation = self.reservation
            if (reservation is None or reservation[0] is not handle.request
                    or not self.enabled or self.closed or self.state == 'fault'):
                handle.abort()
                return FollowJointTrajectory.Result(error_code=-1, error_string='Stopped before execution')
            self.reservation = None
            _, plan, tolerance = reservation
            try:
                current = self._feedback()
                if np.max(np.abs(current-plan.start)) > 0.002:
                    raise ValueError('Measured start changed before trajectory execution')
                future = Future()
                self._start(plan, handle=handle, future=future, tolerances=tolerance)
            except ValueError as exc:
                handle.abort()
                return FollowJointTrajectory.Result(error_code=-1, error_string=str(exc))
        code, message, outcome = await future
        if outcome == 'succeeded':
            handle.succeed()
        elif outcome == 'canceled':
            handle.canceled()
        else:
            handle.abort()
        return FollowJointTrajectory.Result(error_code=code, error_string=message)

    def _finish(self, code=0, message='Measured goal settled', outcome='succeeded'):
        active, self.active = self.active, None
        if active is not None and active['future'] is not None and not active['future'].done():
            active['future'].set_result((code, message, outcome))

    def _hold(self, message='Stopped', outcome='aborted'):
        jog, self.calibration_jog = self.calibration_jog, None
        self.reservation = None
        self._finish(-1, message, outcome)
        if self.enabled:
            raw = self._valid_raw(self.bus.read_positions())
            if self.bus.position_limits is None or any(
                    not lo <= t <= hi for t, (lo, hi) in zip(raw, self.bus.position_limits)):
                raise RuntimeError('Cannot confirm a hold target inside servo register limits')
            if jog is not None:
                self.bus.write_single_position(jog['axis'], raw[jog['axis']])
            else:
                self.bus.write_positions(raw)
            self.measured = raw
            self.last_feedback = time.monotonic()
            self.last_written = self.measured
        if self.state != 'fault':
            self.state = 'stopped' if self.enabled else 'disabled'

    def stop(self, _request, _response):
        with self.lock:
            try:
                self._hold()
                return Trigger.Response(success=True, message='Trajectory canceled; holding current position')
            except Exception as exc:
                self._fault(exc)
                return Trigger.Response(success=False, message=str(exc))

    def enable(self, request, _response):
        with self.lock:
            if self.closed:
                return SetBool.Response(success=False, message='Controller is closed')
            if not request.data:
                self.calibration_jog = None
                self.reservation = None
                self._finish(-1, 'Disabled', 'aborted')
                self.enabled = False
                failures = self.bus.disarm() if self.bus else []
                self.state = 'fault' if failures else 'disabled'
                return SetBool.Response(success=not failures, message='; '.join(failures) or 'Disabled')
            try:
                if self.state == 'fault':
                    raise ValueError('Clear fault with enable false before enable true')
                if self.enabled:
                    return SetBool.Response(success=True, message='Already enabled')
                if self.p['home_on_enable']:
                    # Reject unknown homes, out-of-bounds starts and invalid plans
                    # before writing a hold goal or enabling any motor.
                    raw = self._valid_raw(self._unarmed_positions(self.bus.read_positions()))
                    self._home_plan(raw)
                self.measured = self._valid_raw(self.bus.arm(
                    self.p['profile_acceleration'], self.p['profile_velocity'],
                    command_rate_hz=self.p['rate_hz']))
                self.last_feedback = time.monotonic()
                self.last_written = self.measured
                self.enabled = True
                self.state = 'holding'
                if self.p['home_on_enable']:
                    current = self.model.decode(self.measured)
                    self._check_motion_bounds(current)
                    # Re-plan from the actual torque-on position, never a stale sample.
                    plan = self._home_plan(self.measured)
                    tolerance = self.p['home_arrival_tolerance_rad']
                    if np.all(np.abs(current[:6]-plan.points[-1][:6]) <= tolerance):
                        return SetBool.Response(success=True, message=(
                            'Within taught home tolerance; holding measured position, gripper opening held'))
                    self._start(plan, tolerances={'goal': np.full(6, tolerance)})
                    return SetBool.Response(success=True, message=(
                        'Enabled; returning to taught home, gripper opening held. '
                        'Wait for motion_status=holding'))
                return SetBool.Response(success=True, message='Holding measured position; no automatic homing')
            except Exception as exc:
                if self.enabled:
                    self._fault(exc)
                else:
                    self.state = 'fault'
                return SetBool.Response(success=False, message=str(exc))

    def _fault(self, exc, code=-4):
        self._finish(code, str(exc), 'aborted')
        try:
            # Best effort hold uses freshly read feedback, independent of motion bounds.
            self._hold(str(exc))
        except Exception as hold_error:
            self.get_logger().error(f'Hold failed: {hold_error}')
            if self.bus:
                failures = self.bus.disarm()
                if failures:
                    self.get_logger().error('Torque state unconfirmed: ' + '; '.join(failures))
        self.enabled = False
        self.state = 'fault'
        self.get_logger().error(f'Motion fault: {exc}; disable then enable to reset', throttle_duration_sec=2.0)

    def tick(self):
        with self.lock:
            if self.closed:
                return
            try:
                now = time.monotonic()
                if ((self.active or self.calibration_jog) and self.last_tick is not None
                        and now-self.last_tick > self.p['feedback_timeout_s']):
                    raise RuntimeError('Control loop missed its feedback deadline')
                raw = self.bus.read_positions()
                raw = self._valid_raw(raw if self.enabled else self._unarmed_positions(raw))
                self.measured = raw
                if self.enabled and now-self.last_health_check >= 1.0:
                    errors = self.bus.read_hardware_errors()
                    self.last_health_check = time.monotonic()
                    if any(errors):
                        raise RuntimeError(f'Hardware error flags by motor ID: '
                                           f'{dict(zip(self.model.ids, errors))}')
                if time.monotonic()-now > self.p['feedback_timeout_s']:
                    raise RuntimeError('Serial feedback transaction exceeded its deadline')
                self.last_feedback = time.monotonic()
                self.last_tick = self.last_feedback
                if self.calibration_jog is not None:
                    self._step_calibration_jog(self.last_feedback)
                elif self.active:
                    self._step(self.last_feedback)
                if self.measured is not None:
                    self._publish()
            except Exception as exc:
                self._fault(exc)
            self.status_pub.publish(String(data=self.state))

    def _step(self, now):
        active = self.active
        if active['handle'] is not None and active['handle'].is_cancel_requested:
            self._hold('Action canceled', 'canceled')
            return
        current = self.model.decode(self.measured)
        self.model.encode(current)
        lag = np.abs(current-self.model.decode(self.last_written))
        if (np.any(lag[:6] > active['path_tolerance'])
                or lag[6] > self.p['gripper_tracking_error_m']):
            raise RuntimeError('Measured position exceeded trajectory tracking tolerance')
        plan = active['plan']
        elapsed = now-active['started']
        command = plan.sample(elapsed)
        ticks = self.model.encode(command)
        if ticks != self.last_written:
            self.bus.write_positions(ticks)
            self.last_written = ticks
        if active['handle'] is not None:
            feedback = FollowJointTrajectory.Feedback()
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = list(JOINT_NAMES)
            feedback.desired.positions = command[:6].tolist()
            feedback.actual.positions = current[:6].tolist()
            feedback.error.positions = (command[:6]-current[:6]).tolist()
            for point in (feedback.desired, feedback.actual, feedback.error):
                point.time_from_start.sec = int(elapsed)
                point.time_from_start.nanosec = int((elapsed-int(elapsed))*1e9)
            active['handle'].publish_feedback(feedback)
        if elapsed >= plan.duration:
            if not active.get('final_goal_verified', False):
                # SyncWrite has no per-motor acknowledgement. Confirm the final
                # target before relying on a passive arrival wait.
                self.bus.write_verified_positions(ticks)
                active['final_goal_verified'] = True
            error = np.abs(current-plan.points[-1])
            within = (np.all(error[:6] <= active['goal_tolerance'])
                      and error[6] <= self.p['gripper_arrival_tolerance_m'])
            if not within:
                active['stable'] = None
            elif active['stable'] is None:
                active['stable'] = now
            elif now-active['stable'] >= self.p['settle_time_s']:
                self._finish()
                self.state = 'holding'
                return
            if elapsed > plan.duration+active['timeout']:
                self._fault(RuntimeError('Measured goal did not settle before timeout'), code=-5)

    def _publish(self):
        q = self.model.decode(self.measured)
        joints = JointState()
        joints.header.stamp = self.get_clock().now().to_msg()
        joints.name = list(JOINT_NAMES)+['gripper_joint']
        joints.position = q.tolist()  # First six: rad; gripper_joint: full jaw gap in m.
        pose = PoseStamped()
        pose.header.stamp = joints.header.stamp
        pose.header.frame_id = self.frame
        matrix = self.model.forward(q[:6])
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, matrix[:3, 3])
        quat = quaternion_from_matrix(matrix[:3, :3])
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = map(float, quat)
        self.joint_pub.publish(joints)
        self.pose_pub.publish(pose)
        self.tick_pub.publish(Int32MultiArray(data=list(self.measured)))
        self.origin_pub.publish(Int32MultiArray(data=list(self.user_origin_ticks)))
        self.width_pub.publish(Float64(data=float(q[6])))

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.reservation = None
            self.calibration_jog = None
            self._finish(-1, 'Controller shutting down', 'aborted')
            if hasattr(self, 'timer'):
                self.timer.cancel()
            if self.bus:
                try:
                    failures = self.bus.disarm()
                    if failures:
                        self.get_logger().error('Torque-off unconfirmed: ' + '; '.join(failures))
                finally:
                    self.bus.close()

    def destroy_node(self):
        self.close()
        if self.action_server:
            self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = Controller()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
        executor.shutdown(timeout_sec=2.0)
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
