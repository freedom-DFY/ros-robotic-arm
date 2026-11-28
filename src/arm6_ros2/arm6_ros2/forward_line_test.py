"""Live TCP test: from the taught home, move horizontally along a 0.10 m line."""

import argparse
import math
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int32MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint

from arm6_ros2.config import DEFAULTS
from arm6_ros2.model import ArmModel, JOINT_NAMES
from arm6_ros2.trajectory import Plan

DISTANCE_M = 0.10


def forward_plan(model, start, axis, speed, bearing_degrees=None):
    """Preflight every 1 mm in the real execution path, preserving TCP attitude."""
    origin = model.forward(start[:6])
    directions = {'x': [1., 0., 0.], 'y': [0., 1., 0.],
                  '-x': [-1., 0., 0.], '-y': [0., -1., 0.]}
    if axis == 'radial' or bearing_degrees is not None:
        direction = np.r_[origin[:2, 3], 0.]
        length = np.linalg.norm(direction)
        if length < 1e-6:
            raise ValueError('TCP is over the base; specify --axis x or --axis y')
        direction /= length
        if bearing_degrees is not None:
            angle = math.radians(bearing_degrees)
            c, s = math.cos(angle), math.sin(angle)
            direction[:2] = np.array([[c, -s], [s, c]]) @ direction[:2]
    else:
        direction = np.array(directions[axis])
    target = origin.copy()
    target[:3, 3] += DISTANCE_M * direction
    count = math.ceil(DISTANCE_M / min(0.001, model.p['cartesian_resolution_m']))
    if count > model.p['max_waypoints']:
        raise ValueError('Straight-line test exceeds max_waypoints')
    points = []
    q = start[:6].copy()
    for index in range(1, count+1):
        pose = origin.copy()
        pose[:3, 3] += DISTANCE_M * index/count * direction
        try:
            following = model.inverse(pose, q)
        except ValueError as exc:
            raise ValueError(
                f'Horizontal path is unreachable at {DISTANCE_M*index/count*1000:.1f} mm '
                f'of 100 mm: {exc}') from exc
        if np.max(np.abs(following-q)) > model.p['max_ik_joint_step_rad']:
            raise ValueError('Straight line crosses an IK branch or singularity')
        points.append(np.r_[following, start[6]])
        q = following
    automatic = Plan(model, start, points)
    # Respect joint limits and the requested Cartesian peak speed. Leave timing
    # headroom for a small difference in the action server's measured start.
    intervals = np.maximum(np.diff(automatic.times)*1.2,
                           1.875*(DISTANCE_M/count)/speed)
    plan = Plan(model, start, points, np.cumsum(intervals).tolist())
    return plan, origin, target, direction


class LiveLineTest(Node):
    def __init__(self, controller):
        super().__init__('arm6_forward_line_test')
        self.controller = '/' + controller.strip('/')
        self.ticks = None
        self.state = None
        self.tick_time = self.state_time = 0.
        self.goal_handle = None
        self.user_origin = None
        self.tick_sub = self.create_subscription(
            Int32MultiArray, self.controller+'/encoder_ticks', self.on_ticks, 10)
        self.state_sub = self.create_subscription(
            String, self.controller+'/motion_status', self.on_state, 10)
        self.origin_sub = self.create_subscription(
            Int32MultiArray, self.controller+'/user_origin_ticks', self.on_origin, 10)
        self.action = ActionClient(self, FollowJointTrajectory,
                                   self.controller+'/follow_joint_trajectory')

    def on_ticks(self, message):
        if len(message.data) == 7:
            self.ticks = tuple(message.data)
            self.tick_time = time.monotonic()

    def on_state(self, message):
        self.state = message.data
        self.state_time = time.monotonic()

    def on_origin(self, message):
        if len(message.data) == 6:
            self.user_origin = tuple(message.data)

    def wait_future(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('Controller request timed out')
        return future.result()

    def pump(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self, axis, speed, bearing_degrees=None):
        parameters = AsyncParameterClient(self, self.controller)
        if not parameters.wait_for_services(timeout_sec=5.):
            raise RuntimeError('Arm controller is not online')
        names = list(DEFAULTS)
        response = self.wait_future(parameters.get_parameters(names), 5.)
        if any(value.type == 0 for value in response.values):
            raise RuntimeError('Controller parameters are incomplete; load the current controller version')
        settings = dict(zip(names, map(parameter_value_to_python, response.values)))
        model = ArmModel(settings)
        self.pump(0.5)
        if self.ticks is None or min(self.tick_time, self.state_time) < time.monotonic()-settings['feedback_timeout_s']:
            raise RuntimeError('No fresh live encoder/status feedback')
        start = model.decode(self.ticks)
        home = model.decode((self.user_origin or tuple(settings['home_ticks']))+(self.ticks[6],))
        home_error = np.abs(start[:6]-home[:6])
        print(f'Live state={self.state}; ticks={self.ticks}', flush=True)
        print(f'Home error (deg)={np.degrees(home_error).round(3).tolist()}', flush=True)
        home_tolerance = settings['home_arrival_tolerance_rad']
        print(f'Home-position check: {"PASS" if np.all(home_error <= home_tolerance) else "FAIL"}; '
              f'tolerance={math.degrees(home_tolerance):.2f} deg per joint', flush=True)
        blockers = []
        if self.state != 'holding':
            blockers.append(f'controller state is {self.state}, expected holding')
        if not settings['home_confirmed']:
            blockers.append('taught home is not confirmed')
        for index in np.flatnonzero(home_error > home_tolerance):
            blockers.append(f'ID{model.ids[index]} is {math.degrees(home_error[index]):.2f} deg '
                            f'from home, exceeding the {math.degrees(home_tolerance):.2f} deg '
                            'home_arrival_tolerance_rad; this is separate from motor motion limits')
        if not settings['calibration_confirmed']:
            blockers.append('Manual encoder-calibration confirmation is pending '
                            '(calibration_confirmed=false): verify CAD-pose encoder zeros, '
                            'joint directions and measured limits; this is not a home-position error')
        if not settings['geometry_confirmed']:
            blockers.append('Manual geometry confirmation is pending (geometry_confirmed=false): '
                            'verify the physical assembly against the CAD DH model and reference TCP; '
                            'this is not a home-position error')
        try:
            plan, origin, target, direction = forward_plan(model, start, axis, speed, bearing_degrees)
        except ValueError as exc:
            blockers.append(str(exc))
        if blockers:
            raise RuntimeError('Live test not started:\n  - '+'\n  - '.join(blockers))
        print(f'Frame={settings["base_frame"]}; horizontal direction={direction.tolist()}', flush=True)
        print(f'TCP start (m)={origin[:3, 3].tolist()}; target (m)={target[:3, 3].tolist()}', flush=True)
        print(f'Distance=100 mm; duration={plan.duration:.2f}s; gripper opening held', flush=True)
        self.pump(0.2)
        fresh = model.decode(self.ticks)
        if np.max(np.abs(fresh-start)) > 0.002:
            raise RuntimeError('Measured starting pose changed during path planning')
        if self.state != 'holding':
            raise RuntimeError(f'Controller changed state to {self.state}')
        if not self.action.wait_for_server(timeout_sec=5.):
            raise RuntimeError('Trajectory action is not online')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        for when, state in zip(plan.times[1:], plan.points[1:]):
            point = JointTrajectoryPoint()
            point.positions = state[:6].tolist()
            nanoseconds = round(when*1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 10**9)
            goal.trajectory.points.append(point)
        self.goal_handle = self.wait_future(self.action.send_goal_async(goal), 5.)
        if not self.goal_handle.accepted:
            self.goal_handle = None
            raise RuntimeError('Controller rejected the live straight-line trajectory')
        result = self.goal_handle.get_result_async()
        deadline = time.monotonic()+plan.duration+settings['arrival_timeout_s']+10.
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > deadline:
                raise RuntimeError('Live test result timed out')
            if min(self.tick_time, self.state_time) < time.monotonic()-settings['feedback_timeout_s']:
                raise RuntimeError('Live feedback became stale during motion')
            if self.state == 'fault':
                raise RuntimeError('Controller fault during live motion')
            position = model.forward(model.decode(self.ticks)[:6])[:3, 3]
            along = float(np.dot(position-origin[:3, 3], direction))
            nearest = origin[:3, 3]+np.clip(along, 0., DISTANCE_M)*direction
            if np.linalg.norm(position-nearest) > 0.005:
                raise RuntimeError('Encoder-derived TCP left the line by more than 5 mm')
        outcome = result.result().result
        self.goal_handle = None
        if outcome.error_code != 0:
            raise RuntimeError(f'Controller reported {outcome.error_code}: {outcome.error_string}')
        self.pump(0.2)
        end = model.forward(model.decode(self.ticks)[:6])
        delta = end[:3, 3]-origin[:3, 3]
        print(f'Encoder-derived TCP displacement (mm)={(1000*delta).round(3).tolist()}', flush=True)
        if np.linalg.norm(end[:3, 3]-target[:3, 3]) > 0.005:
            raise RuntimeError('Encoder-derived TCP endpoint differs from target by more than 5 mm')
        print('Live straight-line test completed; controller remains enabled at the endpoint.', flush=True)

    def cancel_own_motion(self):
        if self.goal_handle is not None:
            try:
                self.wait_future(self.goal_handle.cancel_goal_async(), 3.)
            except Exception as exc:
                self.get_logger().error(f'Could not confirm trajectory cancellation: {exc}')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', default='/arm6')
    direction = parser.add_mutually_exclusive_group()
    direction.add_argument('--axis', choices=['radial', 'x', 'y', '-x', '-y'], default='radial')
    direction.add_argument('--bearing-deg', type=float,
                           help='Angle from base-to-TCP horizontal forward; positive left, negative right')
    parser.add_argument('--speed', type=float, default=0.005, help='TCP peak speed in m/s (default 0.005)')
    options = parser.parse_args(args)
    if not math.isfinite(options.speed) or not 0 < options.speed <= 0.005:
        parser.error('--speed must be greater than zero and at most 0.005 m/s')
    if options.bearing_deg is not None and (not math.isfinite(options.bearing_deg)
                                          or not -180 <= options.bearing_deg <= 180):
        parser.error('--bearing-deg must be finite and between -180 and 180 degrees')
    # Keep the ROS context alive until Ctrl+C has cancelled our own live goal.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = LiveLineTest(options.controller)
    code = 0
    try:
        node.run(options.axis, options.speed, options.bearing_deg)
    except (ValueError, RuntimeError) as exc:
        print(str(exc), flush=True)
        node.cancel_own_motion()
        code = 1
    except KeyboardInterrupt:
        node.cancel_own_motion()
        code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if code:
        raise SystemExit(code)
