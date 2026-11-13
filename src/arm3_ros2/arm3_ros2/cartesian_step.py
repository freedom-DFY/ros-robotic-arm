"""Send one Cartesian line relative to fresh, stable real motor feedback."""

import argparse
import json
import math
import sys
import time

from arm3_ros2.kinematics import Calibration, WristKinematics


def prepare_relative_line(parameters, ticks, delta):
    if parameters['dry_run'] is not False:
        raise ValueError('This command requires real hardware: dry_run must be false')
    if parameters['calibration_confirmed'] is not True:
        raise ValueError('Reference calibration and trial bounds are not confirmed')
    if list(parameters['motor_ids']) != [1, 2, 3]:
        raise ValueError('Expected motor IDs [1,2,3]')
    if len(delta) != 3 or not all(math.isfinite(d) for d in delta):
        raise ValueError('Expected three finite displacement components in metres')
    length = math.sqrt(sum(d * d for d in delta))
    if not 0 < length <= 0.2 + 1e-10:
        raise ValueError('Displacement must be positive and at most 200 mm')
    cal = Calibration(parameters['zero_ticks'], parameters['joint_directions'],
                      parameters['position_min_ticks'], parameters['position_max_ticks'])
    if not cal.contains_ticks(ticks) or any(type(t) is not int for t in ticks):
        raise ValueError('Current feedback is outside the loaded trial bounds')
    kin = WristKinematics(parameters['upper_arm_m'], parameters['forearm_m'],
                          parameters['base_height_m'], parameters['dh_offsets_rad'], cal)
    start = cal.radians(ticks)
    origin = kin.forward(start)
    target = tuple(a + d for a, d in zip(origin, delta))
    path = kin.line_path(start, target, parameters['path_resolution_m'],
                         parameters['max_path_joint_step_rad'])
    return {
        'start_ticks': list(ticks), 'start_xyz_m': list(origin),
        'delta_xyz_m': list(delta), 'target_xyz_m': list(target),
        'length_mm': length * 1000, 'frame_id': parameters['base_frame'],
        'waypoints': len(path), 'expected_end_ticks': list(cal.ticks(path[-1][0])),
    }


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dx', type=float, default=0.0, help='X displacement, metres')
    parser.add_argument('--dy', type=float, default=0.0, help='Y displacement, metres')
    parser.add_argument('--dz', type=float, default=0.0, help='Z displacement, metres')
    options = parser.parse_args(args)

    import rclpy
    from geometry_msgs.msg import PointStamped
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import Int32MultiArray

    rclpy.init(args=[])
    node = rclpy.create_node('arm3_cartesian_step')
    try:
        samples = []
        subscription = node.create_subscription(
            Int32MultiArray, '/arm3/encoder_ticks',
            lambda msg: samples.append((time.monotonic(), list(msg.data))), 1)
        publisher = node.create_publisher(PointStamped, '/arm3/cartesian_target', 1)
        client = node.create_client(GetParameters, '/arm3/get_parameters')
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('No /arm3 service; start the real three-axis node first')
        names = ['dry_run', 'calibration_confirmed', 'motor_ids', 'zero_ticks', 'joint_directions',
                 'position_min_ticks', 'position_max_ticks', 'upper_arm_m', 'forearm_m',
                 'base_height_m', 'dh_offsets_rad', 'base_frame',
                 'path_resolution_m', 'max_path_joint_step_rad']
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('Parameter query timed out')
        parameters = {}
        for key, value in zip(names, future.result().values):
            if value.type == 1:
                parameters[key] = value.bool_value
            elif value.type == 3:
                parameters[key] = value.double_value
            elif value.type == 4:
                parameters[key] = value.string_value
            elif value.type == 7:
                parameters[key] = list(value.integer_array_value)
            elif value.type == 8:
                parameters[key] = list(value.double_array_value)
            else:
                raise RuntimeError(f'Missing or unexpected parameter: {key}')
        deadline = time.monotonic() + 3.0
        while len(samples) < 5 or publisher.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('No fresh encoder feedback or Cartesian command subscriber')
            rclpy.spin_once(node, timeout_sec=0.05)
        recent = [ticks for _, ticks in samples[-5:]]
        if time.monotonic() - samples[-1][0] > 0.25:
            raise RuntimeError('Encoder feedback is stale')
        if any(len(ticks) != 3 for ticks in recent):
            raise RuntimeError('Expected three encoder readings per message')
        if any(max(t[i] for t in recent) - min(t[i] for t in recent) > 3 for i in range(3)):
            raise RuntimeError('The arm is moving; wait for it to stop before another relative command')
        plan = prepare_relative_line(parameters, recent[-1], (options.dx, options.dy, options.dz))
        msg = PointStamped()
        msg.header.frame_id = plan['frame_id']
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = plan['target_xyz_m']
        publisher.publish(msg)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        plan['status'] = 'Target published; controller feedback is needed to confirm completion'
        print(json.dumps(plan, indent=2))
        assert subscription is not None
    except (ValueError, RuntimeError) as exc:
        print(f'No further command sent: {exc}', file=sys.stderr)
        raise SystemExit(1) from exc
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
