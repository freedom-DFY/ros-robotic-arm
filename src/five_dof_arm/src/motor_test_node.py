#!/usr/bin/env python3
"""Command individual Protocol 2.0 motors using the X/MX(2.0) control table."""
import math

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
import rclpy
from rclpy.node import Node


class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')
        self.ids = list(self.declare_parameter('motor_ids', [4, 5]).value)
        self.dry_run = self.declare_parameter('dry_run', True).value
        self.enable_torque = self.declare_parameter('enable_torque', False).value
        self.device = self.declare_parameter('device_name', '/dev/ttyUSB0').value
        self.baud = self.declare_parameter('baud_rate', 1000000).value
        self.lower = self.declare_parameter('position_min', 0).value
        self.upper = self.declare_parameter('position_max', 4095).value
        self.velocity = self.declare_parameter('profile_velocity', 20).value
        self.acceleration = self.declare_parameter('profile_acceleration', 10).value
        self.duration_ms = self.declare_parameter('profile_duration_ms', 2000).value
        self.acceleration_ms = self.declare_parameter('profile_acceleration_time_ms', 500).value
        self.expected_model = self.declare_parameter('expected_model_number', 0).value
        self.rate = self.declare_parameter('publish_rate', 10.0).value
        self.disable_on_exit = self.declare_parameter('disable_torque_on_shutdown', True).value
        if (not self.ids or len(set(self.ids)) != len(self.ids) or
                any(i < 0 or i > 252 for i in self.ids)):
            raise ValueError('motor_ids must be unique IDs in 0..252')
        if not 0 <= self.lower <= self.upper <= 4095:
            raise ValueError('Position limits must satisfy 0 <= min <= max <= 4095')
        if (self.baud <= 0 or self.velocity <= 0 or self.acceleration <= 0 or
                self.velocity > 32767 or self.acceleration > 32767 or
                not math.isfinite(self.rate) or not 0 < self.rate <= 100):
            raise ValueError('Invalid baud rate, profile or publish rate')
        if (not 1 <= self.duration_ms <= 32737 or
                not 1 <= self.acceleration_ms <= self.duration_ms / 2 or self.expected_model < 0):
            raise ValueError('Invalid expected model or time-based profile (acceleration time <= duration/2)')
        self.positions = {i: max(self.lower, min(2048, self.upper)) for i in self.ids}
        self.port = None
        self.packet = None
        self.opened = False
        self.enabled_ids = []
        try:
            if not self.dry_run:
                self.port = PortHandler(self.device)
                self.packet = PacketHandler(2.0)
                if not self.port.openPort():
                    raise RuntimeError('Cannot open ' + self.device)
                self.opened = True
                if not self.port.setBaudRate(self.baud):
                    raise RuntimeError('Cannot set baud rate')
                # Ping every requested ID before making any configuration writes.
                for motor_id in self.ids:
                    model, result, error = self.packet.ping(self.port, motor_id)
                    self.check(result, error, motor_id)
                    if self.expected_model and model != self.expected_model:
                        raise RuntimeError(f'ID {motor_id}: model {model}, expected {self.expected_model}')
                    self.get_logger().info(f'ID {motor_id}: model number {model}')
                drive_modes = {}
                for motor_id in self.ids:
                    drive_mode, result, error = self.packet.read1ByteTxRx(self.port, motor_id, 10)
                    self.check(result, error, motor_id)
                    drive_modes[motor_id] = drive_mode
                    units = 'time-based (ms)' if drive_mode & 4 else 'velocity-based'
                    self.get_logger().info(f'ID {motor_id}: Drive Mode={drive_mode}, {units}')
                    if self.enable_torque and drive_mode & 8:
                        raise RuntimeError(
                            f'ID {motor_id}: disable Torque On by Goal Update (Drive Mode bit 3) first')
                if self.enable_torque:
                    for motor_id in self.ids:
                        self.write(motor_id, 64, 0, one_byte=True)
                        self.write(motor_id, 11, 3, one_byte=True)
                        if drive_modes[motor_id] & 4:
                            self.write(motor_id, 108, self.acceleration_ms)
                            self.write(motor_id, 112, self.duration_ms)
                        else:
                            self.write(motor_id, 108, self.acceleration)
                            self.write(motor_id, 112, self.velocity)
                for motor_id in self.ids:
                    position = self.read(motor_id)
                    self.positions[motor_id] = position
                    self.get_logger().info(f'ID {motor_id}: present position {position}')
                    if self.enable_torque:
                        if not 0 <= position <= 4095:
                            raise RuntimeError(f'ID {motor_id}: invalid single-turn startup position')
                        # Keep the current physical pose when enabling torque.
                        self.write(motor_id, 116, position)
                if self.enable_torque:
                    for motor_id in self.ids:
                        # Include the ID before the write, to cover lost acknowledgements.
                        self.enabled_ids.append(motor_id)
                        self.write(motor_id, 64, 1, one_byte=True)
            self.publisher = self.create_publisher(SetPosition, 'motor_test/position', 10)
            self.subscription = self.create_subscription(
                SetPosition, 'motor_test/set_position', self.set_position, 10)
            self.service = self.create_service(
                GetPosition, 'motor_test/get_position', self.get_position)
            self.timer = self.create_timer(1.0 / self.rate, self.publish_positions)
        except Exception:
            self.close()
            self.destroy_node()
            raise
        mode = 'dry_run' if self.dry_run else ('torque enabled' if self.enable_torque else 'READ ONLY')
        self.get_logger().info(f'Motor test ready: IDs={self.ids}, {mode}')

    def check(self, result, error, motor_id):
        if result != COMM_SUCCESS:
            raise RuntimeError(f'ID {motor_id}: {self.packet.getTxRxResult(result)}')
        if error:
            raise RuntimeError(f'ID {motor_id}: {self.packet.getRxPacketError(error)}')

    def write(self, motor_id, address, value, one_byte=False):
        method = self.packet.write1ByteTxRx if one_byte else self.packet.write4ByteTxRx
        result, error = method(self.port, motor_id, address, value)
        self.check(result, error, motor_id)

    def read(self, motor_id):
        position, result, error = self.packet.read4ByteTxRx(self.port, motor_id, 132)
        self.check(result, error, motor_id)
        # Present Position is signed in the device control table.
        return position if position < 0x80000000 else position - 0x100000000

    def set_position(self, msg):
        if msg.id not in self.ids:
            self.get_logger().error(f'Rejected ID {msg.id}: configured IDs are {self.ids}')
            return
        if not self.lower <= msg.position <= self.upper:
            self.get_logger().error(f'Rejected position: valid range {self.lower}..{self.upper}')
            return
        if not self.dry_run and not self.enable_torque:
            self.get_logger().error('READ ONLY: restart with enable_torque:=true to send position commands')
            return
        try:
            if not self.dry_run:
                self.write(msg.id, 116, msg.position)
            else:
                self.positions[msg.id] = msg.position
            self.get_logger().info(f'ID {msg.id}: goal position {msg.position}')
        except RuntimeError as error:
            self.get_logger().error(str(error))

    def get_position(self, request, response):
        # This upstream service has no success field. -1 indicates failure.
        response.position = -1
        if request.id not in self.ids:
            self.get_logger().error(f'Cannot read unconfigured ID {request.id}')
            return response
        try:
            position = self.positions[request.id] if self.dry_run else self.read(request.id)
            if not 0 <= position <= 4095:
                raise RuntimeError('Present position is outside single-turn range; check operating mode')
            response.position = position
        except RuntimeError as error:
            self.get_logger().error(str(error))
        return response

    def publish_positions(self):
        for motor_id in self.ids:
            try:
                position = self.positions[motor_id] if self.dry_run else self.read(motor_id)
                msg = SetPosition()
                msg.id, msg.position = motor_id, position
                self.publisher.publish(msg)
            except RuntimeError as error:
                self.get_logger().error(str(error), throttle_duration_sec=2.0)

    def close(self):
        if not self.opened:
            return
        if self.disable_on_exit:
            for motor_id in self.enabled_ids:
                try:
                    self.write(motor_id, 64, 0, one_byte=True)
                except RuntimeError as error:
                    self.get_logger().error(f'Torque disable failed: {error}')
        self.port.closePort()
        self.opened = False


def main():
    rclpy.init()
    node = None
    try:
        node = MotorTestNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as error:
        print(f'motor_test_node failed: {error}', flush=True)
        raise SystemExit(1)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
