"""Exercise hardware setup with a fake SDK; no serial port or DDS is opened."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition

spec = importlib.util.spec_from_file_location(
    'motor_test_node', Path(__file__).resolve().parents[1] / 'src' / 'motor_test_node.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MotorTest(unittest.TestCase):
    def setUp(self):
        self.params = {'dry_run': False}
        self.port = MagicMock()
        self.port.openPort.return_value = True
        self.port.setBaudRate.return_value = True
        self.packet = MagicMock()
        self.packet.ping.return_value = (1020, 0, 0)
        self.packet.read1ByteTxRx.return_value = (0, 0, 0)
        self.packet.read4ByteTxRx.side_effect = lambda port, motor_id, address: (
            {4: 2100, 5: 2200}[motor_id], 0, 0)
        self.events = []

        def record_write(port, motor_id, address, value):
            self.events.append((motor_id, address, value))
            return 0, 0

        self.packet.write1ByteTxRx.side_effect = record_write
        self.packet.write4ByteTxRx.side_effect = record_write
        self.packet.getTxRxResult.return_value = 'fake communication error'
        self.packet.getRxPacketError.return_value = 'fake device error'
        patches = [
            patch.object(module.Node, '__init__', return_value=None),
            patch.object(module.Node, 'declare_parameter', side_effect=lambda name, default:
                         SimpleNamespace(value=self.params.get(name, default))),
            patch.object(module.Node, 'get_logger', return_value=MagicMock()),
            patch.object(module.Node, 'create_publisher', return_value=MagicMock()),
            patch.object(module.Node, 'create_subscription', return_value=MagicMock()),
            patch.object(module.Node, 'create_service', return_value=MagicMock()),
            patch.object(module.Node, 'create_timer', return_value=MagicMock()),
            patch.object(module.Node, 'destroy_node', return_value=None),
            patch.object(module, 'PortHandler', return_value=self.port),
            patch.object(module, 'PacketHandler', return_value=self.packet),
        ]
        for item in patches:
            item.start()
            self.addCleanup(item.stop)

    def node(self):
        node = module.MotorTestNode()
        self.addCleanup(node.close)
        return node

    def test_read_only_opens_only_ids_4_5_and_never_writes(self):
        node = self.node()
        self.assertEqual([c.args[1] for c in self.packet.ping.call_args_list], [4, 5])
        self.assertEqual(self.events, [])
        node.set_position(SetPosition(id=4, position=2150))
        self.assertEqual(self.events, [])
        response = node.get_position(GetPosition.Request(id=5), GetPosition.Response())
        self.assertEqual(response.position, 2200)
        node.close()
        self.assertEqual(self.events, [])

    def test_seeds_actual_positions_before_enabling_torque(self):
        self.params['enable_torque'] = True
        node = self.node()
        first_enable = next(i for i, event in enumerate(self.events) if event[1:] == (64, 1))
        self.assertIn((4, 116, 2100), self.events[:first_enable])
        self.assertIn((5, 116, 2200), self.events[:first_enable])
        self.assertTrue(all(event[0] in (4, 5) for event in self.events))
        self.events.clear()
        node.set_position(SetPosition(id=4, position=2110))
        self.assertEqual(self.events, [(4, 116, 2110)])
        node.close()
        self.assertEqual(self.events[-2:], [(4, 64, 0), (5, 64, 0)])

    def test_rejects_unconfigured_id_and_out_of_range_target(self):
        self.params['enable_torque'] = True
        node = self.node()
        self.events.clear()
        for msg in [SetPosition(id=1, position=2048), SetPosition(id=4, position=-1),
                    SetPosition(id=5, position=4096)]:
            node.set_position(msg)
        self.assertEqual(self.events, [])
        self.assertEqual(node.get_position(
            GetPosition.Request(id=1), GetPosition.Response()).position, -1)

    def test_time_based_drive_uses_milliseconds_and_preserves_direction(self):
        self.params['enable_torque'] = True
        self.params['expected_model_number'] = 1000
        self.packet.ping.return_value = (1000, 0, 0)
        self.packet.read1ByteTxRx.side_effect = [(4, 0, 0), (5, 0, 0)]
        self.node()
        for motor_id in [4, 5]:
            self.assertIn((motor_id, 108, 500), self.events)
            self.assertIn((motor_id, 112, 2000), self.events)
        self.assertFalse(any(address == 10 for _, address, _ in self.events))

    def test_model_mismatch_fails_before_any_write(self):
        self.params.update(enable_torque=True, expected_model_number=1000)
        with self.assertRaisesRegex(RuntimeError, 'model 1020, expected 1000'):
            module.MotorTestNode()
        self.assertEqual(self.events, [])
        self.port.closePort.assert_called_once()

    def test_auto_torque_on_goal_update_rejected_before_writes(self):
        self.params['enable_torque'] = True
        self.packet.read1ByteTxRx.return_value = (12, 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'Torque On by Goal Update'):
            module.MotorTestNode()
        self.assertEqual(self.events, [])

    def test_missing_id_fails_before_any_configuration_write(self):
        self.params['enable_torque'] = True
        self.packet.ping.side_effect = [(1020, 0, 0), (0, -3001, 0)]
        with self.assertRaisesRegex(RuntimeError, 'ID 5'):
            module.MotorTestNode()
        self.assertEqual(self.events, [])
        self.port.closePort.assert_called_once()

    def test_read_failure_returns_error_sentinel(self):
        node = self.node()
        self.packet.read4ByteTxRx.side_effect = None
        self.packet.read4ByteTxRx.return_value = (0, -3001, 0)
        self.assertEqual(node.get_position(
            GetPosition.Request(id=4), GetPosition.Response()).position, -1)

    def test_lost_torque_enable_acknowledgement_attempts_disable(self):
        self.params['enable_torque'] = True

        def write(port, motor_id, address, value):
            self.events.append((motor_id, address, value))
            return (-3001, 0) if (address, value) == (64, 1) else (0, 0)

        self.packet.write1ByteTxRx.side_effect = write
        with self.assertRaisesRegex(RuntimeError, 'ID 4'):
            module.MotorTestNode()
        self.assertIn((4, 64, 0), self.events[self.events.index((4, 64, 1)) + 1:])
        self.port.closePort.assert_called_once()

    def test_dry_run_uses_no_serial_and_keeps_motors_independent(self):
        self.params['dry_run'] = True
        node = self.node()
        module.PortHandler.assert_not_called()
        node.set_position(SetPosition(id=4, position=2060))
        self.assertEqual(node.positions, {4: 2060, 5: 2048})
        self.assertEqual(node.get_position(
            GetPosition.Request(id=4), GetPosition.Response()).position, 2060)


if __name__ == '__main__':
    unittest.main()
