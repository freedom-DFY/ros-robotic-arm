"""Verify initial holding with a fake SDK, without opening a real serial port."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from arm3_ros2 import dynamixel_bus as bus_module


@pytest.fixture
def sdk(monkeypatch):
    port = MagicMock()
    port.openPort.return_value = True
    port.setBaudRate.return_value = True
    packet = MagicMock()
    packet.ping.return_value = (1120, 0, 0)
    packet.getTxRxResult.return_value = 'communication error'
    packet.getRxPacketError.return_value = 'device error'
    reader = MagicMock()
    reader.addParam.return_value = True
    reader.txRxPacket.return_value = 0
    reader.isAvailable.return_value = True
    positions = {1: 3811, 2: 999, 3: 2040}
    reader.getData.side_effect = lambda motor_id, address, size: positions[motor_id]
    registers = {}
    for motor_id in (1, 2, 3):
        for address, value in {
            bus_module.TORQUE_ENABLE: 0, bus_module.OPERATING_MODE: 3,
            bus_module.DRIVE_MODE: 4, bus_module.HOMING_OFFSET: 0,
            bus_module.MIN_POSITION_LIMIT: 0, bus_module.MAX_POSITION_LIMIT: 4095,
            bus_module.HARDWARE_ERROR: 0, bus_module.GOAL_POSITION: 0,
            bus_module.BUS_WATCHDOG: 0,
            bus_module.PRESENT_POSITION: positions[motor_id],
        }.items():
            registers[motor_id, address] = value
    events = []

    def read(port, motor_id, address):
        return registers[motor_id, address], 0, 0

    def write(port, motor_id, address, value):
        events.append((motor_id, address, value))
        registers[motor_id, address] = value
        return 0, 0

    packet.read1ByteTxRx.side_effect = read
    packet.read4ByteTxRx.side_effect = read
    packet.write1ByteTxRx.side_effect = write
    packet.write4ByteTxRx.side_effect = write
    monkeypatch.setattr(bus_module, 'PortHandler', lambda device: port)
    monkeypatch.setattr(bus_module, 'PacketHandler', lambda protocol: packet)
    monkeypatch.setattr(bus_module, 'GroupSyncRead', lambda *args: reader)
    monkeypatch.setattr(bus_module, 'GroupSyncWrite', lambda *args: MagicMock())
    monkeypatch.setattr(bus_module, 'sleep', lambda _: None)
    return SimpleNamespace(port=port, packet=packet, reader=reader,
                           positions=positions, registers=registers, events=events)


def test_enable_preloads_actual_positions_before_any_torque_on(sdk):
    bus = bus_module.DynamixelBus('/fake/port', 1000000, (1, 2, 3))
    assert bus.arm(5, 12, command_rate_hz=30.0) == (3811, 999, 2040)
    first_on = next(i for i, (_, address, value) in enumerate(sdk.events)
                    if (address, value) == (bus_module.TORQUE_ENABLE, 1))
    for motor_id in (1, 2, 3):
        assert (motor_id, bus_module.GOAL_POSITION, sdk.positions[motor_id]) in sdk.events[:first_on]
    assert bus.position_limits == ((0, 4095),) * 3
    assert bus.possibly_enabled == {1, 2, 3}
    assert not bus.disarm()
    bus.close()


@pytest.mark.parametrize('address,value', [
    (bus_module.OPERATING_MODE, 4),
    (bus_module.DRIVE_MODE, 12),
    (bus_module.HOMING_OFFSET, 100),
    (bus_module.HARDWARE_ERROR, 1),
])
def test_servo_configuration_fault_still_prevents_enable(sdk, address, value):
    sdk.registers[3, address] = value
    bus = bus_module.DynamixelBus('/fake/port', 1000000, (1, 2, 3))
    with pytest.raises(RuntimeError):
        bus.arm(5, 12, command_rate_hz=30.0)
    assert not sdk.events and not bus.possibly_enabled
    bus.close()


def test_hold_goal_cannot_be_clipped_by_servo_internal_limits(sdk):
    sdk.registers[1, bus_module.MIN_POSITION_LIMIT] = 3820
    bus = bus_module.DynamixelBus('/fake/port', 1000000, (1, 2, 3))
    with pytest.raises(RuntimeError, match="servo's internal limits"):
        bus.arm(5, 12, command_rate_hz=30.0)
    assert not sdk.events and not bus.possibly_enabled
    bus.close()


def test_failed_torque_on_acknowledgement_attempts_torque_off(sdk):
    original_write = sdk.packet.write1ByteTxRx.side_effect

    def fail_on_enable(port, motor_id, address, value):
        result = original_write(port, motor_id, address, value)
        return (-1, 0) if (address, value) == (bus_module.TORQUE_ENABLE, 1) else result

    sdk.packet.write1ByteTxRx.side_effect = fail_on_enable
    bus = bus_module.DynamixelBus('/fake/port', 1000000, (1, 2, 3))
    with pytest.raises(RuntimeError, match='communication error'):
        bus.arm(5, 12, command_rate_hz=30.0)
    assert sdk.registers[1, bus_module.TORQUE_ENABLE] == 0
    assert not bus.possibly_enabled
    bus.close()


def test_hold_readback_is_rechecked_before_any_torque_on(sdk):
    original_read = sdk.packet.read4ByteTxRx.side_effect
    goal_reads = []

    def delayed_read(port, motor_id, address):
        if motor_id == 1 and address == bus_module.GOAL_POSITION:
            goal_reads.append(len(sdk.events))
            if len(goal_reads) < 3:
                assert not any(address == bus_module.TORQUE_ENABLE and value == 1
                               for _, address, value in sdk.events)
                return 0, 0, 0
        return original_read(port, motor_id, address)

    sdk.packet.read4ByteTxRx.side_effect = delayed_read
    bus = bus_module.DynamixelBus('/fake/port', 1000000, (1, 2, 3))
    assert bus.arm(5, 12, command_rate_hz=30.0) == (3811, 999, 2040)
    assert len(goal_reads) == 3
    # Readback retries must never introduce extra goal writes.
    assert sdk.events.count((1, bus_module.GOAL_POSITION, 3811)) == 1
    assert not bus.disarm()
    bus.close()


@pytest.mark.parametrize('failed_id', [1, 3])
def test_persistent_hold_mismatch_reports_values_without_enabling_any_motor(sdk, failed_id):
    original_write = sdk.packet.write4ByteTxRx.side_effect

    def ignore_goal(port, motor_id, address, value):
        if motor_id == failed_id and address == bus_module.GOAL_POSITION:
            sdk.events.append((motor_id, address, value))
            return 0, 0
        return original_write(port, motor_id, address, value)

    sdk.packet.write4ByteTxRx.side_effect = ignore_goal
    bus = bus_module.DynamixelBus('/fake/port', 1000000, (1, 2, 3))
    with pytest.raises(RuntimeError) as failure:
        bus.arm(5, 12, command_rate_hz=30.0)
    message = str(failure.value)
    assert 'Motor {} did not accept'.format(failed_id) in message
    assert 'requested={}'.format(sdk.positions[failed_id]) in message
    assert 'goal_readbacks=[0, 0, 0]' in message
    assert 'present_position={}'.format(sdk.positions[failed_id]) in message
    assert 'torque_enable=0' in message and 'bus_watchdog=0' in message
    assert not any(address == bus_module.TORQUE_ENABLE and value == 1
                   for _, address, value in sdk.events)
    assert not bus.possibly_enabled
    bus.close()


def test_diagnostic_read_failure_does_not_hide_hold_mismatch(sdk):
    original_read = sdk.packet.read4ByteTxRx.side_effect

    def failed_read(port, motor_id, address):
        if address == bus_module.GOAL_POSITION:
            return 123, 0, 0
        if address == bus_module.PRESENT_POSITION:
            return 0, -1, 0
        return original_read(port, motor_id, address)

    sdk.packet.read4ByteTxRx.side_effect = failed_read
    bus = bus_module.DynamixelBus('/fake/port', 1000000, (1, 2, 3))
    with pytest.raises(RuntimeError, match='goal_readbacks=') as failure:
        bus.arm(5, 12, command_rate_hz=30.0)
    assert 'present_position=unavailable' in str(failure.value)
    assert 'communication error' in str(failure.value)
    assert not bus.possibly_enabled
    bus.close()
