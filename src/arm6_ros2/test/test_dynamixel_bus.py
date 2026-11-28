"""Verify initial holding with a fake SDK, without opening a real serial port."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from arm6_ros2 import dynamixel_bus as bus_module
from arm6_ros2.model import ArmModel, RADIANS_PER_TICK


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
    writer = MagicMock()
    writer.addParam.return_value = True
    writer.txPacket.return_value = 0
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
    monkeypatch.setattr(bus_module, 'GroupSyncWrite', lambda *args: writer)
    monkeypatch.setattr(bus_module, 'sleep', lambda _: None)
    return SimpleNamespace(port=port, packet=packet, reader=reader, writer=writer,
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


def seven_motors(sdk):
    for motor_id in range(4, 8):
        sdk.positions[motor_id] = 2048
        for (original_id, address), value in list(sdk.registers.items()):
            if original_id == 1:
                sdk.registers[motor_id, address] = value
    sdk.packet.ping.side_effect = lambda port, motor_id: (1120 if motor_id <= 3 else 1020, 0, 0)
    return tuple(range(1, 8))


def test_seven_mixed_model_motors_preload_all_goals_before_torque(sdk):
    ids = seven_motors(sdk)
    bus = bus_module.DynamixelBus('/fake/port', 1000000, ids)
    assert bus.models == (1120, 1120, 1120, 1020, 1020, 1020, 1020)
    assert bus.arm(5, 12, command_rate_hz=30.0) == tuple(sdk.positions[i] for i in ids)
    first_on = next(i for i, (_, address, value) in enumerate(sdk.events)
                    if (address, value) == (bus_module.TORQUE_ENABLE, 1))
    for motor_id in ids:
        assert (motor_id, bus_module.GOAL_POSITION, sdk.positions[motor_id]) in sdk.events[:first_on]
    assert bus.read_hardware_errors() == (0,)*7
    with pytest.raises(ValueError):
        bus.write_positions((2048,)*6)
    with pytest.raises(ValueError):
        bus.write_positions((2048,)*6+(-1,))
    assert not bus.disarm()
    bus.close()


def test_seventh_motor_enable_failure_rolls_back_all_seven(sdk):
    ids = seven_motors(sdk)
    original_write = sdk.packet.write1ByteTxRx.side_effect

    def fail_last(port, motor_id, address, value):
        result = original_write(port, motor_id, address, value)
        return (-1, 0) if (motor_id, address, value) == (7, bus_module.TORQUE_ENABLE, 1) else result

    sdk.packet.write1ByteTxRx.side_effect = fail_last
    bus = bus_module.DynamixelBus('/fake/port', 1000000, ids)
    with pytest.raises(RuntimeError, match='communication error'):
        bus.arm(5, 12, command_rate_hz=30.0)
    assert not bus.possibly_enabled
    assert all(sdk.registers[motor_id, bus_module.TORQUE_ENABLE] == 0 for motor_id in ids)
    bus.close()


@pytest.mark.parametrize('raw_gripper,canonical', [(3550, 3550), (100, 4196),
                                                  (-546, 3550), (15838, 3550)])
def test_wrapped_gripper_anchors_actual_counter_without_full_turn_jump(sdk, raw_gripper, canonical):
    ids = seven_motors(sdk)
    sdk.positions[7] = raw_gripper & 0xffffffff
    sdk.registers[7, bus_module.OPERATING_MODE] = 4
    bus = bus_module.DynamixelBus('/fake/port', 1000000, ids, gripper_wrap_limits=(3180, 4376))
    try:
        held = bus.arm(5, 12, command_rate_hz=30.0)
        assert held[-1] == canonical
        assert sdk.registers[7, bus_module.GOAL_POSITION] == raw_gripper & 0xffffffff
        assert bus.position_limits[-1] == (3180, 4376)
        target = held[:-1] + (4376,)
        bus.write_positions(target)
        words = {call.args[0]: int.from_bytes(bytes(call.args[1]), 'little')
                 for call in sdk.writer.addParam.call_args_list}
        assert words[7] == (raw_gripper + 4376-canonical) & 0xffffffff
        assert sdk.registers[6, bus_module.OPERATING_MODE] == 3
        for invalid in (held[:5] + (4096, held[-1]), held[:-1] + (3179,), held[:-1] + (4377,)):
            with pytest.raises(ValueError):
                bus.write_positions(invalid)
        # While enabled, the offset is fixed even when the hardware crosses 0.
        sdk.positions[7] = (raw_gripper + 4096-canonical) & 0xffffffff
        assert bus.read_positions()[-1] == 4096
        # A sudden counter reset must fault, not normalize into another turn.
        sdk.positions[7] = (raw_gripper-canonical+100) & 0xffffffff
        with pytest.raises(RuntimeError, match='possible counter reset'):
            bus.read_positions()
    finally:
        bus.disarm()
        bus.close()


def test_wrapped_gripper_requires_mode4_without_changing_joint_modes(sdk):
    ids = seven_motors(sdk)
    bus = bus_module.DynamixelBus('/fake/port', 1000000, ids, gripper_wrap_limits=(3180, 4376))
    with pytest.raises(RuntimeError, match=r'Extended Position Control Mode \(4\)'):
        bus.arm(5, 12, command_rate_hz=30.0)
    assert not sdk.events
    bus.close()


def test_final_goal_readback_retries_dropped_sync_write_without_changing_target(sdk):
    ids = (1, 2, 3)
    bus = bus_module.DynamixelBus('/fake/port', 1000000, ids)
    frames = []
    def send():
        targets = {call.args[0]: int.from_bytes(bytes(call.args[1]), 'little')
                   for call in sdk.writer.addParam.call_args_list[-3:]}
        frames.append(targets)
        if len(frames) > 1:
            for mid, tick in targets.items():
                sdk.registers[mid, bus_module.GOAL_POSITION] = tick
        return 0
    sdk.writer.txPacket.side_effect = send
    bus.write_verified_positions((3819, 989, 2013))
    assert frames == [{1: 3819, 2: 989, 3: 2013}]*2
    sdk.writer.txPacket.side_effect = None
    sdk.registers[3, bus_module.GOAL_POSITION] = 0
    with pytest.raises(RuntimeError, match='Final position goals were not accepted'):
        bus.write_verified_positions((3819, 989, 2013))
    assert not sdk.events  # No torque, profile or EEPROM changes.
    bus.close()


def test_verified_wrapped_final_goal_reads_back_the_same_hardware_branch(sdk):
    ids = seven_motors(sdk)
    sdk.positions[7] = 100
    sdk.registers[7, bus_module.OPERATING_MODE] = 4
    bus = bus_module.DynamixelBus('/fake/port', 1000000, ids, gripper_wrap_limits=(3180, 4376))
    held = bus.arm(5, 12, command_rate_hz=30.0)
    def send():
        for call in sdk.writer.addParam.call_args_list[-7:]:
            sdk.registers[call.args[0], bus_module.GOAL_POSITION] = int.from_bytes(bytes(call.args[1]), 'little')
        return 0
    sdk.writer.txPacket.side_effect = send
    bus.write_verified_positions(held[:-1]+(4376,))
    assert sdk.registers[7, bus_module.GOAL_POSITION] == 280
    assert sdk.writer.txPacket.call_count == 1
    bus.disarm()
    bus.close()


@pytest.mark.parametrize('small_model', [1000, 1020])
def test_rewired_feedback_and_j4_goal_use_physical_motor_ids(sdk, small_model):
    model = ArmModel({'expected_model_numbers': [1120]*3 + [small_model]*4})
    assert model.ids == (1, 2, 3, 5, 4, 11, 12)
    sdk.positions.update({1: 3923, 2: 1061, 3: 2105, 4: 2004, 5: 2055, 11: 2111, 12: 3550})
    sdk.packet.ping.side_effect = lambda port, motor_id: (
        1120 if motor_id <= 3 else small_model, 0, 0)
    bus = bus_module.DynamixelBus('/fake/port', 1000000, model.ids)
    try:
        assert bus.models == model.models
        raw = bus.read_positions()
        assert raw == (3923, 1061, 2105, 2055, 2004, 2111, 3550)
        assert not sdk.events  # Startup and feedback do not write registers.
        bus.write_positions(model.encode(model.decode(raw)))
        before = {call.args[0]: int.from_bytes(bytes(call.args[1]), 'little')
                  for call in sdk.writer.addParam.call_args_list}
        sdk.writer.addParam.reset_mock()

        target = model.decode(raw)
        target[3] += 10 * RADIANS_PER_TICK
        bus.write_positions(model.encode(target))
        calls = sdk.writer.addParam.call_args_list
        assert [call.args[0] for call in calls] == [1, 2, 3, 5, 4, 11, 12]
        after = {call.args[0]: int.from_bytes(bytes(call.args[1]), 'little') for call in calls}
        assert {motor_id for motor_id in before if before[motor_id] != after[motor_id]} == {5}
        assert after[5] == before[5] + 10
    finally:
        bus.close()
