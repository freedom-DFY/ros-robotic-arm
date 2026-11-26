"""DYNAMIXEL Protocol 2.0 synchronized I/O for six joints and a gripper."""

from time import sleep

from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler

DRIVE_MODE = 10
TIME_BASED_PROFILE = 0x04
TORQUE_ON_BY_GOAL_UPDATE = 0x08
OPERATING_MODE = 11
HOMING_OFFSET = 20
MAX_POSITION_LIMIT = 48
MIN_POSITION_LIMIT = 52
TORQUE_ENABLE = 64
HARDWARE_ERROR = 70
BUS_WATCHDOG = 98
PROFILE_ACCELERATION = 108
PROFILE_VELOCITY = 112
GOAL_POSITION = 116
PRESENT_POSITION = 132


class DynamixelBus:
    def __init__(self, device, baud, ids):
        self.ids = ids
        self.possibly_enabled = set()
        self.position_limits = None
        self.port = PortHandler(device)
        self.packet = PacketHandler(2.0)
        self.reader = GroupSyncRead(self.port, self.packet, PRESENT_POSITION, 4)
        self.writer = GroupSyncWrite(self.port, self.packet, GOAL_POSITION, 4)
        try:
            if not self.port.openPort() or not self.port.setBaudRate(baud):
                raise RuntimeError("Cannot open {} at {} bps".format(device, baud))
            models = []
            for motor_id in ids:
                model, result, error = self.packet.ping(self.port, motor_id)
                self._check(motor_id, result, error)
                models.append(model)
                if not self.reader.addParam(motor_id):
                    raise RuntimeError("Failed to add motor {} to sync read".format(motor_id))
            self.models = tuple(models)
        except Exception:
            self.port.closePort()
            raise

    def _check(self, motor_id, result, error=0):
        if result != COMM_SUCCESS:
            raise RuntimeError("DXL {}: {}".format(motor_id, self.packet.getTxRxResult(result)))
        if error:
            raise RuntimeError("DXL {}: {}".format(motor_id, self.packet.getRxPacketError(error)))

    def _read(self, motor_id, address, size=1):
        read = self.packet.read1ByteTxRx if size == 1 else self.packet.read4ByteTxRx
        value, result, error = read(self.port, motor_id, address)
        self._check(motor_id, result, error)
        return value

    def _write(self, motor_id, address, value, size=1):
        write = self.packet.write1ByteTxRx if size == 1 else self.packet.write4ByteTxRx
        result, error = write(self.port, motor_id, address, value)
        self._check(motor_id, result, error)

    def read_positions(self):
        self._check("sync read", self.reader.txRxPacket())
        positions = []
        for motor_id in self.ids:
            if not self.reader.isAvailable(motor_id, PRESENT_POSITION, 4):
                raise RuntimeError("Missing position for motor {}".format(motor_id))
            raw = self.reader.getData(motor_id, PRESENT_POSITION, 4)
            positions.append(raw if raw < 2**31 else raw - 2**32)
        return tuple(positions)

    def _verify_hold_position(self, motor_id, tick):
        # Re-read the same goal before torque-on. Do not accept a different
        # target or rewrite it based on a mismatching response.
        observed = []
        for attempt in range(3):
            if attempt:
                sleep(0.01)
            actual = self._read(motor_id, GOAL_POSITION, 4)
            observed.append(actual)
            if actual == tick:
                return

        diagnostics = []
        for name, address, size in (
            ('present_position', PRESENT_POSITION, 4),
            ('torque_enable', TORQUE_ENABLE, 1),
            ('operating_mode', OPERATING_MODE, 1),
            ('drive_mode', DRIVE_MODE, 1),
            ('hardware_error', HARDWARE_ERROR, 1),
            ('bus_watchdog', BUS_WATCHDOG, 1),
        ):
            try:
                value = self._read(motor_id, address, size)
                if address == PRESENT_POSITION and value >= 2**31:
                    value -= 2**32
                diagnostics.append('{}={}'.format(name, value))
            except Exception as exc:
                diagnostics.append('{}=unavailable ({})'.format(name, exc))
        raise RuntimeError(
            'Motor {} did not accept the hold position: requested={}, goal_readbacks={}; {}. '
            'Torque-on was not attempted by this enable request'
            .format(motor_id, tick, observed, ', '.join(diagnostics))
        )

    def arm(self, acceleration, velocity, *, command_rate_hz):
        """Enable torque at the measured position, independently of software motion bounds."""
        if self.possibly_enabled:
            raise RuntimeError("Disable the previous arm session first")
        # Each streamed position step gets one command period in time-based mode.
        duration_ms = int(1000 / command_rate_hz)
        time_profile = (duration_ms // 4, duration_ms)
        profiles = []
        limits = []
        for motor_id in self.ids:
            if self._read(motor_id, TORQUE_ENABLE):
                raise RuntimeError("Motor {} torque is already on".format(motor_id))
            if self._read(motor_id, OPERATING_MODE) != 3:
                raise RuntimeError("Motor {} needs Position Control Mode (3)".format(motor_id))
            drive_mode = self._read(motor_id, DRIVE_MODE)
            if drive_mode & TORQUE_ON_BY_GOAL_UPDATE:
                raise RuntimeError("Motor {} needs Torque On by Goal Update off".format(motor_id))
            profiles.append(time_profile if drive_mode & TIME_BASED_PROFILE else (acceleration, velocity))
            if self._read(motor_id, HOMING_OFFSET, 4) != 0:
                raise RuntimeError("Motor {} needs Homing Offset 0; use software zero_ticks"
                                   .format(motor_id))
            motor_min = self._read(motor_id, MIN_POSITION_LIMIT, 4)
            motor_max = self._read(motor_id, MAX_POSITION_LIMIT, 4)
            if not 0 <= motor_min <= motor_max <= 4095:
                raise RuntimeError("Motor {} reports invalid internal position limits".format(motor_id))
            limits.append((motor_min, motor_max))
            if self._read(motor_id, HARDWARE_ERROR):
                raise RuntimeError("Motor {} reports a hardware error".format(motor_id))

        # Position can be signed/multiturn while torque is off. Mode 3 resets to
        # the absolute single-turn reading when enabled, with zero Homing Offset.
        present = tuple(tick % 4096 for tick in self.read_positions())
        # A hold goal must be accepted unchanged by the servo, without clipping
        # against its own register limits. Application motion bounds are checked
        # only when a motion is requested.
        for motor_id, tick, (lo, hi) in zip(self.ids, present, limits):
            if not lo <= tick <= hi:
                raise RuntimeError("Motor {} current position {} is outside the servo's internal limits {}..{}"
                                   .format(motor_id, tick, lo, hi))
        self.position_limits = tuple(limits)
        try:
            for motor_id, tick, (accel, speed_or_time) in zip(self.ids, present, profiles):
                self._write(motor_id, PROFILE_ACCELERATION, accel, 4)
                self._write(motor_id, PROFILE_VELOCITY, speed_or_time, 4)
                self._write(motor_id, GOAL_POSITION, tick, 4)
                self._verify_hold_position(motor_id, tick)
            for motor_id in self.ids:
                self.possibly_enabled.add(motor_id)
                self._write(motor_id, TORQUE_ENABLE, 1)
                if self._read(motor_id, TORQUE_ENABLE) != 1:
                    raise RuntimeError("Motor {} torque-on could not be confirmed".format(motor_id))
            return present
        except Exception as exc:
            failures = self.disarm()
            if failures:
                raise RuntimeError("{}; torque-off could not be confirmed: {}"
                                   .format(exc, "; ".join(failures))) from exc
            raise

    def write_positions(self, ticks):
        if (len(ticks) != len(self.ids)
                or any(type(tick) is not int or not 0 <= tick <= 4095 for tick in ticks)):
            raise ValueError('Expected one single-turn integer target for every motor')
        try:
            for motor_id, tick in zip(self.ids, ticks):
                if not self.writer.addParam(motor_id, list(tick.to_bytes(4, "little"))):
                    raise RuntimeError("Failed to add motor {} to sync write".format(motor_id))
            self._check("sync write", self.writer.txPacket())
        finally:
            self.writer.clearParam()

    def read_hardware_errors(self):
        """Read error flags in the configured ID order without changing registers."""
        return tuple(self._read(motor_id, HARDWARE_ERROR) for motor_id in self.ids)

    def disarm(self):
        failures = []
        for motor_id in sorted(self.possibly_enabled):
            try:
                self._write(motor_id, TORQUE_ENABLE, 0)
                if self._read(motor_id, TORQUE_ENABLE):
                    raise RuntimeError("Torque off not confirmed for motor {}".format(motor_id))
                self.possibly_enabled.remove(motor_id)
            except Exception as exc:
                failures.append(str(exc))
        return failures

    def close(self):
        self.port.closePort()
