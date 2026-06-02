"""Dynamixel servo motor driver for MicroPython.

Provides a high-level property-based interface for communicating with
Dynamixel actuators over UART using Protocol 1.0 or 2.0 packet framing.
"""

from machine import UART
from .table import get_control_table, ControlTableItem
from .packet import DynamixelTXPacket, DynamixelRXPacket
import time
from struct import pack_into, unpack_from
from .model import DynamixelModel

# ── Module-level constants (MicroPython const() replaces name lookups
#    with literal values at compile time, saving RAM and CPU cycles) ──────
try:
    from micropython import const
except ImportError:  # allow running on CPython for testing
    const = lambda x: x  # noqa: E731

_INS_PING = const(1)
_INS_READ = const(2)
_INS_WRITE = const(3)
_INS_ACK = const(85)
_INS_FACTORY_RESET = const(6)
_TIMEOUT_US = const(5000)  # response timeout in microseconds

# Struct format character indexed by data byte-length
_FMT = {1: "B", 2: "H", 4: "I"}

# Pre-allocated reusable buffers – avoids repeated heap allocation
# in the hot read path (safe because MicroPython is single-threaded).
_READ_PARAM_BUF = bytearray(4)
_PING_PARAM = b"\x01"
_RESET_PARAM = b"\xff"


class TimeoutError(Exception):
    """Raised when a Dynamixel motor does not respond within the timeout."""

    pass


class Dynamixel:
    """High-level driver for a single Dynamixel servo motor.

    Wraps UART communication with Dynamixel Protocol 1.0 / 2.0 packet
    framing, exposing motor registers (position, velocity, PWM, operating
    mode, etc.) as Python properties.
    """

    def __init__(self, connector: UART, model: int = DynamixelModel.UNKNOWN, id: int = 1, protocol_version: int = 2):
        """
        Args:
            connector: UART interface connected to the Dynamixel bus.
            model: Motor model number; loads the control table immediately
                   when not ``DynamixelModel.UNKNOWN``.
            id: Motor ID on the bus (1–253, or 254 for broadcast).
            protocol_version: Dynamixel protocol version (1 or 2).
        """
        self._connector = connector
        self.model = model
        self._control_table = get_control_table(model) if model != DynamixelModel.UNKNOWN else {}
        self.id = id
        self.ver = protocol_version

    # ── Utility helpers ──────────────────────────────────────────────

    @staticmethod
    def _to_rel(value, lo, hi):
        """Normalise *value* to the 0.0–1.0 range within [lo, hi]."""
        return (value - lo) / (hi - lo)

    @staticmethod
    def _to_abs(value, lo, hi):
        """Convert a 0.0–1.0 relative value back to the [lo, hi] range."""
        return value * (hi - lo) + lo

    @staticmethod
    def _to_signed(n_bytes, value):
        """Interpret *value* as a two's-complement signed integer."""
        bits = n_bytes << 3
        if value >= (1 << (bits - 1)):
            value -= 1 << bits
        return value

    # ── Low-level register helpers ───────────────────────────────────

    def _get_addr_length(self, item: int):
        """Return ``(address, byte_length)`` for a control-table *item*."""
        entry = self._control_table.get(item)
        if entry is None:
            raise ValueError("Control table item not found.", item)
        addr, length = entry.get("addr"), entry.get("length")
        if addr is None or length is None:
            raise ValueError("Control table item missing address or length.", item)
        return addr, length

    @staticmethod
    def _length_to_fmt(length: int):
        """Return the struct format character for a 1/2/4-byte register."""
        fmt = _FMT.get(length)
        if fmt is None:
            raise ValueError("Unsupported data length.", length)
        return fmt

    # ── UART I/O primitives ──────────────────────────────────────────

    def _clear_buffer(self):
        """Drain any stale bytes from the UART receive buffer."""
        conn = self._connector
        while conn.any():
            conn.read()

    def _read_buffer(self, log_errors: bool = True) -> DynamixelRXPacket | None:
        """Block until a complete response packet arrives or the timeout expires.

        Returns:
            The parsed RX packet, or ``None`` on timeout / checksum failure.
        """
        try:
            rx = DynamixelRXPacket(self.ver)
            length_to_read = rx.LENGTH_BYTE
            conn = self._connector
            ticks_us = time.ticks_us
            ticks_diff = time.ticks_diff

            # Phase 1 – wait for the fixed-length header bytes
            t_start = ticks_us()
            while conn.any() < length_to_read:
                if ticks_diff(ticks_us(), t_start) > _TIMEOUT_US:
                    raise TimeoutError("No response from the Dynamixel motor.")

            data = conn.read(length_to_read)
            if data is not None:
                rx.buffer.extend(data)

            # Phase 2 – wait for the remaining payload + checksum bytes
            length_to_read = rx.length
            t_start = ticks_us()
            while conn.any() < length_to_read:
                if ticks_diff(ticks_us(), t_start) > _TIMEOUT_US:
                    raise TimeoutError("Incomplete response from the Dynamixel motor.")

            data = conn.read(length_to_read)
            if data is not None:
                rx.buffer.extend(data)

            if not rx.has_valid_checksum():
                raise ValueError("Invalid checksum in response packet.")
            return rx
        except TimeoutError as e:
            if log_errors:
                print(f"Timeout error: {e}")
            return None
        except ValueError as e:
            if log_errors:
                print(f"Value error: {e}")
            return None
        except Exception as e:
            if log_errors:
                print(f"Unexpected error in _read_buffer: {e}")
            return None

    def _send_packet(self, instruction: int, param):
        """Build a TX packet from *instruction* + *param* and write it to the bus."""
        ver = self.ver
        # Length field = instruction (1) + param + checksum (2 for v2, 1 for v1)
        length = len(param) + (3 if ver == 2 else 2)

        msg = DynamixelTXPacket(
            id=self.id,
            instruction=instruction,
            length=length,
            param=param,
            protocol_version=ver,
        )

        conn = self._connector
        conn.write(msg.buffer)
        conn.flush()

    # ── Register read / write ────────────────────────────────────────

    def _read_raw(self, item: int):
        """Send a read instruction and return the raw parameter bytes (or ``None``)."""
        self._clear_buffer()
        self._send_read_instruction(item)
        rx = self._read_buffer()
        return rx.param if rx is not None else None

    def _read_value(self, item: int, retries: int = 3):
        """Read a numeric register, retrying up to *retries* times.

        Returns 0 if all attempts fail.
        """
        _, length = self._get_addr_length(item)
        fmt = "<" + self._length_to_fmt(length)
        for _ in range(retries):
            param = self._read_raw(item)
            if param is not None and len(param) > 0:
                return unpack_from(fmt, param, 0)[0]
        raise TimeoutError("Failed to read register after retries.")

    def _send_read_instruction(self, item: int):
        """Transmit a READ instruction for the given control-table *item*.

        Re-uses a module-level buffer to avoid per-call allocation.
        """
        address, length = self._get_addr_length(item)
        pack_into("<H", _READ_PARAM_BUF, 0, address)
        pack_into("<H", _READ_PARAM_BUF, 2, length)
        self._send_packet(_INS_READ, _READ_PARAM_BUF)

    def _write_register(self, item: int, data):
        """Write *data* to the given control-table register and wait for ACK.

        Logs an error if the ACK is missing or has an unexpected instruction.
        """
        try:
            address, length = self._get_addr_length(item)
            fmt = "<" + self._length_to_fmt(length)
            param = bytearray(length + 2)
            pack_into("<H", param, 0, address)
            pack_into(fmt, param, 2, data)

            self._clear_buffer()
            self._send_packet(_INS_WRITE, param)

            # Read the status/ACK packet so the bus is clear for the next command
            ack = self._read_buffer()
            if ack is None:
                raise ValueError("No ACK received after write instruction.")
            if ack.instruction != _INS_ACK:
                raise ValueError(f"Unexpected instruction in ACK: {ack.instruction}")
        except Exception as e:
            print(f"Error in _write_register: {e}")

    # ── High-level commands ──────────────────────────────────────────

    def ping(self, baudrate: int, id: int = 254):
        """Scan for a motor at *baudrate*.

        On success, loads the appropriate control table and stores the
        discovered motor ID.  Returns ``True`` / ``False``.
        """
        self._connector.init(baudrate=baudrate, bits=8, parity=None, stop=1)
        self.id = id
        self._send_packet(_INS_PING, _PING_PARAM)

        rx = self._read_buffer(log_errors=False)
        if rx is None:
            return False

        # First two param bytes = model number (little-endian uint16)
        model = unpack_from("<H", rx.param, 0)[0]
        self.model = model
        self._control_table = get_control_table(model)
        self.id = rx.id
        return True

    def reset(self):
        """Issue a factory-reset instruction and wait for the motor to reboot."""
        self._send_packet(_INS_FACTORY_RESET, _RESET_PARAM)
        time.sleep(0.1)
        self._clear_buffer()

    # ── Properties: configuration registers ──────────────────────────

    @property
    def led(self) -> bool:
        """LED on / off state."""
        return bool(self._read_value(ControlTableItem.DXL_LED))

    @led.setter
    def led(self, value: bool):
        self._write_register(ControlTableItem.DXL_LED, int(value))

    @property
    def operating_mode(self) -> int:
        """Current operating mode (position, velocity, PWM, extended-position, etc.)."""
        if self.model == DynamixelModel.XL320:
            return self._read_value(ControlTableItem.CONTROL_MODE)
        return self._read_value(ControlTableItem.OPERATING_MODE)

    @operating_mode.setter
    def operating_mode(self, value: int):
        if self.model == DynamixelModel.XL320:
            self._write_register(ControlTableItem.CONTROL_MODE, value)
        else:
            self._write_register(ControlTableItem.OPERATING_MODE, value)

    @property
    def torque_enabled(self) -> bool:
        """Whether the motor output is energised."""
        return bool(self._read_value(ControlTableItem.TORQUE_ENABLE))

    @torque_enabled.setter
    def torque_enabled(self, value: bool):
        self._write_register(ControlTableItem.TORQUE_ENABLE, 1 if value else 0)

    # ── Properties: limits ───────────────────────────────────────────

    @property
    def min_voltage_limit(self) -> int:
        """Minimum allowed input voltage."""
        return self._read_value(ControlTableItem.MIN_VOLTAGE_LIMIT)

    @min_voltage_limit.setter
    def min_voltage_limit(self, value: int):
        self._write_register(ControlTableItem.MIN_VOLTAGE_LIMIT, value)

    @property
    def max_voltage_limit(self) -> int:
        """Maximum allowed input voltage."""
        return self._read_value(ControlTableItem.MAX_VOLTAGE_LIMIT)

    @max_voltage_limit.setter
    def max_voltage_limit(self, value: int):
        self._write_register(ControlTableItem.MAX_VOLTAGE_LIMIT, value)

    @property
    def position_limit_low(self) -> int:
        """Minimum allowed position (raw encoder ticks)."""
        if self.model == DynamixelModel.XL320:
            return self._read_value(ControlTableItem.CW_ANGLE_LIMIT)
        return self._read_value(ControlTableItem.MIN_POSITION_LIMIT)

    @position_limit_low.setter
    def position_limit_low(self, value: int):
        if self.model == DynamixelModel.XL320:
            self._write_register(ControlTableItem.CW_ANGLE_LIMIT, value)
        else:
            self._write_register(ControlTableItem.MIN_POSITION_LIMIT, value)

    @property
    def position_limit_high(self) -> int:
        """Maximum allowed position (raw encoder ticks)."""
        if self.model == DynamixelModel.XL320:
            return self._read_value(ControlTableItem.CCW_ANGLE_LIMIT)
        return self._read_value(ControlTableItem.MAX_POSITION_LIMIT)

    @position_limit_high.setter
    def position_limit_high(self, value: int):
        if self.model == DynamixelModel.XL320:
            self._write_register(ControlTableItem.CCW_ANGLE_LIMIT, value)
        else:
            self._write_register(ControlTableItem.MAX_POSITION_LIMIT, value)

    @property
    def velocity_limit(self) -> int:
        """Maximum allowed velocity (raw units)."""
        if self.model == DynamixelModel.XL320:
            return 1023  # fixed limit for XL-320 in velocity mode
        else:
            return self._read_value(ControlTableItem.VELOCITY_LIMIT)

    @velocity_limit.setter
    def velocity_limit(self, value: int):
        if self.model != DynamixelModel.XL320:
            self._write_register(ControlTableItem.VELOCITY_LIMIT, value)

    @property
    def acceleration_limit(self) -> int:
        """Maximum allowed acceleration (raw units)."""
        return self._read_value(ControlTableItem.ACCELERATION_LIMIT)

    @acceleration_limit.setter
    def acceleration_limit(self, value: int):
        self._write_register(ControlTableItem.ACCELERATION_LIMIT, value)

    @property
    def pwm_limit(self) -> int:
        """Maximum allowed PWM value."""
        return self._read_value(ControlTableItem.PWM_LIMIT)

    @pwm_limit.setter
    def pwm_limit(self, value: int):
        self._write_register(ControlTableItem.PWM_LIMIT, value)

    @property
    def current_limit(self) -> int:
        """Maximum allowed current draw."""
        return self._read_value(ControlTableItem.CURRENT_LIMIT)

    @current_limit.setter
    def current_limit(self, value: int):
        self._write_register(ControlTableItem.CURRENT_LIMIT, value)

    # ── Properties: present state (read-only) ────────────────────────

    @property
    def present_position(self) -> int:
        """Present position in raw encoder ticks (signed)."""
        if self.model == DynamixelModel.XL320:
            return self._to_signed(2, self._read_value(ControlTableItem.PRESENT_POSITION))
        return self._to_signed(4, self._read_value(ControlTableItem.PRESENT_POSITION))

    @property
    def present_position_rel(self) -> float:
        """Present position normalised to 0.0–1.0 within the position limits."""
        return self._to_rel(self.present_position, self.position_limit_low, self.position_limit_high)

    @property
    def present_velocity(self) -> int:
        """Present velocity in raw units (signed)."""
        return self._to_signed(4, self._read_value(ControlTableItem.PRESENT_VELOCITY))

    @property
    def present_velocity_rel(self) -> float:
        """Present velocity normalised to 0.0–1.0 within the velocity limit."""
        return self._to_rel(self.present_velocity, 0, self.velocity_limit)

    @property
    def present_pwm(self) -> int:
        """Present PWM value (signed)."""
        if self.model == DynamixelModel.XL320:
            tmp = self._to_signed(2, self._read_value(ControlTableItem.PRESENT_SPEED))
            if tmp > 1023:
                tmp = -(tmp - 1024)
            return tmp
        return self._to_signed(2, self._read_value(ControlTableItem.PRESENT_PWM))

    @property
    def present_current(self) -> int:
        """Present current draw (signed)."""
        return self._to_signed(2, self._read_value(ControlTableItem.PRESENT_CURRENT))

    # ── Properties: goal registers ───────────────────────────────────

    @property
    def goal_position(self) -> int:
        """Goal position in raw encoder ticks (signed)."""
        if self.model == DynamixelModel.XL320:
            return self._to_signed(2, self._read_value(ControlTableItem.GOAL_POSITION))
        return self._to_signed(4, self._read_value(ControlTableItem.GOAL_POSITION))

    @goal_position.setter
    def goal_position(self, value: int):
        self._write_register(ControlTableItem.GOAL_POSITION, value)

    @property
    def goal_position_rel(self) -> float:
        """Goal position normalised within the position limits."""
        return self._to_rel(self.goal_position, self.position_limit_low, self.position_limit_high)

    @goal_position_rel.setter
    def goal_position_rel(self, value: float):
        self.goal_position = int(round(self._to_abs(value, self.position_limit_low, self.position_limit_high)))

    @property
    def goal_extend_position(self) -> int:
        """Goal position for extended-position mode (signed, full multi-turn range)."""
        return self._to_signed(4, self._read_value(ControlTableItem.GOAL_POSITION))

    @goal_extend_position.setter
    def goal_extend_position(self, value: int):
        self._write_register(ControlTableItem.GOAL_POSITION, value)

    @property
    def goal_extend_position_rel(self) -> float:
        """Goal position normalised within the extended ±1 048 575 range."""
        return self._to_rel(self.goal_position, -1_048_575, 1_048_575)

    @goal_extend_position_rel.setter
    def goal_extend_position_rel(self, value: float):
        self.goal_extend_position = int(round(self._to_abs(value, -1_048_575, 1_048_575)))

    @property
    def goal_velocity(self) -> int:
        """Goal velocity in raw units (signed)."""
        return self._to_signed(4, self._read_value(ControlTableItem.GOAL_VELOCITY))

    @goal_velocity.setter
    def goal_velocity(self, value: int):
        self._write_register(ControlTableItem.GOAL_VELOCITY, value)

    @property
    def goal_velocity_rel(self) -> float:
        """Goal velocity normalised to 0.0–1.0 within the velocity limit."""
        return self._to_rel(self.goal_velocity, 0, self.velocity_limit)

    @goal_velocity_rel.setter
    def goal_velocity_rel(self, value: float):
        self.goal_velocity = int(round(self._to_abs(value, 0, self.velocity_limit)))

    @property
    def goal_acceleration(self) -> int:
        """Goal acceleration in raw units."""
        return self._read_value(ControlTableItem.GOAL_ACCELERATION)

    @goal_acceleration.setter
    def goal_acceleration(self, value: int):
        self._write_register(ControlTableItem.GOAL_ACCELERATION, value)

    @property
    def goal_acceleration_rel(self) -> float:
        """Goal acceleration normalised to 0.0–1.0 within the acceleration limit."""
        return self._to_rel(self.goal_acceleration, 0, self.acceleration_limit)

    @goal_acceleration_rel.setter
    def goal_acceleration_rel(self, value: float):
        self.goal_acceleration = int(round(self._to_abs(value, 0, self.acceleration_limit)))

    @property
    def goal_pwm(self) -> int:
        """Goal PWM value (signed)."""
        if self.model == DynamixelModel.XL320:
            tmp = self._to_signed(2, self._read_value(ControlTableItem.MOVING_SPEED))
            if tmp > 1023:
                tmp = -(tmp - 1024)
            return tmp
        return self._to_signed(2, self._read_value(ControlTableItem.GOAL_PWM))

    @goal_pwm.setter
    def goal_pwm(self, value: int):
        if self.model == DynamixelModel.XL320:
            if value < 0:
                value = -value + 1024  # two's complement encoding for reverse direction
            self._write_register(ControlTableItem.MOVING_SPEED, value)
        else:
            self._write_register(ControlTableItem.GOAL_PWM, value)

    @property
    def goal_pwm_rel(self) -> float:
        """Goal PWM normalised within ±pwm_limit."""
        return self._to_rel(self.goal_pwm, -self.pwm_limit, self.pwm_limit)

    @goal_pwm_rel.setter
    def goal_pwm_rel(self, value: float):
        self.goal_pwm = int(round(self._to_abs(value, -self.pwm_limit, self.pwm_limit)))

    @property
    def profile_velocity(self) -> int:
        """Profile velocity used for trajectory generation."""
        return self._read_value(ControlTableItem.PROFILE_VELOCITY)

    @profile_velocity.setter
    def profile_velocity(self, value: int):
        self._write_register(ControlTableItem.PROFILE_VELOCITY, value)
