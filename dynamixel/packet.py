"""Dynamixel Protocol 1.0 / 2.0 packet encoding and decoding.

Packet layout (Protocol 2.0):
    [0xFF][0xFF][0xFD][0x00][ID][LEN_L][LEN_H][INST][PARAM...][CRC_L][CRC_H]
    |------- header -------|    |-- length --|        |-- payload --|

Packet layout (Protocol 1.0):
    [0xFF][0xFF][ID][LEN][INST][PARAM...][CHECKSUM]
"""

from struct import pack_into, unpack_from
from .crc import CRC_TABLE_V2

# Pre-built header bytes, shared by all TX packets (avoids per-packet allocation)
_HEADER_V2 = b"\xff\xff\xfd\x00"
_HEADER_V1 = b"\xff\xff"


class DynamixelPacket:
    """Base class for Dynamixel instruction / status packets.

    Provides property access to the ID, length, instruction, parameter,
    and checksum fields inside a flat ``bytearray`` buffer, abstracting
    the byte-offset differences between Protocol 1.0 and 2.0.
    """

    def __init__(self, buffer: bytearray, protocol_version: int = 2):
        if protocol_version not in (1, 2):
            raise ValueError("Unsupported protocol version.")

        self.buffer = buffer
        self.ver = protocol_version

        # Offset (in bytes) to the first byte *after* the length field.
        # Everything before this is the fixed header.
        #   v2: [Header(4)] [ID(1)] [Len(2)] → 7 bytes
        #   v1: [Header(2)] [ID(1)] [Len(1)] → 4 bytes
        self.LENGTH_BYTE = 7 if protocol_version == 2 else 4

    # ── Field accessors ──────────────────────────────────────────────

    @property
    def length(self) -> int:
        """Packet length field (little-endian u16 for v2, u8 for v1)."""
        if self.ver == 2:
            return unpack_from("<H", self.buffer, self.LENGTH_BYTE - 2)[0]
        return self.buffer[self.LENGTH_BYTE - 1]

    @length.setter
    def length(self, value: int):
        value &= 0xFFFF
        if self.ver == 2:
            pack_into("<H", self.buffer, self.LENGTH_BYTE - 2, value)
        else:
            self.buffer[self.LENGTH_BYTE - 1] = value

    @property
    def instruction(self) -> int:
        """Instruction byte (immediately follows the length field)."""
        return self.buffer[self.LENGTH_BYTE]

    @instruction.setter
    def instruction(self, value: int):
        self.buffer[self.LENGTH_BYTE] = value & 0xFF

    @property
    def id(self) -> int:
        """Motor ID byte."""
        if self.ver == 2:
            return self.buffer[self.LENGTH_BYTE - 3]
        return self.buffer[self.LENGTH_BYTE - 2]

    @id.setter
    def id(self, value: int):
        idx = (self.LENGTH_BYTE - 3) if self.ver == 2 else (self.LENGTH_BYTE - 2)
        self.buffer[idx] = value & 0xFF

    @property
    def param(self) -> bytearray:
        """Parameter bytes slice (excludes instruction and checksum)."""
        if self.ver == 2:
            # Offset from end: -(length - 1) for instruction, -2 for CRC
            return self.buffer[~self.length + 3:-2]
        return self.buffer[~self.length + 2:-1]

    @param.setter
    def param(self, value: bytearray):
        n = len(value)
        if self.ver == 2:
            self.buffer[-(n + 2):-2] = value
        else:
            self.buffer[-(n + 1):-1] = value

    @property
    def checksum(self) -> int:
        """Trailing checksum (CRC-16 for v2, inverted byte-sum for v1)."""
        if self.ver == 2:
            return unpack_from("<H", self.buffer, -2)[0]
        return self.buffer[-1]

    @checksum.setter
    def checksum(self, value: int):
        if self.ver == 2:
            pack_into("<H", self.buffer, -2, value)
        else:
            self.buffer[-1] = value

    # ── Checksum calculation ─────────────────────────────────────────

    def calc_checksum(self) -> int:
        """Compute the expected checksum over the current buffer contents.

        Protocol 2.0 uses CRC-16 (Maxim/Robotis variant) over all bytes
        except the trailing two CRC bytes.
        Protocol 1.0 uses the bitwise NOT of the low byte of the sum
        from the ID field to the last parameter byte.
        """
        if self.ver == 2:
            crc = 0x0000
            crc_table = CRC_TABLE_V2           # local ref for speed
            buf = memoryview(self.buffer)[:-2]  # avoid copying the slice
            for byte in buf:
                crc = crc_table[((crc >> 8) ^ byte) & 0xFF] ^ ((crc << 8) & 0xFFFF)
            return crc
        # v1: sum bytes from ID through last param (skip header, skip checksum)
        return ~sum(self.buffer[self.LENGTH_BYTE - 2:-1]) & 0xFF

    # ── Dunder helpers ───────────────────────────────────────────────

    def __repr__(self):
        return f"DynamixelPacket(ver={self.ver}, buffer={self.buffer})"

    def __len__(self):
        return len(self.buffer)


class DynamixelRXPacket(DynamixelPacket):
    """Received status / ACK packet, built incrementally from UART reads."""

    def __init__(self, protocol_version: int = 2):
        super().__init__(bytearray(), protocol_version)

    @property
    def error(self) -> int:
        """Error field from the status packet (0 = no error)."""
        return self.buffer[8 if self.ver == 2 else 4]

    def has_valid_checksum(self) -> bool:
        """Return ``True`` if the received checksum matches the computed one."""
        return self.checksum == self.calc_checksum()


class DynamixelTXPacket(DynamixelPacket):
    """Outgoing instruction packet, ready to write to the UART bus.

    The constructor fills in the header, ID, length, instruction, and
    parameter fields, then computes and appends the checksum so that
    ``self.buffer`` is immediately ready for ``uart.write()``.
    """

    def __init__(self, id: int, instruction: int, length: int,
                 param: bytearray, protocol_version: int = 2):
        if protocol_version == 2:
            # Total size: 4 (header) + 1 (ID) + 2 (length field) + length
            buffer = bytearray(7 + length)
            buffer[0:4] = _HEADER_V2
        else:
            # Total size: 2 (header) + 1 (ID) + 1 (length field) + length
            buffer = bytearray(4 + length)
            buffer[0:2] = _HEADER_V1

        super().__init__(buffer, protocol_version)
        self.id = id
        self.length = length
        self.instruction = instruction
        if param:
            self.param = param
        self.checksum = self.calc_checksum()
