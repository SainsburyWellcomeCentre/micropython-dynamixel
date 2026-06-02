"""MicroPython driver for Dynamixel smart servos (Protocol 1.0 & 2.0).

Eagerly imports the two most-used classes; everything else is loaded on
first access via ``__getattr__`` to save RAM on constrained boards.
"""

from ..dynamixel import Dynamixel
from ..model import DynamixelModel

# Lazy-loaded symbols — mapped to their submodule name.
_attrs = {
    "DynamixelPacket": "packet",
    "DynamixelRXPacket": "packet",
    "DynamixelTXPacket": "packet",
    "ControlTableItem": "table",
    "get_control_table": "table",
    "CRC_TABLE_V2": "crc",
}


def __getattr__(attr):
    """Import secondary symbols on first access to reduce startup memory."""
    if attr in _attrs:
        module_name = _attrs[attr]
        module = __import__(f"dynamixel.{module_name}", fromlist=[attr])
        return getattr(module, attr)
    raise AttributeError(f"module 'dynamixel' has no attribute '{attr}'")
