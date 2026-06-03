# Example: Scan for a Dynamixel motor and drive it in position control mode.
# Tested on Raspberry Pi Pico with Protocol 2.0 servos (e.g. XL430-W250).

from machine import UART
from dynamixel import Dynamixel, DynamixelModel
import time

# ---------------------------------------------------------------------------
# Configuration — adjust these to match your wiring and servo setup
# ---------------------------------------------------------------------------
TX_PIN = 0            # GPIO pin connected to the half-duplex data line (TX)
RX_PIN = 1            # GPIO pin connected to the half-duplex data line (RX)
UART_ID = 0           # RP2040 UART peripheral index (0 or 1)
PROTOCOL_VERSION = 2  # 1 for legacy AX/RX servos, 2 for X/MX-series servos
# ---------------------------------------------------------------------------

# Initialise the UART peripheral.  The driver will reconfigure the baudrate
# automatically during the scan below, so the initial value does not matter.
uart = UART(UART_ID, baudrate=57600, tx=TX_PIN, rx=RX_PIN)

# Create the Dynamixel driver bound to the UART interface.
dxl = Dynamixel(uart, protocol_version=PROTOCOL_VERSION)

# Baudrates to scan in order from slowest to fastest.
# The first baudrate at which a servo responds is used for all subsequent
# communication.  Add or remove entries to match your network configuration.
BAUD = [9600, 57600, 115200, 1_000_000, 2_000_000, 3_000_000, 4_000_000, 4_500_000]

for baud in BAUD:
    if dxl.ping(baudrate=baud):
        print(f"Motor found at baudrate {baud}!")
        print("Model:", DynamixelModel.get_name(dxl.model))
        break

if not dxl.model:
    print("No motor found — check wiring, power, and ID/baudrate settings.")
    while True:
        pass  # Halt execution so the error message remains visible

# Configure the servo for position control.
# Torque must be disabled before changing the operating mode.
dxl.torque_enabled = False
dxl.operating_mode = 3  # 3 = Position Control Mode (single-turn)
dxl.torque_enabled = True

# Repeatedly increment the goal position by half a revolution (2048 steps)
# and print the servo's reported position after each move.
while True:
    dxl.goal_position += 2048
    print("Current Position:", dxl.current_position)
    time.sleep(1)