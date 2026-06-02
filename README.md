# micropython-dynamixel

MicroPython driver for [Dynamixel](https://www.robotis.com/en/dynamixel.php) servo motors.  Supports **Protocol 1.0** and **Protocol 2.0** over UART.

## Features

- Property-based API for reading/writing motor registers (position, velocity,
  PWM, current, LED, operating mode, etc.).
- Automatic control-table selection via model number returned by `ping()`.
- Covers AX, DX, RX, EX, MX, XL, XC, XM, XH, XD, XW, PRO, PRO+, and Y series
  motors.

## Installation

### Via `mip` (network-capable boards)

```python
import mip
mip.install("github:org/micropython-dynamixel")
```

### Via `mpremote` (from your PC)

```bash
mpremote mip install github:org/micropython-dynamixel
```

> Replace `org/micropython-dynamixel` with the actual GitHub path of this
> repository.

### Manual

Copy the `dynamixel/` folder to the `lib/` directory on your device.

## Quick Start

```python
from machine import UART
from dynamixel import Dynamixel, DynamixelModel

# Initialise UART (pins depend on your board)
uart = UART(0, baudrate=57600, tx=0, rx=1)

# Create a driver instance (Protocol 2.0)
dxl = Dynamixel(uart, protocol_version=2)

# Scan for a motor at 57600 baud
if dxl.ping(baudrate=57600):
    print("Motor found!")

    # Enable torque and move to a position
    dxl.torque_enabled = True
    dxl.goal_position = 2048

    # Read present position
    print("Position:", dxl.current_position)
```

## API Overview

| Property / Method          | Description                              |
| -------------------------- | ---------------------------------------- |
| `ping(baudrate, id=254)`   | Scan for a motor; loads control table    |
| `reset()`                  | Factory-reset the motor                  |
| `torque_enabled`           | Enable / disable motor output            |
| `led`                      | LED on / off                             |
| `operating_mode`           | Operating mode (position, velocity, …)   |
| `goal_position`            | Goal position (raw ticks)                |
| `goal_position_rel`        | Goal position normalised to 0.0 – 1.0   |
| `goal_velocity`            | Goal velocity (raw units, signed)        |
| `goal_pwm`                 | Goal PWM (signed)                        |
| `current_position`         | Present position (signed)                |
| `current_velocity`         | Present velocity (signed)                |
| `profile_velocity`         | Profile velocity for trajectory gen.     |
| `position_limit_low/high`  | Position limits                          |
| `velocity_limit`           | Velocity limit                           |

See the source for the complete list of properties.

## Supported Models

AX-12A · AX-12W · AX-18A · DX-113 · DX-116 · DX-117 · RX-10 · RX-24F ·
RX-28 · RX-64 · EX-106 · MX-12W · MX-28 · MX-64 · MX-106 · MX-28(2.0) ·
MX-64(2.0) · MX-106(2.0) · XL-320 · XL-330 · XC-330 · XC-430 · XL-430 ·
XM-430 · XH-430 · XD-430 · XM-540 · XH-540 · XD-540 · XW-430 · XW-540 ·
PRO series · PRO+ series · Y series

## License

