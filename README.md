# micropython-dynamixel

[![GitHub release](https://img.shields.io/github/v/release/SainsburyWellcomeCentre/micropython-dynamixel?style=flat-square&cacheSeconds=3600)](https://github.com/SainsburyWellcomeCentre/micropython-dynamixel/releases)
[![GitHub issues](https://img.shields.io/github/issues/SainsburyWellcomeCentre/micropython-dynamixel?style=flat-square)](https://github.com/SainsburyWellcomeCentre/micropython-dynamixel/issues)

MicroPython driver for [Dynamixel](https://www.robotis.com/en/dynamixel.php) servo motors. Supports **Protocol 1.0** and **Protocol 2.0** over UART.

## Supported Models

AX-12A · AX-12W · AX-18A · DX-113 · DX-116 · DX-117 · RX-10 · RX-24F ·
RX-28 · RX-64 · EX-106 · MX-12W · MX-28 · MX-64 · MX-106 · MX-28(2.0) ·
MX-64(2.0) · MX-106(2.0) · XL-320 · XL-330 · XC-330 · XC-430 · XL-430 ·
XM-430 · XH-430 · XD-430 · XM-540 · XH-540 · XD-540 · XW-430 · XW-540 ·
PRO series · PRO+ series · Y series

## API Overview

| Property / Method         | Description                            |
| ------------------------- | -------------------------------------- |
| `ping(baudrate, id=254)`  | Scan for a motor; loads control table  |
| `reset()`                 | Factory-reset the motor                |
| `torque_enabled`          | Enable / disable motor output          |
| `led`                     | LED on / off                           |
| `operating_mode`          | Operating mode (position, velocity, …) |
| `goal_position`           | Goal position (raw ticks)              |
| `goal_position_rel`       | Goal position normalised to 0.0 – 1.0  |
| `goal_velocity`           | Goal velocity (raw units, signed)      |
| `goal_pwm`                | Goal PWM (signed)                      |
| `current_position`        | Present position (signed)              |
| `current_velocity`        | Present velocity (signed)              |
| `profile_velocity`        | Profile velocity for trajectory gen.   |
| `position_limit_low/high` | Position limits                        |
| `velocity_limit`          | Velocity limit                         |

See the source for the complete list of properties.

## Installation

### Via `mip` (network-capable boards)

```python
import mip
mip.install("github:SainsburyWellcomeCentre/micropython-dynamixel")
```

### Via `mpremote` (from your PC)

```bash
mpremote mip install github:SainsburyWellcomeCentre/micropython-dynamixel
```

### Manual

Copy the contents of `src/` to `lib/dynamixel/` on your device.

## Quick Start

If you don't know the baudrate of your motor, run `example/scan.py` to find it. Set the `UART_ID`, `TX_PIN`, and `RX_PIN` variables to match your board's pinout. The script will scan through common baudrates and print the model of any motor it finds.

```bash
mpremote cp -r example/scan.py :main.py
mpremote run main.py
```

If you know the baudrate and ID of your motor, you can skip the scan and run the following example code:

```python
from machine import UART
from dynamixel import Dynamixel, DynamixelModel
import time

# Initialise UART (pins depend on your board)
uart = UART(0, baudrate=57600, tx=0, rx=1)

# Create a driver instance for XM430_W350_T (Protocol 2.0)
dxl = Dynamixel(uart, model=DynamixelModel.XM430_W350_T, id=1, protocol_version=2)

dxl.torque_enabled = False
dxl.operating_mode = 3  # 3 = Position Control Mode (single-turn)
dxl.torque_enabled = True

while True:
    dxl.goal_position += 2048
    print("Current Position:", dxl.current_position)
    time.sleep(1)
```

## License

**Sainsbury Wellcome Centre code, firmware, and software is released under the [BSD 3-Clause License](https://opensource.org/license/bsd-3-clause).**

## 📚 Credits

This library references the following resources:

- [Dynamixel2Arduino](https://github.com/ROBOTIS-GIT/dynamixel2arduino)
- [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/main)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## ❤ Contributors

 <a href = "https://github.com/SainsburyWellcomeCentre/micropython-dynamixel/graphs/contributors">
   <img src = "https://contrib.rocks/image?repo=SainsburyWellcomeCentre/micropython-dynamixel" alt="Contributors"/>
 </a>

## 📧 Contact

- **Author**: [Sainsbury Wellcome Centre FabLabs](https://www.sainsburywellcome.org/content/fablab)
- **Email**: [swc.fablabs@ucl.ac.uk](mailto:swc.fablabs@ucl.ac.uk)
- **Website**: [FabLabs](https://sainsburywellcomecentre.github.io/fablabs-documentation/#micropython-dynamixel)
