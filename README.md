# SLAM-Firmware

Compact SLAM-capable robot firmware for ESP32-based differential-drive platforms.

## Overview

SLAM-Firmware implements low-level hardware drivers, sensor fusion, and a lightweight communication protocol for integrating an ESP32-based robot with a host computer that runs mapping and high-level SLAM. The ESP32 handles IMU, wheel encoders, a servo-mounted ToF scanner, and motor control; the host is responsible for mapping and visualization.

## Features

- Real-time sensor fusion (IMU + wheel encoders) for odometry
- Servo-mounted VL53L0X ToF scanner with angle-distance streaming
- Simple ASCII/binary serial protocol for telemetry and commands
- Motor control via L298N H-bridge with PWM and direction pins
- Non-blocking main loop with multiple frequency tasks (control, scanning, telemetry, safety)

## Hardware

- ESP32 development board
- MPU6050 IMU (I2C)
- VL53L0X ToF sensor (I2C)
- MG90S servo for scanner sweep (PWM)
- L298N motor driver (left/right channels)
- IR wheel encoders (digital pulses)

See [docs/pin-mapping.md](docs/pin-mapping.md) for the detailed pinout and electrical notes.

## Quick Start

Prerequisites:

- Install PlatformIO (recommended) or the Arduino/ESP32 toolchain.
- Python 3.8+ (for `telemetry_receiver.py` and host-side helpers).

Build and upload (PlatformIO):

```powershell
# Build
platformio run

# Upload to the ESP32 (auto-detects the connected board)
platformio run --target upload
```

Open a serial monitor to view telemetry (default baud 115200):

```powershell
platformio device monitor --baud 115200
```

Host telemetry receiver example (simple):

```powershell
# On Windows use the correct COM port, e.g. COM3
python telemetry_receiver.py --port COM3 --baud 115200
```

## Communication Protocol

The firmware speaks a small ASCII-based protocol. Telemetry headers include `$ODOM`, `$IMUR`, and `$SCAN`. Commands to the robot use `V,linear,angular` or `C,<cmd>`. See [docs/comms-protocol.md](docs/comms-protocol.md).

## Project Layout

- `src/` — main application source (`main.cpp`)
- `lib/` — device and algorithm libraries (e.g., `CommProtocol`, `IMUReader`, `SensorFusion`)
- `include/` — project-wide headers and configuration
- `docs/` — extended documentation and guides
- `platformio.ini` — PlatformIO build configuration
- `telemetry_receiver.py` — simple host-side serial parser and visualizer

## Development

See [docs/setup.md](docs/setup.md) for environment setup and [docs/development.md](docs/development.md) for contribution and coding guidelines.

## Modules and APIs

Refer to [docs/modules.md](docs/modules.md) for summaries of library responsibilities and public interfaces.

## License

This repository includes a `LICENSE` file at the project root. Follow the terms within for reuse and contribution guidance.

---

If you want, I can also run a quick build or add CI (GitHub Actions) to validate compile for the default board.
