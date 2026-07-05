# Setup & Build

This guide explains how to prepare your development environment and build the firmware.

## Requirements

- PlatformIO (recommended) — install via Visual Studio Code extension or `pip install platformio`.
- Python 3.8+ for host utilities.
- USB cable and a serial-capable ESP32 board.

## Install PlatformIO (CLI)

```powershell
pip install -U platformio
```

Or install the PlatformIO extension inside VS Code for an integrated workflow.

## Build and Upload

From the repository root:

```powershell
# Build
platformio run

# Upload (detects the connected serial device)
platformio run --target upload

# Monitor serial output (115200 baud)
platformio device monitor --baud 115200
```

If the upload fails, ensure you have the correct USB driver and the board is in flashing mode (some boards require you to hold `BOOT` while pressing `EN`/reset).

## Python Host Tools

Install required Python packages (if any):

```powershell
python -m pip install -r requirements.txt
```

If `requirements.txt` is not provided, the `telemetry_receiver.py` script uses only the standard library by default.

## Board Configuration

Modify `platformio.ini` to match your specific ESP32 board if necessary. The default configuration targets a commonly used dev kit.

## Troubleshooting

- Serial permissions on Windows: ensure the device shows under Device Manager and you have rights to open the COM port.
- If I2C devices hang under motor noise, see the I2C recovery logic in `IMUReader` and hardware noise mitigation in [docs/pin-mapping.md](docs/pin-mapping.md).
