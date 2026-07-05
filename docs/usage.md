# Usage & Runtime

This document covers how to operate the robot at runtime and the telemetry you can expect on the host.

## Serial Telemetry

- Baud rate: 115200 (default)
- Telemetry packets include `$ODOM`, `$IMUR`, `$SCAN`, and warning messages such as `$WARN,COMM_TIMEOUT`.

Example `$ODOM` packet:

```
$ODOM,0.124,-0.045,0.785,0.22,0.15
```

Meaning: `x (m), y (m), theta (rad), linear_velocity (m/s), angular_velocity (rad/s)`.

## Sending Commands

To command the robot send ASCII command velocity packets over serial:

```
V,linear_x,angular_z
```

Example to drive forward at 0.2 m/s and turn left at 0.3 rad/s:

```
V,0.2,0.3
```

System control commands:

```
C,R   # Reset odometry
C,C   # Calibrate IMU
C,S   # Emergency stop
```

## Telemetry Receiver

`telemetry_receiver.py` is a simple host-side parser. Run with:

```powershell
python telemetry_receiver.py --port COM3 --baud 115200
```

The script prints parsed packets and can be extended to forward data to ROS, log CSV files, or plot scans.

## Safety

- If the firmware does not receive `cmd_vel` type packets within 2.0 seconds it will enter `SAFE_STOP` and set motor outputs to zero.
- Always test motors with the robot elevated or disconnected from wheels to verify direction and scaling before driving on the floor.
