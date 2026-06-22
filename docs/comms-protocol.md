# Serial Communication Protocol

This document defines the packet protocol used to send telemetry upstream from the ESP32 to the Host, and command velocities downstream from the Host to the ESP32.

The protocol uses a simple ASCII-based framing or direct light binary framing for speed and debugging ease. Below is the reference standard.

---

## 1. Upstream Telemetry (ESP32 $\rightarrow$ Host)

The ESP32 transmits telemetry packets periodically (10 Hz). Telemetry is packaged as a comma-separated values (CSV) string prefixed with a unique header character, terminated by a newline (`\n`).

### Pose Packet (Odom)
Contains the current fused pose estimate of the robot.
*   **Header**: `$ODOM`
*   **Format**: `$ODOM,x,y,theta,linear_velocity,angular_velocity\n`
*   **Fields**:
    *   `x`: Position along x-axis (meters, float)
    *   `y`: Position along y-axis (meters, float)
    *   `theta`: Heading angle (radians, float, range $-\pi$ to $\pi$)
    *   `linear_velocity`: Current forward speed (meters/sec, float)
    *   `angular_velocity`: Current rotational speed (rad/sec, float)
*   **Example**: `$ODOM,0.124,-0.045,0.785,0.22,0.15`

### IMU Raw Packet
Provides raw data for advanced host processing.
*   **Header**: `$IMUR`
*   **Format**: `$IMUR,ax,ay,az,gx,gy,gz\n`
*   **Fields**:
    *   `ax, ay, az`: Accelerometer values (g, float)
    *   `gx, gy, gz`: Gyroscope rates (degrees/sec, float)
*   **Example**: `$IMUR,0.01,0.02,0.98,0.1,-1.2,0.5`

### ToF Scanner Packet
Transmits distance sweeps from the MG90S servo scanner.
*   **Header**: `$SCAN`
*   **Format**: `$SCAN,angle,distance_mm\n`
*   **Fields**:
    *   `angle`: Servo angle (degrees, integer 0 - 180)
    *   `distance_mm`: Distance reading from VL53L0X (millimeters, integer)
*   **Example**: `$SCAN,45,340`

---

## 2. Downstream Commands (Host $\rightarrow$ ESP32)

Commands from the host control the movement of the car. The ESP32 parses incoming serial data in its main loop.

### Command Velocity (`cmd_vel`)
Sets the desired linear and angular velocity of the robot.
*   **Header**: `V` or `$CMD`
*   **Format**: `V,linear_x,angular_z\n`
*   **Fields**:
    *   `linear_x`: Desired forward speed (meters/sec, float)
    *   `angular_z`: Desired turning rate (radians/sec, float, positive is CCW/left turn)
*   **Example**: `V,0.35,-0.5` (Drive forward at 0.35 m/s while turning right at 0.5 rad/s)

### System Control Commands
Utility commands to set system modes or recalibrate parameters.
*   **Format**: `C,command_type\n`
*   **Fields**:
    *   `command_type`:
        *   `R`: Reset Odometry to (0,0,0)
        *   `C`: Trigger IMU gyroscope calibration
        *   `S`: Emergency Stop (Enter SAFE_STOP state)
*   **Example**: `C,R`

---

## 3. Communication Timeout & Safety

To prevent the robot from driving off and crashing if the serial cable is disconnected or the host software crashes:
1.  **Heartbeat**: Every command received resets a watchdog timer (`last_comm_time = millis()`).
2.  **Timeout**: If no command packet is received within **2.0 seconds** (`millis() - last_comm_time > 2000`), the ESP32 state machine transitions into `SAFE_STOP`.
3.  **Action**: Both motors are immediately set to $0$ speed, and a warning packet `$WARN,COMM_TIMEOUT\n` is sent to serial.
