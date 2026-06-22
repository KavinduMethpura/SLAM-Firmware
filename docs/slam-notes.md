# SLAM Algorithm Notes & Integration Guide

This document discusses the coordinate systems, sensor characteristics, and mapping/localization algorithms suitable for this ESP32 robot car.

## Coordinate Frames (REP 105 Standard)

We follow the standard ROS (Robot Operating System) spatial representation conventions:

1.  **`base_link`**: Attached to the rigid body of the robot.
    *   Origin: Center of the drive wheel axle.
    *   $X$-axis: Points forward.
    *   $Y$-axis: Points left.
    *   $Z$-axis: Points up.
2.  **`odom`**: Odometry coordinate frame.
    *   Origin: The position where the robot was powered on or where the pose was reset.
    *   This frame drifts over time due to wheel slippage and gyro integration error, but it is locally continuous (no jumps).
3.  **`map`**: Map coordinate frame.
    *   Fixed world frame. Standard SLAM algorithms correct for the `odom` $\rightarrow$ `map` drift using laser scan matches against the constructed occupancy grid.

## SLAM Algorithm Choices

Because the ESP32 has limited RAM ($520\text{KB}$ internal SRAM), running full 2D SLAM (mapping + localization) directly on the microcontroller is not feasible. The typical architecture is:
- **Low-level (ESP32)**: Fuses high-rate sensors (IMU + Encoders) to provide high-speed, low-latency odometry (`odom` frame). Performs physical ToF scanning sweeps.
- **High-level (Host PC / Raspberry Pi)**: Runs the mapping algorithm (Hector SLAM, Gmapping, Cartographer) receiving the scan points and odometry, and publishing the map.

### Recommended SLAM Packages:

1.  **Hector SLAM**:
    *   *Pros*: Does not require wheel odometry! It relies purely on high-update-rate LIDAR/scanner data and fits scans using scan-matching.
    *   *Cons*: Since our MG90S servo scanner is relatively slow compared to a $360^\circ$ LIDAR (takes ~1-2 seconds per sweep), Hector SLAM might fail if the robot rotates too quickly.
2.  **Gmapping**:
    *   *Pros*: Industry standard for 2D differential-drive robots. Uses the odometry estimate (`$ODOM` packet) to predict robot movement, then refines the prediction using the ToF sensor scans.
    *   *Cons*: Requires a good wheel odometry calibration.
3.  **Cartographer**:
    *   *Pros*: Excellent performance, uses submaps, very robust.
    *   *Cons*: Resource-intensive on the host computer.

## Dealing with the Servo Scanner Limitations

Standard SLAM algorithms expect a single, simultaneous snapshot of a $360^\circ$ range scan. Our MG90S servo is a **sweeping scanner**, meaning range readings are collected sequentially over time as the servo rotates.

To avoid map distortion (skewing) while the robot is moving:
- **Skew Correction**: The host must use the robot's estimated velocity to project each individual `$SCAN` range reading into the world coordinate frame using the exact timestamp/position when the reading was taken.
- Alternatively, keep the robot still while performing a scan, then move, then stop and scan again (Stop-Scan-Go pattern).

## Odometry & Sensor Fusion Calibration

### 1. Systematic Odometry Calibration (UMea Method)
Run the robot in a square path ($1\text{m} \times 1\text{m}$) clockwise and counter-clockwise.
- If it ends up short of the start line: increase `WHEEL_DIAMETER_M`.
- If it turns too much/little: adjust `WHEEL_BASE_M` (track width).

### 2. IMU Gyro Calibration
At startup, the gyroscope must be kept perfectly still for 2-3 seconds. The IMU reader should average the first 500-1000 readings to calculate the gyroscope offsets (bias). Subtract these offsets from all future gyroscope readings.
- **Formula**: $\omega_z = \omega_{z,\text{raw}} - \text{Bias}_z$.
