# Libraries & Module Reference

Short descriptions of the primary libraries in `lib/` and their responsibilities.

- `CommProtocol` — Serial framing and packet parsing/serialization for `$ODOM`, `$IMUR`, `$SCAN`, `V,` and `C,` messages. Use this to extend or change packet formats.
- `EncoderOdometry` — Interrupt-driven wheel encoder counting, tick-to-distance conversion, and raw odometry estimation.
- `IMUReader` — I2C interface to the MPU6050, gyroscope calibration, and filtered attitude/gyro outputs.
- `MotorDriver` — Abstraction for the L298N H-bridge with PWM control and direction handling.
- `SensorFusion` — Combines encoder and IMU data to compute a fused pose estimate `(x, y, theta)` and velocities.
- `ServoScanner` — Controls MG90S servo sweep and coordinates ToF readings into angle-distance pairs.
- `ToFSensor` — VL53L0X driver, measurement timing budget management, and error recovery (XSHUT control).

For contribution, open the appropriate library folder and follow the existing header/implementation patterns. Public API functions are documented at the top of each header file.
