# ESP32 Pin Mapping & Electrical Configurations

This document serves as the hardware reference sheet for the SLAM Robotic Car.

## Hardware Pin Connections

| ESP32 Pin | Peripheral / Component | Signal Name | Type | Notes / Constraints |
| :--- | :--- | :--- | :--- | :--- |
| **GPIO 12** | L298N Motor Driver | ENA (Left PWM) | Output | Assigned to LEDC channel 0. |
| **GPIO 13** | L298N Motor Driver | IN1 (Left Dir 1) | Output | Set HIGH/LOW for direction. |
| **GPIO 14** | L298N Motor Driver | IN2 (Left Dir 2) | Output | Set LOW/HIGH for direction. |
| **GPIO 25** | L298N Motor Driver | ENB (Right PWM) | Output | Assigned to LEDC channel 1. |
| **GPIO 26** | L298N Motor Driver | IN3 (Right Dir 1) | Output | Set HIGH/LOW for direction. |
| **GPIO 27** | L298N Motor Driver | IN4 (Right Dir 2) | Output | Set LOW/HIGH for direction. |
| **GPIO 15** | MG90S Sweep Servo | PWM Control | Output | Utilizes the ESP32Servo library. |
| **GPIO 34** | Left Wheel IR Encoder | Pulse Input | Input | **Input Only.** Requires external pull-up on sensor board. |
| **GPIO 35** | Right Wheel IR Encoder | Pulse Input | Input | **Input Only.** Requires external pull-up on sensor board. |
| **GPIO 21** | I2C Bus | SDA (Data) | I/O | Shares SDA between MPU6050 and VL53L0X. |
| **GPIO 22** | I2C Bus | SCL (Clock) | I/O | Shares SCL between MPU6050 and VL53L0X. |
| **GPIO 4**  | VL53L0X ToF Sensor | XSHUT | Output | Used to hardware-reset/shutdown the ToF sensor. |

## Important Hardware & Electrical Notes

### 1. ESP32 Input-Only Pins (GPIO 34 & 35)
- GPIO 34 and 35 do **not** have internal pull-up or pull-down resistors in the ESP32 silicon.
- **Action**: Verify that your IR phototransistor / speed encoder boards include onboard pull-up resistors (typically $10\text{k}\Omega$ resistors pull to $3.3\text{V}$). If not, you must solder external pull-up resistors or choose other pins (e.g., GPIO 18 and 19) that support internal pull-ups (`pinMode(pin, INPUT_PULLUP)`).

### 2. Common Ground
- Ensure that the ESP32 ground (GND), L298N power ground, and battery ground are all connected together. 
- **Warning**: Failure to connect grounds will cause floating signals, causing motors to twitch uncontrollably or fail to spin, and sensors to return corrupt readings.

### 3. Noise & Decoupling on the I2C Bus
- Running DC motors creates significant electromagnetic interference (EMI). This noise can easily freeze the ESP32's hardware I2C bus (known issue with MPU6050/VL53L0X hanging during motor startup).
- **Mitigation**:
  1. Solder $0.1\mu\text{F}$ ceramic capacitors across the motor terminals (+ and -) to suppress spark noise.
  2. Keep SDA/SCL lines physically separated from the motor power wires.
  3. Keep I2C wires as short as possible.
  4. Rely on the `recoverI2C()` logic implemented in `IMUReader` to automatically clear bus lockups.
