#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// PIN MAPPINGS
// =============================================================================

// L298N Motor Driver Pins
// (Using LEDC on ESP32 for hardware PWM generation)
#define PIN_MOTOR_L_PWM  18  // Left motor speed control pin
#define PIN_MOTOR_L_IN1  5 // Left motor direction pin 1
#define PIN_MOTOR_L_IN2  14  // Left motor direction pin 2

#define PIN_MOTOR_R_PWM  23  // Right motor speed control pin
#define PIN_MOTOR_R_IN1  4  // Right motor direction pin 1
#define PIN_MOTOR_R_IN2  2  // Right motor direction pin 2

// Servo Scanner Pin
#define PIN_SERVO_SCAN   15  // MG90S Sweep Servo signal pin

// Wheel Encoder Pins
#define PIN_ENCODER_L    26  // Left wheel IR speed encoder pulse pin
#define PIN_ENCODER_R    27  // Right wheel IR speed encoder pulse pin

// I2C Pins (For MPU6050 IMU and VL53L0X ToF)
#define PIN_I2C_SDA      21  // ESP32 default SDA pin
#define PIN_I2C_SCL      22  // ESP32 default SCL pin

// VL53L0X XSHUT (Optional shutdown control pin)
// #define PIN_TOF_XSHUT    4   // Pulled HIGH to enable, LOW to reset/shutdown. will use at next version

// =============================================================================
// LEDC PWM CONFIGURATION (ESP32 specific)
// =============================================================================
#define LEDC_PWM_FREQ       1000 // 1kHz frequency (optimal for L298N motor driver efficiency and response)
#define LEDC_PWM_RES        8     // 8-bit resolution (0 - 255)
#define LEDC_PWM_CH_LEFT    0     // PWM channel for left motor
#define LEDC_PWM_CH_RIGHT   1     // PWM channel for right motor

// =============================================================================
// I2C ADDRESSES
// =============================================================================
#define ADDR_IMU_MPU6050    0x68  // Default MPU6050 address
#define ADDR_TOF_VL53L0X    0x29  // Default VL53L0X address

// =============================================================================
// ROBOT GEOMETRY & TUNING CONSTANTS
// =============================================================================
// Modify these parameters according to your physical car chassis dimensions
const float WHEEL_DIAMETER_M = 0.067; // Wheel diameter in meters (e.g. standard 65mm yellow wheel)
const float WHEEL_BASE_M     = 0.105; // Distance between left and right wheel contact points in meters
const int   ENCODER_CPR      = 20;    // Encoder Counts Per Revolution (ticks per single full wheel turn)

// Calculated helper constants for Odometry
const float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * PI;
const float METERS_PER_TICK       = WHEEL_CIRCUMFERENCE_M / (float)ENCODER_CPR;

// Sensor Fusion Weights (Complementary Filter)
const float ALPHA_GYRO            = 0.98; // Gyroscope weight (high pass)
const float ALPHA_ACCEL           = 0.02; // Accelerometer weight (low pass)

// Serial Communication Settings
#define SERIAL_BAUD_RATE    115200

// Default Motor Calibration Scales (Adjust manually as needed)
#define DEFAULT_LEFT_MOTOR_SCALE  1.0f
#define DEFAULT_RIGHT_MOTOR_SCALE 1.0f

// =============================================================================
// NETWORK CONFIGURATION
// =============================================================================
#define UDP_SEND_PORT    5005   // ESP32 sends sensor data to host on this port
#define UDP_RECV_PORT    5006   // ESP32 listens for velocity commands on this port

#endif // CONFIG_H

