#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "ServoScanner.h"
#include "CommProtocol.h"
#include "ToFSensor.h"
#include "IMUReader.h"
#include "EncoderOdometry.h"

MotorDriver motorDriver;
ServoScanner servoScanner;
CommProtocol comm;
ToFSensor tofSensor;
IMUReader imuReader;
EncoderOdometry odometry;

// Helper function to perform a delay while continuously updating and testing the servo sweep
void delayWithServoUpdate(unsigned long ms) {
    unsigned long start = millis();
    unsigned long lastImuPrint = 0;
    unsigned long lastOdomUpdate = 0;
    unsigned long lastOdomPrint = 0;
    
    while (millis() - start < ms) {
        // Read IMU at high rate to keep sensor data fresh
        imuReader.read();

        // Print IMU data at 10Hz (every 100ms)
        unsigned long now = millis();
        if (now - lastImuPrint >= 100) {
            float ax, ay, az, gx, gy, gz;
            imuReader.getRawData(ax, ay, az, gx, gy, gz);
            Serial.printf("[IMU] Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f\n", ax, ay, az, gx, gy, gz);
            lastImuPrint = now;
        }

        // Update encoder odometry at 50Hz (every 20ms)
        if (now - lastOdomUpdate >= 20) {
            float dt = (now - lastOdomUpdate) / 1000.0f;
            lastOdomUpdate = now;
            odometry.update(dt);
        }

        // Print odometry pose at 2Hz (every 500ms)
        if (now - lastOdomPrint >= 500) {
            float ox, oy, otheta;
            odometry.getPose(ox, oy, otheta);
            float linVel, angVel;
            odometry.getVelocities(linVel, angVel);
            Serial.printf("[ODOM] Pose: X=%.3f m, Y=%.3f m, Theta=%.3f rad | Vel: Lin=%.3f m/s, Ang=%.3f rad/s\n", ox, oy, otheta, linVel, angVel);
            lastOdomPrint = now;
        }

        if (servoScanner.update()) {
            int angle = servoScanner.getAngle();
            int dist = tofSensor.readDistance();
            
            Serial.printf("[SERVO] Angle: %d | [TOF] Distance: %d mm\n", angle, dist);
            
            if (servoScanner.isSweepDone()) {
                Serial.println("[SERVO] Sweep completed!");
            }
        }
        yield(); // Yield to ESP32 system tasks / watchdog
    }
}

void setup() {
    // Initialize communication protocol (handles Serial initialization, WiFi connection, and UDP port listener)
    comm.begin();
    
    Serial.println("[TEST] Initializing Motor Driver, Servo Scanner, ToF Sensor, IMU & Odometry...");
    
    motorDriver.begin();
    motorDriver.stop();
    
    servoScanner.begin();
    
    odometry.begin();
    
    if (!tofSensor.begin()) {
        Serial.println("[TEST] ERROR: Failed to detect VL53L0X ToF sensor!");
    } else {
        Serial.println("[TEST] VL53L0X ToF sensor successfully initialized.");
    }

    if (!imuReader.begin()) {
        Serial.println("[TEST] ERROR: Failed to detect MPU6050 IMU!");
    } else {
        Serial.println("[TEST] MPU6050 IMU successfully initialized. Calibrating gyroscope (keep robot stationary)...");
        imuReader.calibrate();
        Serial.println("[TEST] IMU calibration completed.");
    }
    
    Serial.println("[TEST] Initialized. Starting test loop in 3 seconds...");
    delay(3000);
}

void loop() {
    // 1. Forward
    Serial.println("[TEST] Moving forward (PWM 150)...");
    motorDriver.drive(150, 150);
    delayWithServoUpdate(2000);
    
    // Stop
    Serial.println("[TEST] Stopping...");
    motorDriver.stop();
    delayWithServoUpdate(1000);
    
    // 2. Backward
    Serial.println("[TEST] Moving backward (PWM -150)...");
    motorDriver.drive(-150, -150);
    delayWithServoUpdate(2000);
    
    // Stop
    Serial.println("[TEST] Stopping...");
    motorDriver.stop();
    delayWithServoUpdate(1000);
    
    // 3. Turn Left
    Serial.println("[TEST] Turning left (PWM -150, 150)...");
    motorDriver.drive(-150, 150);
    delayWithServoUpdate(2000);
    
    // Stop
    Serial.println("[TEST] Stopping...");
    motorDriver.stop();
    delayWithServoUpdate(1000);
    
    // 4. Turn Right
    Serial.println("[TEST] Turning right (PWM 150, -150)...");
    motorDriver.drive(150, -150);
    delayWithServoUpdate(2000);
    
    // Stop
    Serial.println("[TEST] Stopping...");
    motorDriver.stop();
    
    Serial.println("[TEST] Test cycle complete. Waiting 5 seconds before next run...");
    delayWithServoUpdate(5000);
}
