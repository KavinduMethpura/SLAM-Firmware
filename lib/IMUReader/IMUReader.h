#ifndef IMU_READER_H
#define IMU_READER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

class IMUReader {
public:
    /**
     * @brief Constructor for the IMU Reader.
     */
    IMUReader();

    /**
     * @brief Initializes the MPU6050 IMU. If the sensor is not found or fails,
     * it triggers I2C recovery and attempts to initialize again.
     * 
     * @return true if initialization succeeded, false if IMU is offline.
     */
    bool begin();

    /**
     * @brief Reads raw sensor values from the IMU via I2C, scales them, 
     * and updates internal variables. Must be called in the high-frequency loop.
     * 
     * @return true if reading succeeded, false if an I2C transaction error occurred.
     */
    bool read();

    /**
     * @brief Calibrates the gyroscope by averaging offsets when the robot is static.
     * Must be called while the car is perfectly still.
     */
    void calibrate(int samples = 500);

    /**
     * @brief Manually toggles the I2C SCL clock lines to release a locked bus (SDA stuck LOW).
     * Re-initializes Wire on completion.
     */
    void recoverI2C();

    // Getters for filtered or calibrated readings
    float getGyroZ() const; // Yaw rate in radians/second
    float getAccelX() const; // Forward acceleration in m/s^2
    float getAccelY() const; // Lateral acceleration in m/s^2

    // Getters for raw values (useful for raw telemetry)
    void getRawData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) const;

private:
    // Raw scaled sensor readings
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;

    // Gyroscope calibration offsets (biases)
    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;

    // Helper function to write to a single register
    bool writeRegister(uint8_t reg, uint8_t value);
};

#endif // IMU_READER_H
