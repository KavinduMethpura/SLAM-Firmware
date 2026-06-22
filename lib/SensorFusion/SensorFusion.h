#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>
#include "config.h"

class SensorFusion {
public:
    /**
     * @brief Constructor for the Sensor Fusion module.
     */
    SensorFusion();

    /**
     * @brief Initializes the sensor fusion system state.
     */
    void begin();

    /**
     * @brief Combines encoder odometry outputs with the IMU gyro yaw rate to compute a fused pose.
     * Typically called at high frequency (50 Hz) immediately after reading sensors.
     * 
     * @param encX Current X estimated by encoders (meters)
     * @param encY Current Y estimated by encoders (meters)
     * @param encTheta Current heading estimated by encoders (radians)
     * @param gyroYawRate Current Z-axis rotation rate from IMU (rad/s)
     * @param dt Time elapsed since last update (seconds)
     */
    void update(float encX, float encY, float encTheta, float gyroYawRate, float dt);

    /**
     * @brief Gets the current fused pose estimate.
     * 
     * @param outX Fused X coordinate (meters)
     * @param outY Fused Y coordinate (meters)
     * @param outTheta Fused heading angle (radians, -PI to PI)
     */
    void getFusedPose(float &outX, float &outY, float &outTheta) const;

    /**
     * @brief Resets the fused pose variables.
     */
    void reset(float startX = 0.0f, float startY = 0.0f, float startTheta = 0.0f);

private:
    // Fused pose states
    float fusedX;
    float fusedY;
    float fusedTheta;

    // Track previous values for differential calculations
    float prevEncX;
    float prevEncY;
    float prevEncTheta;
};

#endif // SENSOR_FUSION_H
