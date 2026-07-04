#ifndef ENCODER_ODOMETRY_H
#define ENCODER_ODOMETRY_H

#include <Arduino.h>
#include "config.h"

// External Interrupt Service Routines (ISRs) declared with IRAM_ATTR for ESP32 RAM execution.
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();

class EncoderOdometry {
public:
    /**
     * @brief Constructor for Encoder Odometry.
     */
    EncoderOdometry();

    /**
     * @brief Attaches hardware interrupts to the encoder pins.
     * Must be called in setup().
     */
    void begin();

    /**
     * @brief Reads the accumulated ticks since last update, computes raw speeds,
     * and updates the dead-reckoning pose estimate (x, y, theta).
     * Typically called at high frequency (50 Hz / 20ms).
     * 
     * @param dt Elapsed time in seconds since the last update.
     */
    void update(float dt);

    /**
     * @brief Retrieves the current dead-reckoning pose of the robot.
     * 
     * @param x Outputs X position in meters.
     * @param y Outputs Y position in meters.
     * @param theta Outputs heading angle in radians (-PI to PI).
     */
    void getPose(float &x, float &y, float &theta) const;

    /**
     * @brief Retrieves the current linear and angular velocities.
     * 
     * @param linear Outputs linear velocity in m/s.
     * @param angular Outputs angular velocity in rad/s.
     */
    void getVelocities(float &linear, float &angular) const;

    /**
     * @brief Resets the pose estimate (x, y, theta) and tick counts back to zero.
     */
    void reset();

    /**
     * @brief Gets the current raw tick counts for left and right encoders.
     */
    void getTicks(long &left, long &right) const;

    /**
     * @brief Manually overrides the current yaw (theta) angle.
     * Useful when fusing with IMU data.
     * 
     * @param newTheta The new heading in radians.
     */
    void setTheta(float newTheta);

private:
    // Raw pose variables
    float x;
    float y;
    float theta;

    // Velocities
    float linearVelocity;
    float angularVelocity;

    // Track tick states at previous update
    long prevLeftTicks;
    long prevRightTicks;
};

#endif // ENCODER_ODOMETRY_H
