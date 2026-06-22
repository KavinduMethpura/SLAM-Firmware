#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "config.h"

class MotorDriver {
public:
    /**
     * @brief Constructor for the Motor Driver.
     */
    MotorDriver();

    /**
     * @brief Initializes the ESP32 LEDC PWM channels and sets direction pins as outputs.
     * Must be called in setup().
     */
    void begin();

    /**
     * @brief Drives both motors with speed values between -255 (full reverse) and 255 (full forward).
     * 
     * @param leftSpeed Target speed for the left motor (-255 to 255)
     * @param rightSpeed Target speed for the right motor (-255 to 255)
     */
    void drive(int leftSpeed, int rightSpeed);

    /**
     * @brief Instantly stops both motors by writing speed 0 and putting the driver pins to LOW.
     */
    void stop();

private:
    /**
     * @brief Set speed and direction for a single motor using ESP32 LEDC API.
     * 
     * @param speed Target speed (-255 to 255)
     * @param pwmChannel LEDC channel assigned to the motor
     * @param dirPin1 Direction control pin 1 (IN1/IN3)
     * @param dirPin2 Direction control pin 2 (IN2/IN4)
     */
    void setMotorSpeed(int speed, uint8_t pwmChannel, uint8_t dirPin1, uint8_t dirPin2);
};

#endif // MOTOR_DRIVER_H
