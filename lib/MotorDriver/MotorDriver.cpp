#include "MotorDriver.h"

MotorDriver::MotorDriver() {
    // Constructor
}

void MotorDriver::begin() {
    // =========================================================================
    // STEP 1: Configure Direction Pins as Outputs
    // =========================================================================
    // Use pinMode() to set PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_R_IN1, and PIN_MOTOR_R_IN2 as OUTPUT.
    // Ensure all direction pins start in a LOW state to prevent random movement on boot.
    
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    
    // =========================================================================
    // STEP 2: Configure ESP32 LEDC PWM
    // =========================================================================
    // ESP32 does not use standard analogWrite(). Instead, we use the LEDC API.
    // 1. Setup the PWM properties (frequency, resolution, channel) using:
    //    ledcSetup(channel, frequency, resolution)
    // 2. Attach the PWM pin to the specified LEDC channel using:
    //    ledcAttachPin(pin, channel)
    //
    // Use the constants defined in config.h:
    // - LEDC_PWM_CH_LEFT, LEDC_PWM_CH_RIGHT (channels)
    // - LEDC_PWM_FREQ (frequency e.g. 20000 Hz)
    // - LEDC_PWM_RES (resolution e.g. 8-bit)
    // - PIN_MOTOR_L_PWM, PIN_MOTOR_R_PWM (pins)

    ledcSetup(LEDC_PWM_CH_LEFT, LEDC_PWM_FREQ, LEDC_PWM_RES);
    ledcAttachPin(PIN_MOTOR_L_PWM, LEDC_PWM_CH_LEFT);
    ledcSetup(LEDC_PWM_CH_RIGHT, LEDC_PWM_FREQ, LEDC_PWM_RES);
    ledcAttachPin(PIN_MOTOR_R_PWM, LEDC_PWM_CH_RIGHT);
}

void MotorDriver::drive(int leftSpeed, int rightSpeed) {
    // =========================================================================
    // STEP 3: Write Speeds to Left and Right Motors
    // =========================================================================
    // We constrain the input speed to [-255, 255] and call setMotorSpeed().
    // You can also add safety features here (e.g. limiting acceleration or max speed).
    
    int leftConstrained = constrain(leftSpeed, -255, 255);
    int rightConstrained = constrain(rightSpeed, -255, 255);

    // Call helper for left motor (passing PWM channel)
    setMotorSpeed(leftConstrained, LEDC_PWM_CH_LEFT, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2);

    // Call helper for right motor (passing PWM channel)
    setMotorSpeed(rightConstrained, LEDC_PWM_CH_RIGHT, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2);
}

void MotorDriver::stop() {
    // =========================================================================
    // STEP 4: Quick Stop
    // =========================================================================
    // Set speed to 0 for both motors to bring them to a halt.
    drive(0, 0);
}

void MotorDriver::setMotorSpeed(int speed, uint8_t pwmChannel, uint8_t dirPin1, uint8_t dirPin2) {
    // =========================================================================
    // STEP 5: Motor Direction & Speed Logic (L298N Truth Table)
    // =========================================================================
    // The L298N controls motor direction through two input pins (IN1 & IN2):
    // - Forward:  IN1 = HIGH, IN2 = LOW
    // - Backward: IN1 = LOW,  IN2 = HIGH
    // - Brake:    IN1 = LOW,  IN2 = LOW (or both HIGH)
    //
    // Speed is controlled by writing the absolute value of 'speed' to the LEDC channel:
    // - Use ledcWrite(pwmChannel, absolute_speed_value)
    //
    // Instructions:
    // 1. Check if speed is positive (> 0):
    //    - Set dirPin1 to HIGH, dirPin2 to LOW
    //    - Write absolute speed to pwmChannel
    // 2. Check if speed is negative (< 0):
    //    - Set dirPin1 to LOW, dirPin2 to HIGH
    //    - Write absolute speed (e.g. -speed) to pwmChannel
    // 3. Otherwise (speed is 0):
    //    - Set both dirPin1 and dirPin2 to LOW
    //    - Write 0 to pwmChannel

    if (speed > 0) {
        // Forward
        digitalWrite(dirPin1, HIGH);
        digitalWrite(dirPin2, LOW);
        ledcWrite(pwmChannel, speed);
    } else if (speed < 0) {
        // Backward
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);
        ledcWrite(pwmChannel, -speed);
    } else {
        // Brake
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, LOW);
        ledcWrite(pwmChannel, 0);
    }
}
