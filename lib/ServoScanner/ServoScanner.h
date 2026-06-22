#ifndef SERVO_SCANNER_H
#define SERVO_SCANNER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

class ServoScanner {
public:
    /**
     * @brief Constructor for the Servo Scanner.
     */
    ServoScanner();

    /**
     * @brief Attaches the Servo object to the physical control pin.
     * Must be called in setup().
     */
    void begin();

    /**
     * @brief Non-blocking update function. Moves the servo incrementally
     * based on time intervals, allowing other tasks to run.
     * 
     * @return true if the servo has reached a new step and is ready for a sensor read.
     * @return false otherwise.
     */
    bool update();

    /**
     * @brief Gets the current angle of the servo scanner.
     * 
     * @return int Current angle in degrees (0 - 180).
     */
    int getAngle() const;

    /**
     * @brief Sets the angular sweep limits.
     * 
     * @param minAngle Minimum angle limit (typically >= 0)
     * @param maxAngle Maximum angle limit (typically <= 180)
     */
    void setSweepRange(int minAngle, int maxAngle);

    /**
     * @brief Checks if a complete left-to-right or right-to-left sweep has just finished.
     * Useful to trigger host maps updates.
     */
    bool isSweepDone() const;

private:
    Servo myServo;

    int currentAngle;
    int sweepMin;
    int sweepMax;
    int sweepStep;        // Angular step size per movement (e.g., 2 degrees)
    int direction;        // +1 for sweeping up, -1 for sweeping down
    
    unsigned long lastMoveTime;
    unsigned long stepDelayMs; // Time to wait at each step for the servo to settle (e.g., 30-50ms)
    
    bool sweepCompletedFlag;
};

#endif // SERVO_SCANNER_H
