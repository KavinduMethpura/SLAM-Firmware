#include "ServoScanner.h"

ServoScanner::ServoScanner() 
    : currentAngle(90), 
      sweepMin(10), 
      sweepMax(170), 
      sweepStep(5), 
      direction(1), 
      lastMoveTime(0), 
      stepDelayMs(40), // Standard settling delay for MG90S
      sweepCompletedFlag(false) {
}

void ServoScanner::begin() {
    // =========================================================================
    // STEP 1: Attach the Servo Pin
    // =========================================================================
    // Allocate only Timers 1, 2, and 3 for ESP32Servo.
    // This prevents the library from using Timer 0, which is reserved for the
    // MotorDriver PWM channels.
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // The ESP32Servo library requires calling attach() with:
    // - Pin number (PIN_SERVO_SCAN)
    // - Optional: min pulse width in microseconds (usually 544 for MG90S)
    // - Optional: max pulse width in microseconds (usually 2400 for MG90S)
    //
    // Initialize the servo position to the starting/center angle (90 degrees).

    myServo.attach(PIN_SERVO_SCAN, 500, 2400);
    myServo.write(currentAngle);
    lastMoveTime = millis();
}

bool ServoScanner::update() {
    sweepCompletedFlag = false;
    unsigned long currentTime = millis();

    // =========================================================================
    // STEP 2: Non-Blocking Timing Control
    // =========================================================================
    // We check if enough time has elapsed since the last move (currentTime - lastMoveTime >= stepDelayMs).
    // If not, we immediately return false to allow other code (odometry, IMU) to run.

    if (currentTime - lastMoveTime < stepDelayMs) {
        return false;
    }

    lastMoveTime = currentTime;

    // =========================================================================
    // STEP 3: Servo Stepping Logic
    // =========================================================================
    // 1. Calculate the next angle: nextAngle = currentAngle + (direction * sweepStep).
    // 2. Write the nextAngle to the servo: myServo.write(nextAngle).
    // 3. Check boundaries:
    //    - If nextAngle >= sweepMax:
    //         Set direction = -1 (sweep down)
    //         Set sweepCompletedFlag = true
    //    - If nextAngle <= sweepMin:
    //         Set direction = 1 (sweep up)
    //         Set sweepCompletedFlag = true
    // 4. Update currentAngle.
    // 5. Return true to signal that the servo has moved to a new step, meaning
    //    the main application can trigger a new ToF distance measurement.

    int nextAngle = currentAngle + (direction * sweepStep);
    myServo.write(nextAngle);
    
    if(nextAngle >= sweepMax){
        direction = -1;
        sweepCompletedFlag = true;
    }else if(nextAngle <= sweepMin){
        direction = 1;
        sweepCompletedFlag = true;
    }
    
    currentAngle = nextAngle;
    return true;
}

int ServoScanner::getAngle() const {
    return currentAngle;
}

void ServoScanner::setSweepRange(int minAngle, int maxAngle) {
    sweepMin = minAngle;
    sweepMax = maxAngle;
}

bool ServoScanner::isSweepDone() const {
    return sweepCompletedFlag;
}
