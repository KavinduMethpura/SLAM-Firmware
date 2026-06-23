#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "ServoScanner.h"

MotorDriver motorDriver;
ServoScanner servoScanner;

// Helper function to perform a delay while continuously updating and testing the servo sweep
void delayWithServoUpdate(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        if (servoScanner.update()) {
            Serial.printf("[SERVO] Angle: %d\n", servoScanner.getAngle());
            if (servoScanner.isSweepDone()) {
                Serial.println("[SERVO] Sweep completed!");
            }
        }
        yield(); // Yield to ESP32 system tasks / watchdog
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[TEST] Initializing Motor Driver & Servo Scanner...");
    
    motorDriver.begin();
    motorDriver.stop();
    
    servoScanner.begin();
    
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
