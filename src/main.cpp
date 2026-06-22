#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"

MotorDriver motorDriver;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[TEST] Initializing Motor Driver...");
    
    motorDriver.begin();
    motorDriver.stop();
    Serial.println("[TEST] Motor Driver Initialized. Starting test loop in 3 seconds...");
    delay(3000);
}

void loop() {
    // 1. Forward
    Serial.println("[TEST] Moving forward (PWM 150)...");
    motorDriver.drive(150, 150);
    delay(2000);
    
    // Stop
    Serial.println("[TEST] Stopping...");
    motorDriver.stop();
    delay(1000);
    
    // 2. Backward
    Serial.println("[TEST] Moving backward (PWM -150)...");
    motorDriver.drive(-150, -150);
    delay(2000);
    
    // Stop
    Serial.println("[TEST] Stopping...");
    motorDriver.stop();
    delay(1000);
    
    // 3. Turn Left
    Serial.println("[TEST] Turning left (PWM -150, 150)...");
    motorDriver.drive(-150, 150);
    delay(2000);
    
    // Stop
    Serial.println("[TEST] Stopping...");
    motorDriver.stop();
    delay(1000);
    
    // 4. Turn Right
    Serial.println("[TEST] Turning right (PWM 150, -150)...");
    motorDriver.drive(150, -150);
    delay(2000);
    
    // Stop
    Serial.println("[TEST] Stopping...");
    motorDriver.stop();
    
    Serial.println("[TEST] Test cycle complete. Waiting 5 seconds before next run...");
    delay(5000);
}
