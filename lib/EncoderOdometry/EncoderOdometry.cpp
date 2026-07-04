#include "EncoderOdometry.h"

// =============================================================================
// GLOBAL INTERRUPT TICK COUNTERS
// =============================================================================
// We use volatile variables since they are updated within an interrupt context.
// IRAM_ATTR places this code in the ESP32 instruction RAM instead of flash memory,
// which is required for high frequency interrupt handlers on the ESP32.

volatile long leftTickCount = 0;
volatile long rightTickCount = 0;

void IRAM_ATTR leftEncoderISR() {
    // TODO: Increment leftTickCount (or decrement based on motor direction status if needed)
    leftTickCount++; 
}

void IRAM_ATTR rightEncoderISR() {
    // TODO: Increment rightTickCount (or decrement based on motor direction status if needed)
    rightTickCount++;
}

// =============================================================================
// CLASS IMPLEMENTATION
// =============================================================================

EncoderOdometry::EncoderOdometry() 
    : x(0.0f), y(0.0f), theta(0.0f), 
      linearVelocity(0.0f), angularVelocity(0.0f), 
      prevLeftTicks(0), prevRightTicks(0) {
}

void EncoderOdometry::begin() {
    // =========================================================================
    // STEP 1: Set pin mode and attach interrupts
    // =========================================================================

    // 1. Configure PIN_ENCODER_L and PIN_ENCODER_R as INPUT (or INPUT_PULLUP if needed).
    // 2. Use attachInterrupt() to bind the pins to leftEncoderISR and rightEncoderISR.
    //    Use RISING or CHANGE depending on your encoder sensor disk properties.
    
    pinMode(PIN_ENCODER_L, INPUT_PULLUP);
    pinMode(PIN_ENCODER_R, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R), rightEncoderISR, RISING);
}

void EncoderOdometry::update(float dt) {
    if (dt <= 0.0f) return;

    // =========================================================================
    // STEP 2: Read Tick Counts Safely (Preventing Race Conditions)
    // =========================================================================
    // On 32-bit MCUs like ESP32, reading a 32-bit variable is atomic, but
    // disabling interrupts briefly or using a critical section ensures synchronization.
    // Copy the volatile global tick variables into local variables while interrupts are disabled,
    // then re-enable interrupts.
    
    long currentLeftTicks;
    long currentRightTicks;
    
    noInterrupts(); // Disable interrupts
    currentLeftTicks = leftTickCount;
    currentRightTicks = rightTickCount;
    interrupts();   // Re-enable interrupts

    // =========================================================================
    // STEP 3: Calculate Delta Ticks and Delta Wheel Distances
    // =========================================================================
    // 1. Find delta ticks since last call: 
    //    d_ticks_L = currentLeftTicks - prevLeftTicks
    //    d_ticks_R = currentRightTicks - prevRightTicks
    // 2. Convert ticks to actual wheel travel distance in meters using METERS_PER_TICK (from config.h):
    //    d_dist_L = d_ticks_L * METERS_PER_TICK
    //    d_dist_R = d_ticks_R * METERS_PER_TICK
    
    long d_ticks_L = currentLeftTicks - prevLeftTicks;
    long d_ticks_R = currentRightTicks - prevRightTicks;
    
    float d_dist_L = (float)d_ticks_L * METERS_PER_TICK;
    float d_dist_R = (float)d_ticks_R * METERS_PER_TICK;

    // =========================================================================
    // STEP 4: Apply Differential Drive Kinematics
    // =========================================================================
    // Use the kinematics equations to update x, y, and theta:
    // - Center displacement (delta_d): (d_dist_R + d_dist_L) / 2
    // - Rotation change (delta_theta): (d_dist_R - d_dist_L) / WHEEL_BASE_M
    //
    // - Intermediate orientation: theta_mid = theta + (delta_theta / 2)
    // - Pose updates:
    //   x = x + delta_d * cos(theta_mid)
    //   y = y + delta_d * sin(theta_mid)
    //   theta = theta + delta_theta
    //
    // - Normalize theta to stay within [-PI, PI]:
    //   while (theta > PI) theta -= 2.0 * PI;
    //   while (theta < -PI) theta += 2.0 * PI;

    float delta_d = (d_dist_R + d_dist_L) / 2.0f;
    float delta_theta = (d_dist_R - d_dist_L) / WHEEL_BASE_M;
    
    float theta_mid = theta + (delta_theta / 2.0f);
    
    x += delta_d * cosf(theta_mid);
    y += delta_d * sinf(theta_mid);
    theta += delta_theta;
    
    // Normalize theta to stay within [-PI, PI]
    while (theta > PI) {
        theta -= 2.0f * PI;
    }
    while (theta < -PI) {
        theta += 2.0f * PI;
    }

    // =========================================================================
    // STEP 5: Calculate Speeds for Telemetry
    // =========================================================================
    // - linearVelocity = delta_d / dt
    // - angularVelocity = delta_theta / dt
    
    linearVelocity = delta_d / dt;
    angularVelocity = delta_theta / dt;

    // Save current ticks for the next loop comparison
    prevLeftTicks = currentLeftTicks;
    prevRightTicks = currentRightTicks;
}

void EncoderOdometry::getPose(float &outX, float &outY, float &outTheta) const {
    outX = x;
    outY = y;
    outTheta = theta;
}

void EncoderOdometry::getVelocities(float &outLinear, float &outAngular) const {
    outLinear = linearVelocity;
    outAngular = angularVelocity;
}

void EncoderOdometry::reset() {
    noInterrupts();
    leftTickCount = 0;
    rightTickCount = 0;
    interrupts();

    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;
    linearVelocity = 0.0f;
    angularVelocity = 0.0f;
    prevLeftTicks = 0;
    prevRightTicks = 0;
}

void EncoderOdometry::setTheta(float newTheta) {
    theta = newTheta;
}

void EncoderOdometry::getTicks(long &left, long &right) const {
    long tempLeft, tempRight;
    noInterrupts();
    tempLeft = leftTickCount;
    tempRight = rightTickCount;
    interrupts();
    left = tempLeft;
    right = tempRight;
}
