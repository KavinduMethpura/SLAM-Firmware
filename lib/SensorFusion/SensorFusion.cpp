#include "SensorFusion.h"

SensorFusion::SensorFusion()
    : fusedX(0.0f), fusedY(0.0f), fusedTheta(0.0f),
      prevEncX(0.0f), prevEncY(0.0f), prevEncTheta(0.0f) {
}

void SensorFusion::begin() {
    reset();
}

void SensorFusion::update(float encX, float encY, float encTheta, float gyroYawRate, float dt) {
    if (dt <= 0.0f) return;

    // =========================================================================
    // STEP 1: Compute Delta Heading from Both Sensors
    // =========================================================================
    // 1. Encoder delta heading:
    //    d_theta_enc = encTheta - prevEncTheta
    //    Ensure you wrap d_theta_enc to [-PI, PI] in case the encoder pose wrapped.
    // 2. Gyroscope delta heading:
    //    d_theta_gyro = gyroYawRate * dt

    float d_theta_enc = encTheta - prevEncTheta;
    // Normalize d_theta_enc to [-PI, PI]
    while (d_theta_enc > M_PI) {
        d_theta_enc -= 2.0f * M_PI;
    }
    while (d_theta_enc < -M_PI) {
        d_theta_enc += 2.0f * M_PI;
    }
    
    float d_theta_gyro = gyroYawRate * dt;

    // =========================================================================
    // STEP 2: Fuse the Delta Headings (Complementary Filter)
    // =========================================================================
    // Gyroscopes are highly accurate over short durations but drift over time.
    // Encoders do not drift when stationary but slip during turns.
    // Combine them using ALPHA_GYRO (from config.h):
    //    d_theta_fused = ALPHA_GYRO * d_theta_gyro + (1.0f - ALPHA_GYRO) * d_theta_enc
    //
    // Update the fused heading:
    //    fusedTheta += d_theta_fused
    // Normalize fusedTheta to stay within [-PI, PI].

    float d_theta_fused = ALPHA_GYRO * d_theta_gyro + (1.0f - ALPHA_GYRO) * d_theta_enc;
    fusedTheta += d_theta_fused;
    
    // Normalize fusedTheta to stay within [-PI, PI]
    while (fusedTheta > M_PI) {
        fusedTheta -= 2.0f * M_PI;
    }
    while (fusedTheta < -M_PI) {
        fusedTheta += 2.0f * M_PI;
    }
    
    // =========================================================================
    // STEP 3: Compute Linear Displacement from Encoders
    // =========================================================================
    // 1. Calculate displacement along the ground from encoder X/Y changes:
    //    dx = encX - prevEncX
    //    dy = encY - prevEncY
    //    d_dist = sqrt(dx*dx + dy*dy)
    // 2. Determine direction of displacement:
    //    If the robot drove backwards, d_dist should be negative.
    //    You can check the sign by comparing the movement vector with the heading vector:
    //    direction = (dx * cos(encTheta) + dy * sin(encTheta) >= 0) ? 1.0 : -1.0
    //    d_dist = d_dist * direction

    float dx = encX - prevEncX;
    float dy = encY - prevEncY;
    float d_dist = sqrtf(dx * dx + dy * dy);
    
    float direction = (dx * cosf(encTheta) + dy * sinf(encTheta) >= 0) ? 1.0f : -1.0f;
    d_dist *= direction;

    // =========================================================================
    // STEP 4: Project Position Using Fused Heading
    // =========================================================================
    // Now project the linear displacement along our new, highly accurate fused heading:
    //    fusedX += d_dist * cos(fusedTheta + d_theta_fused / 2.0f)
    //    fusedY += d_dist * sin(fusedTheta + d_theta_fused / 2.0f)

    fusedX += d_dist * cosf(fusedTheta + d_theta_fused / 2.0f);
    fusedY += d_dist * sinf(fusedTheta + d_theta_fused / 2.0f);
    
    // Save current encoder variables for the next cycle
    prevEncX = encX;
    prevEncY = encY;
    prevEncTheta = encTheta;
}

void SensorFusion::getFusedPose(float &outX, float &outY, float &outTheta) const {
    outX = fusedX;
    outY = fusedY;
    outTheta = fusedTheta;
}

void SensorFusion::reset(float startX, float startY, float startTheta) {
    fusedX = startX;
    fusedY = startY;
    fusedTheta = startTheta;
    
    prevEncX = startX;
    prevEncY = startY;
    prevEncTheta = startTheta;
}
