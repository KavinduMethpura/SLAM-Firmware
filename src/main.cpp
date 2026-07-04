#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "ServoScanner.h"
#include "CommProtocol.h"
#include "ToFSensor.h"
#include "IMUReader.h"
#include "EncoderOdometry.h"
#include "SensorFusion.h"

MotorDriver motorDriver;
ServoScanner servoScanner;
CommProtocol comm;
ToFSensor tofSensor;
IMUReader imuReader;
EncoderOdometry odometry;
SensorFusion fusion;

// Robot states for autonomous logic
enum RobotState {
    STATE_STOPPED,
    STATE_FORWARD,
    STATE_AVOID,
    STATE_MANUAL,
    STATE_SCANNING,
    STATE_TURNING
};

RobotState currentState = STATE_STOPPED;

// Manual Control Velocities
float manualLinearX = 0.0f;
float manualAngularZ = 0.0f;

// Navigation Loop Parameters
unsigned long drivingStartTime = 0;
const unsigned long RUN_DURATION_MS = 10000; // 10 seconds run duration

// Motor Calibration Parameters
float leftMotorScale = DEFAULT_LEFT_MOTOR_SCALE;
float rightMotorScale = DEFAULT_RIGHT_MOTOR_SCALE;

// Scanning parameters
int bestAngle = 90;
int maxFoundDist = -1;

// Turning parameters
float startHeading = 0.0f;
float targetDeltaTheta = 0.0f;
float headingTurned = 0.0f;
float lastHeading = 0.0f;

// Obstacle Avoidance Configuration
const int SAFE_DISTANCE_MM = 300;           // Trigger avoidance if obstacle is within 30cm
const unsigned long AVOID_DURATION_MS = 800; // Time in milliseconds to spin in place
unsigned long avoidStartTime = 0;
bool turnDirectionLeft = true;               // Dynamic decision on turn direction

// Task Schedulers
unsigned long lastTime50Hz = 0;
unsigned long lastTime20Hz = 0;
unsigned long lastTime10Hz = 0;

void setup() {
    // 1. Initialize Communication Protocol (starts WiFi and UDP listening)
    comm.begin();
    
    Serial.println("[SYSTEM] Initializing Robot Hardware...");
    
    // 2. Initialize Motor Driver
    motorDriver.begin();
    motorDriver.stop();
    
    // 3. Initialize Servo Scanner
    servoScanner.begin();
    
    // 4. Initialize Encoders and Odometry
    odometry.begin();
    
    // 5. Initialize Sensor Fusion
    fusion.begin();
    
    // 6. Initialize ToF Sensor
    if (!tofSensor.begin()) {
        Serial.println("[SYSTEM] ERROR: Failed to detect VL53L0X ToF sensor!");
    } else {
        Serial.println("[SYSTEM] VL53L0X ToF sensor successfully initialized.");
    }

    // 7. Initialize IMU Reader
    if (!imuReader.begin()) {
        Serial.println("[SYSTEM] ERROR: Failed to detect MPU6050 IMU!");
    } else {
        Serial.println("[SYSTEM] MPU6050 IMU successfully initialized. Calibrating gyroscope (do not move)...");
        imuReader.calibrate();
        Serial.println("[SYSTEM] IMU calibration finished.");
    }
    
    // Setup task baseline timestamps
    unsigned long now = millis();
    lastTime50Hz = now;
    lastTime20Hz = now;
    lastTime10Hz = now;

    Serial.println("[SYSTEM] Initialization Complete! Entering Stopped State. Press 'I' to start the algorithm.");
}

void loop() {
    unsigned long now = millis();
    
    // Check for incoming commands from PC
    comm.update();
    
    // Handle remote reset / control commands from Python script
    if (comm.isNewControlAvailable()) {
        char cmd = comm.getControlCommand();
        if (cmd == 'R') {
            odometry.reset();
            fusion.reset();
            Serial.println("[REMOTE] Resetting Odometry and Sensor Fusion.");
        } else if (cmd == 'C') {
            Serial.println("[REMOTE] Calibrating Gyro. Keep robot stationary...");
            imuReader.calibrate();
            Serial.println("[REMOTE] Gyro calibration complete.");
        } else if (cmd == 'S') {
            Serial.println("[REMOTE] EMERGENCY STOP. Entering Stopped State.");
            motorDriver.stop();
            currentState = STATE_STOPPED;
        } else if (cmd == 'I') {
            Serial.println("[REMOTE] Start command received. Starting simulation...");
            odometry.reset();
            fusion.reset();
            currentState = STATE_FORWARD;
            drivingStartTime = now;
        }
    }

    // Handle manual calibration commands from Python script
    float kLeft, kRight;
    if (comm.getCalibrationCommand(kLeft, kRight)) {
        leftMotorScale = kLeft;
        rightMotorScale = kRight;
        motorDriver.setCalibrationScales(leftMotorScale, rightMotorScale);
        
        char calBuffer[128];
        snprintf(calBuffer, sizeof(calBuffer), "$DEBUG,Manually Set Scales -> Left: %.3f, Right: %.3f\n", leftMotorScale, rightMotorScale);
        comm.sendDebug(calBuffer);
        Serial.print(calBuffer);
    }

    // Handle manual keyboard control commands from Python script
    if (comm.isNewCommandAvailable()) {
        comm.getCommandVelocity(manualLinearX, manualAngularZ);
        currentState = STATE_MANUAL;
    }

    // =========================================================================
    // LOOP A: 50 Hz Task (IMU, Odometry & Sensor Fusion - every 20ms)
    // =========================================================================
    if (now - lastTime50Hz >= 20) {
        float dt = (now - lastTime50Hz) / 1000.0f;
        lastTime50Hz = now;
        
        // 1. Read IMU
        imuReader.read();
        
        // 2. Update Encoder Odometry
        odometry.update(dt);
        
        // 3. Retrieve encoder pose and gyroscope rate
        float encX, encY, encTheta;
        odometry.getPose(encX, encY, encTheta);
        float gyroYawRate = imuReader.getGyroZ();
        
        // 4. Update complementary filter pose estimation
        fusion.update(encX, encY, encTheta, gyroYawRate, dt);
    }

    // =========================================================================
    // LOOP B: 20 Hz Task (Servo Scanner Sweep & Obstacle Check - every 50ms)
    // =========================================================================
    if (currentState != STATE_STOPPED && now - lastTime20Hz >= 50) {
        lastTime20Hz = now;
        
        if (servoScanner.update()) {
            int angle = servoScanner.getAngle();
            int dist = tofSensor.readDistance();
            
            // Send the scanned angle-range pair over UDP to the host PC
            comm.sendScan(angle, dist);
            
            // Obstacle check: Only trigger when driving forward
            if (currentState == STATE_FORWARD) {
                // If a valid range reading is detected within the front field of view (60 to 120 deg)
                // and it is closer than the safety distance threshold:
                if (angle >= 60 && angle <= 120 && dist > 0 && dist < SAFE_DISTANCE_MM) {
                    Serial.printf("[AVOID] Obstacle at %d degrees! Range: %d mm\n", angle, dist);
                    
                    // Stop motors immediately to prevent collision
                    motorDriver.stop();
                    
                    // Choose turn direction: if obstacle is on the right side of the field of view
                    // (angle > 90), turn Left. If it is on the left side (angle <= 90), turn Right.
                    if (angle > 90) {
                        turnDirectionLeft = true;
                        Serial.println("[AVOID] Choosing Action: Spin LEFT.");
                    } else {
                        turnDirectionLeft = false;
                        Serial.println("[AVOID] Choosing Action: Spin RIGHT.");
                    }
                    
                    currentState = STATE_AVOID;
                    avoidStartTime = now;
                }
            } else if (currentState == STATE_AVOID) {
                // If the signal is out of range (meaning long distance / clear path) or far,
                // and the sensor is facing forward (around 90 degrees, i.e., 80 to 100 degrees),
                // we can immediately resume moving forward.
                if (angle >= 80 && angle <= 100 && (dist == -1 || dist > SAFE_DISTANCE_MM)) {
                    Serial.println("[AVOID] Path is clear (out of range/long distance). Resuming forward path.");
                    currentState = STATE_FORWARD;
                    drivingStartTime = now; // Reset timer when resuming
                }
            } else if (currentState == STATE_SCANNING) {
                // Collect scanner data during stopped sweep to find the clearest direction (max distance)
                int effectiveDist = (dist == -1) ? 8000 : dist; // Out-of-range (>2m) is the clearest
                if (dist > 0 || dist == -1) {
                    if (effectiveDist > maxFoundDist) {
                        maxFoundDist = effectiveDist;
                        bestAngle = angle;
                    }
                }
                
                // Once the scanner completes a sweep direction, execute the heading decision
                if (servoScanner.isSweepDone()) {
                    Serial.printf("[SCANNING] Complete! Best Angle: %d deg, Effective Dist: %d mm\n", bestAngle, maxFoundDist);
                    
                    // Turn relative to the center 90-degree position
                    int turnAngle = bestAngle - 90;
                    targetDeltaTheta = turnAngle * (PI / 180.0f);
                    
                    // Retrieve starting yaw heading from complementary filter
                    float fx, fy;
                    fusion.getFusedPose(fx, fy, startHeading);
                    lastHeading = startHeading;
                    headingTurned = 0.0f;
                    
                    Serial.printf("[SCANNING] Current Heading: %.3f rad, Target Turn Delta: %.3f rad (%d deg)\n", startHeading, targetDeltaTheta, turnAngle);
                    
                    if (abs(turnAngle) < 10) {
                        // If chosen path is essentially straight ahead, skip turning and go forward
                        Serial.println("[SCANNING] Clear path is straight. Driving forward.");
                        currentState = STATE_FORWARD;
                        drivingStartTime = now;
                    } else {
                        currentState = STATE_TURNING;
                    }
                }
            }
        }
    }

    // =========================================================================
    // LOOP C: 10 Hz Task (PC Telemetry Streaming - every 100ms)
    // =========================================================================
    unsigned long telemetryInterval = (currentState == STATE_STOPPED) ? 2000 : 100;
    if (now - lastTime10Hz >= telemetryInterval) {
        lastTime10Hz = now;
        
        // Get fused pose
        float fx, fy, ftheta;
        fusion.getFusedPose(fx, fy, ftheta);
        
        // Get velocities
        float linVel, angVel;
        odometry.getVelocities(linVel, angVel);
        
        // Stream telemetry over UDP to Python script (serves as connection heartbeat when stopped)
        comm.sendOdom(fx, fy, ftheta, linVel, angVel);
        
        // Stream raw IMU values (only stream raw IMU if running, to keep stopped state clean)
        if (currentState != STATE_STOPPED) {
            float ax, ay, az, gx, gy, gz;
            imuReader.getRawData(ax, ay, az, gx, gy, gz);
            comm.sendIMU(ax, ay, az, gx, gy, gz);
        }
    }

    // =========================================================================
    // MOTOR STATE MACHINE EXECUTION (Non-blocking)
    // =========================================================================
    if (currentState == STATE_STOPPED) {
        motorDriver.stop();
    } else if (currentState == STATE_FORWARD) {
        // Drive forward (scaling is applied internally by MotorDriver)
        motorDriver.drive(130, 130);
        
        // Transition to scanning after driving for the configured duration (10s)
        if (now - drivingStartTime >= RUN_DURATION_MS) {
            Serial.println("[FORWARD] Driving duration elapsed. Stopping to scan surrounding environment...");
            motorDriver.stop();
            currentState = STATE_SCANNING;
            maxFoundDist = -1;
            bestAngle = 90;
        }
    } else if (currentState == STATE_AVOID) {
        // Spin in place
        if (turnDirectionLeft) {
            motorDriver.drive(-140, 140);
        } else {
            motorDriver.drive(140, -140);
        }
        
        // Check if the spin duration has finished
        if (now - avoidStartTime >= AVOID_DURATION_MS) {
            motorDriver.stop();
            currentState = STATE_FORWARD;
            drivingStartTime = now; // Reset timer when resuming
            Serial.println("[AVOID] Safe duration elapsed. Resuming forward path.");
        }
    } else if (currentState == STATE_MANUAL) {
        // Apply differential drive kinematics to compute left and right speeds
        float v_left = manualLinearX - (manualAngularZ * WHEEL_BASE_M) / 2.0f;
        float v_right = manualLinearX + (manualAngularZ * WHEEL_BASE_M) / 2.0f;

        // Convert to PWM duty cycle (0-255). Map max speed (0.5 m/s) to 255 PWM
        const float MAX_SPEED_MPS = 0.5f;
        int pwm_left = (int)(v_left * (255.0f / MAX_SPEED_MPS));
        int pwm_right = (int)(v_right * (255.0f / MAX_SPEED_MPS));

        // Constrain speeds to -255 to 255
        pwm_left = constrain(pwm_left, -255, 255);
        pwm_right = constrain(pwm_right, -255, 255);

        motorDriver.drive(pwm_left, pwm_right);
    } else if (currentState == STATE_SCANNING) {
        // Keep motors stopped during scanning
        motorDriver.stop();
    } else if (currentState == STATE_TURNING) {
        // Retrieve current heading from complementary filter
        float fx, fy, currentHeading;
        fusion.getFusedPose(fx, fy, currentHeading);
        
        // Integrate relative heading change
        float diff = currentHeading - lastHeading;
        if (diff > PI) diff -= 2 * PI;
        if (diff < -PI) diff += 2 * PI;
        
        headingTurned += diff;
        lastHeading = currentHeading;
        
        // Spin in place towards best target heading
        if (targetDeltaTheta > 0.0f) {
            // Turn Left
            motorDriver.drive(-140, 140);
        } else {
            // Turn Right
            motorDriver.drive(140, -140);
        }
        
        // Check if turning is complete
        if (abs(headingTurned) >= abs(targetDeltaTheta)) {
            motorDriver.stop();
            Serial.printf("[TURNING] Complete. Turned: %.3f rad (Target: %.3f rad). Resuming forward path.\n", headingTurned, targetDeltaTheta);
            currentState = STATE_FORWARD;
            drivingStartTime = now;
        }
    }
}
