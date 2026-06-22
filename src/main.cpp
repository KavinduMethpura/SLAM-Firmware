#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "ServoScanner.h"
#include "EncoderOdometry.h"
#include "IMUReader.h"
#include "ToFSensor.h"
#include "SensorFusion.h"
#include "CommProtocol.h"

// =============================================================================
// GLOBAL OBJECTS & STATE
// =============================================================================
MotorDriver motorDriver;
ServoScanner servoScanner;
EncoderOdometry odometry;
IMUReader imu;
ToFSensor tof;
SensorFusion fusion;
CommProtocol comm;

enum SystemState {
    STATE_INIT,
    STATE_STANDBY,
    STATE_DRIVING,
    STATE_SAFE_STOP
};

SystemState currentState = STATE_INIT;

// Watchdog / Connection Safety Variables
unsigned long lastCommTime = 0;
const unsigned long COMM_TIMEOUT_MS = 2000; // Stop motors if communication is lost for 2s

// Scheduler Timing Variables
unsigned long lastTime50Hz = 0;
unsigned long lastTime20Hz = 0;
unsigned long lastTime10Hz = 0;
unsigned long lastTime1Hz  = 0;

// High-precision delta-t tracking
unsigned long prevMicroseconds = 0;

// Target motion command from host (linear x in m/s, angular z in rad/s)
float targetLinearVel = 0.0f;
float targetAngularVel = 0.0f;

// =============================================================================
// HELPER FUNCTIONS & KINEMATICS
// =============================================================================

/**
 * @brief Splits target linear/angular velocities into individual left and right wheel speeds,
 * converts them to PWM levels, and commands the Motor Driver.
 * 
 * Kinematics Formula (Differential Drive):
 * - Left Wheel Velocity:  v_left  = linear_x - (angular_z * WHEEL_BASE_M) / 2.0
 * - Right Wheel Velocity: v_right = linear_x + (angular_z * WHEEL_BASE_M) / 2.0
 * 
 * Conversion to PWM:
 * - Define a scaling factor or mapping from meters/second to PWM duty cycle (0-255).
 * - For a basic feed-forward approach: pwm_value = wheel_velocity * (255.0 / max_speed_mps)
 * - For closed-loop control: implement a PI or PID controller adjusting PWM based on
 *   encoder feedback speeds vs target speeds.
 */
void setRobotVelocity(float linearX, float angularZ) {
    // If we are in safe stop mode, force zero speed
    if (currentState == STATE_SAFE_STOP || currentState == STATE_STANDBY) {
        motorDriver.stop();
        return;
    }

    // TODO: 1. Apply differential drive kinematics to compute v_left and v_right.
    // TODO: 2. Convert wheel velocities (m/s) to PWM duty cycles (e.g. -255 to 255).
    // TODO: 3. Call motorDriver.drive(pwm_left, pwm_right).
}

// =============================================================================
// MAIN INITIALIZATION
// =============================================================================
void setup() {
    // 1. Initialize Communication
    comm.begin();
    
    // 2. Initialize Motor Driver
    motorDriver.begin();
    motorDriver.stop();
    
    // 3. Initialize Sensors (I2C)
    Serial.println("[SYSTEM] Initializing I2C sensors...");
    if (!imu.begin()) {
        Serial.println("[SYSTEM] ERROR: Failed to detect MPU6050!");
    } else {
        // Calibrate gyroscope offsets on start (requires robot to be stationary)
        imu.calibrate();
    }
    
    if (!tof.begin()) {
        Serial.println("[SYSTEM] ERROR: Failed to detect VL53L0X!");
    }

    // 4. Initialize Servo Scanner
    servoScanner.begin();

    // 5. Initialize Encoders and Odometry
    odometry.begin();
    fusion.begin();

    // 6. Set system state and timestamps
    currentState = STATE_STANDBY;
    lastCommTime = millis();
    unsigned long now = millis();
    lastTime50Hz = now;
    lastTime20Hz = now;
    lastTime10Hz = now;
    lastTime1Hz  = now;
    prevMicroseconds = micros();

    // =========================================================================
    // MOTOR TEST SEQUENCE (Runs once at boot to test wiring and driver logic)
    // =========================================================================
    Serial.println("[TEST] Starting motor driver verification sequence...");
    
    // 1. Move Forward
    Serial.println("[TEST] Moving forward...");
    motorDriver.drive(150, 150);
    delay(2000);
    
    // 2. Move Backward
    Serial.println("[TEST] Moving backward...");
    motorDriver.drive(-150, -150);
    delay(2000);
    
    // 3. Turn Left (Differential Pivot)
    Serial.println("[TEST] Turning left...");
    motorDriver.drive(-150, 150);
    delay(2000);
    
    // 4. Turn Right (Differential Pivot)
    Serial.println("[TEST] Turning right...");
    motorDriver.drive(150, -150);
    delay(2000);
    
    // 5. Stop
    Serial.println("[TEST] Stopping motors.");
    motorDriver.stop();
    delay(1000);

    Serial.println("[SYSTEM] Setup completed. Robot is in STANDBY state.");
}

// =============================================================================
// MAIN EXECUTION LOOP
// =============================================================================
void loop() {
    unsigned long currentTimeMs = millis();
    unsigned long currentTimeUs = micros();

    // Calculate microsecond-accurate time step (dt) for math integration
    float dt = (float)(currentTimeUs - prevMicroseconds) / 1000000.0f;
    prevMicroseconds = currentTimeUs;
    
    // Safety check: protect against micros() rollover (happens every ~70 min)
    if (dt < 0.0f || dt > 0.5f) {
        dt = 0.02f; // Fallback to default 50Hz step size
    }

    // 1. Check for incoming Serial commands from Host
    comm.update();

    if (comm.isNewCommandAvailable()) {
        comm.getCommandVelocity(targetLinearVel, targetAngularVel);
        lastCommTime = currentTimeMs; // Reset watchdog timer
        
        // If we were in Standby or Safe Stop, transition to driving
        if (currentState == STATE_STANDBY || currentState == STATE_SAFE_STOP) {
            currentState = STATE_DRIVING;
            Serial.println("[SYSTEM] Command received. Transitioning to DRIVING.");
        }
    }

    // =========================================================================
    // LOOP A: 50 Hz Task (Sensor Fusion & Control - every 20ms)
    // =========================================================================
    if (currentTimeMs - lastTime50Hz >= 20) {
        lastTime50Hz = currentTimeMs;

        // 1. Read IMU gyroscope and accelerometers
        imu.read();
        float gyroZ = imu.getGyroZ();

        // 2. Read Encoders & update raw Odometry coordinates
        odometry.update(0.020f); // 20ms fixed timestamp or compute dynamic dt
        
        float encX, encY, encTheta;
        odometry.getPose(encX, encY, encTheta);

        // 3. Fused sensor inputs
        fusion.update(encX, encY, encTheta, gyroZ, 0.020f);

        // 4. Update physical motor drive outputs using kinematics
        setRobotVelocity(targetLinearVel, targetAngularVel);
    }

    // =========================================================================
    // LOOP B: 20 Hz Task (MG90S Servo Scanner & VL53L0X ToF - every 50ms)
    // =========================================================================
    if (currentTimeMs - lastTime20Hz >= 50) {
        lastTime20Hz = currentTimeMs;

        // 1. Step the Servo
        // update() returns true if the servo has reached a new angle and stopped
        if (servoScanner.update()) {
            // 2. Trigger ToF laser ranging reading
            int dist = tof.readDistance();
            int angle = servoScanner.getAngle();
            
            // 3. Send the scan point pair immediately to the host computer
            comm.sendScan(angle, dist);
        }
    }

    // =========================================================================
    // LOOP C: 10 Hz Task (Telemetry Logging - every 100ms)
    // =========================================================================
    if (currentTimeMs - lastTime10Hz >= 100) {
        lastTime10Hz = currentTimeMs;

        // 1. Get fused pose estimate
        float fx, fy, ftheta;
        fusion.getFusedPose(fx, fy, ftheta);

        // 2. Get velocities
        float linVel, angVel;
        odometry.getVelocities(linVel, angVel);

        // 3. Transmit Fused Odometry Packet
        comm.sendOdom(fx, fy, ftheta, linVel, angVel);

        // 4. Transmit Raw IMU Packet
        float ax, ay, az, gx, gy, gz;
        imu.getRawData(ax, ay, az, gx, gy, gz);
        comm.sendIMU(ax, ay, az, gx, gy, gz);
    }

    // =========================================================================
    // LOOP D: 1 Hz Task (Safety watchdog & Health diagnostics - every 1000ms)
    // =========================================================================
    if (currentTimeMs - lastTime1Hz >= 1000) {
        lastTime1Hz = currentTimeMs;

        // 1. Watchdog timeout check: If driving but no message from host
        if (currentState == STATE_DRIVING && (currentTimeMs - lastCommTime > COMM_TIMEOUT_MS)) {
            currentState = STATE_SAFE_STOP;
            targetLinearVel = 0.0f;
            targetAngularVel = 0.0f;
            motorDriver.stop();
            Serial.println("[SAFETY] WATCHDOG TIMEOUT: No host command. Entering SAFE_STOP.");
        }
        
        // 2. Add I2C diagnostics check or status prints if needed
    }
}
