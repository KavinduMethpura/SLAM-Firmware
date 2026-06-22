#include "IMUReader.h"

// MPU6050 Registers
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_START 0x3B
#define MPU6050_REG_GYRO_CONFIG 0x1B

IMUReader::IMUReader()
    : accX(0), accY(0), accZ(0),
      gyroX(0), gyroY(0), gyroZ(0),
      gyroBiasX(0), gyroBiasY(0), gyroBiasZ(0) {
}

bool IMUReader::begin() {
    // Initialize I2C Bus with pins defined in config.h
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    
    // =========================================================================
    // STEP 1: Wake up the MPU6050
    // =========================================================================
    // By default, the MPU6050 starts in sleep mode. 
    // You must write 0x00 to the Power Management 1 register (0x6B) to wake it up.
    // Try writing to register 0x6B. If it fails, call recoverI2C() and try again.

    // TODO: Write 0x00 to MPU6050_REG_PWR_MGMT_1.
    // Return true if successful, false if it fails twice.
    
    return true; // Replace when implemented
}

bool IMUReader::read() {
    // =========================================================================
    // STEP 2: Request 14 Bytes of Data from MPU6050 starting at register 0x3B
    // =========================================================================
    // 1. Begin transmission to MPU6050 address (ADDR_IMU_MPU6050).
    // 2. Write the address of the first data register (MPU6050_REG_ACCEL_START, which is 0x3B).
    // 3. End transmission (do not release bus if doing a restart, or just simple false).
    // 4. Request 14 bytes from ADDR_IMU_MPU6050:
    //    - Bytes 0-1: Accel X
    //    - Bytes 2-3: Accel Y
    //    - Bytes 4-5: Accel Z
    //    - Bytes 6-7: Temperature (ignored)
    //    - Bytes 8-9: Gyro X
    //    - Bytes 10-11: Gyro Y
    //    - Bytes 12-13: Gyro Z

    // TODO: Write register address and read 14 bytes into a buffer.
    
    // =========================================================================
    // STEP 3: Parse and Scale the Raw Values
    // =========================================================================
    // Combine high and low bytes for each channel (e.g. rawVal = (high << 8) | low).
    // Keep in mind rawVal is a signed 16-bit integer (int16_t).
    //
    // Scaling constants for default configuration (+/-2g and +/-250 deg/s):
    // - Accel: Divide raw value by 16384.0 to get 'g' units. 
    //   To get m/s^2, multiply 'g' units by 9.80665.
    // - Gyro: Divide raw value by 131.0 to get degrees/second.
    //   To get radians/second, multiply degrees/sec by (PI / 180.0).
    //
    // Remember to subtract gyro biases from the calibrated values!
    //   gyroX = gyroX_scaled - gyroBiasX;
    //   gyroY = gyroY_scaled - gyroBiasY;
    //   gyroZ = gyroZ_scaled - gyroBiasZ;

    // TODO: Implement parsing and scaling here.

    return true; // Replace when implemented
}

void IMUReader::calibrate(int samples) {
    float sumX = 0, sumY = 0, sumZ = 0;
    
    Serial.println("[IMU] Calibrating Gyroscope... Keep robot still.");
    delay(500);

    // =========================================================================
    // STEP 4: Gyroscope Calibration Loop
    // =========================================================================
    // Loop 'samples' times (e.g., 500 times):
    // 1. Read the IMU sensor.
    // 2. Sum the raw/unbiased gyroscope values.
    // 3. Wait a few milliseconds (e.g., delay(3)) between samples.
    // At the end, divide the sums by the number of samples to find the biases:
    //    gyroBiasX = sumX / samples
    //    gyroBiasY = sumY / samples
    //    gyroBiasZ = sumZ / samples

    // TODO: Implement calibration averaging loop here.
    
    Serial.print("[IMU] Calibration complete! Biases: ");
    Serial.print(gyroBiasX); Serial.print(", ");
    Serial.print(gyroBiasY); Serial.print(", ");
    Serial.println(gyroBiasZ);
}

void IMUReader::recoverI2C() {
    Serial.println("[I2C] Attempting bus recovery...");
    
    // =========================================================================
    // STEP 5: Manual I2C Bus Reset (Pin Toggling)
    // =========================================================================
    // If a slave device keeps SDA low (bus freeze), we must toggle the SCL clock manually.
    // 1. Set PIN_I2C_SDA as INPUT.
    // 2. Set PIN_I2C_SCL as OUTPUT.
    // 3. Check if SDA is LOW. If yes:
    //    - Loop 9 times:
    //      - Set PIN_I2C_SCL to LOW
    //      - Wait 5 microseconds (delayMicroseconds(5))
    //      - Set PIN_I2C_SCL to HIGH
    //      - Wait 5 microseconds
    // 4. Force a STOP condition:
    //    - Set PIN_I2C_SDA as OUTPUT, write LOW
    //    - Wait 5 microseconds
    //    - Set PIN_I2C_SCL as INPUT (tri-stated) or pull it HIGH
    //    - Wait 5 microseconds
    //    - Set PIN_I2C_SDA as INPUT
    // 5. Finally, shut down Wire and re-initialize it:
    //    - Wire.end()
    //    - Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)

    // TODO: Implement the toggling sequence here
    
    Serial.println("[I2C] Bus recovery sequence finished.");
}

float IMUReader::getGyroZ() const { return gyroZ; }
float IMUReader::getAccelX() const { return accX; }
float IMUReader::getAccelY() const { return accY; }

void IMUReader::getRawData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) const {
    ax = accX; ay = accY; az = accZ;
    gx = gyroX; gy = gyroY; gz = gyroZ;
}

bool IMUReader::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ADDR_IMU_MPU6050);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}
