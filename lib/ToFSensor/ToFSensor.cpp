#include "ToFSensor.h"
#include <Wire.h>

ToFSensor::ToFSensor() : lastDistance(-1) {
}

bool ToFSensor::begin() {
    // Initialize I2C Bus with pins defined in config.h
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // =========================================================================
    // STEP 1: Initialize Pololu VL53L0X Library
    // =========================================================================
    // 1. Call loxSensor.init(). If it returns false, the sensor cannot be reached.
    if (!loxSensor.init()) {
        return false;
    }
    
    // 2. Set the sensor timeout (e.g., loxSensor.setTimeout(500)).
    loxSensor.setTimeout(500);
    
    // 3. Start continuous measurements.
    loxSensor.startContinuous();

    return true;
}

int ToFSensor::readDistance() {
    // =========================================================================
    // STEP 2: Read Distance in Millimeters
    // =========================================================================
    // 1. Call loxSensor.readRangeContinuousMillimeters() to get the latest reading.
    int distance = loxSensor.readRangeContinuousMillimeters();
    
    // 2. Check if a timeout occurred
    if (loxSensor.timeoutOccurred()) {
        return -1;
    }
    
    // 3. Filter out noise (poor signal returns like 8190/8191, or out of range)
    if (distance > 2000 || distance <= 0) {
        return -1;
    }

    lastDistance = distance;
    return distance;
}
