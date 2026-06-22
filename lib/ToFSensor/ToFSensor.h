#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>
#include <VL53L0X.h>
#include "config.h"

class ToFSensor {
public:
    /**
     * @brief Constructor for the ToF Sensor wrapper.
     */
    ToFSensor();

    /**
     * @brief Initializes the VL53L0X sensor and sets it to continuous reading mode.
     * Optionally handles pulling the XSHUT reset pin HIGH.
     * 
     * @return true if initialization succeeded, false if sensor was not found.
     */
    bool begin();

    /**
     * @brief Reads the current distance in millimeters from the sensor.
     * 
     * @return int Distance in millimeters, or -1 if a timeout/reading error occurs.
     */
    int readDistance();

private:
    VL53L0X loxSensor;
    
    // Internal variable to store last valid reading
    int lastDistance;
};

#endif // TOF_SENSOR_H
