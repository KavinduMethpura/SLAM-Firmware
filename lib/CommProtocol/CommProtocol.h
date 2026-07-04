#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "secrets.h"

class CommProtocol {
public:
    /**
     * @brief Constructor for the CommProtocol module.
     */
    CommProtocol();

    /**
     * @brief Connects to WiFi and initializes the UDP listener port.
     * Must be called in setup().
     */
    void begin();

    /**
     * @brief Non-blocking check for incoming UDP packets.
     * Reads packets and parses them.
     * Typically called at high frequency in the main loop.
     */
    void update();

    /**
     * @brief Checks if a new velocity command packet has arrived.
     * 
     * @return true if new velocity values are available.
     * @return false otherwise.
     */
    bool isNewCommandAvailable();

    /**
     * @brief Checks if a new system control command packet has arrived.
     * 
     * @return true if a new control command is available.
     * @return false otherwise.
     */
    bool isNewControlAvailable();

    /**
     * @brief Retrieves the latest system control command character and resets the flag.
     * 
     * @return The control command character ('R', 'C', 'S') or '\0' if none.
     */
    char getControlCommand();

    /**
     * @brief Retrieves the latest linear and angular velocities requested by the host,
     * and resets the new command flag.
     * 
     * @param outLinearX Outputs desired forward speed (m/s).
     * @param outAngularZ Outputs desired yaw rate (rad/s).
     */
    void getCommandVelocity(float &outLinearX, float &outAngularZ);

    /**
     * @brief Retrieves the latest calibration scales requested by the host,
     * and resets the new calibration command flag.
     * 
     * @param outLeft Outputs target left motor scale.
     * @param outRight Outputs target right motor scale.
     * @return true if a new calibration command was available.
     */
    bool getCalibrationCommand(float &outLeft, float &outRight);

    // =========================================================================
    // UPSTREAM SERIALIZATION METHODS (ESP32 -> Host)
    // =========================================================================
    
    /**
     * @brief Formats and transmits the current fused pose estimate over UDP.
     * Output format: $ODOM,x,y,theta,linear_vel,angular_vel\n
     */
    void sendOdom(float x, float y, float theta, float linearVel, float angularVel);

    /**
     * @brief Formats and transmits raw IMU data over UDP.
     * Output format: $IMUR,ax,ay,az,gx,gy,gz\n
     */
    void sendIMU(float ax, float ay, float az, float gx, float gy, float gz);

    /**
     * @brief Formats and transmits a single angle-range scan pair over UDP.
     * Output format: $SCAN,angle,distance_mm\n
     */
    void sendScan(int angle, int distanceMm);

    /**
     * @brief Transmits a raw debug/log message over UDP to the host.
     */
    void sendDebug(const char* message);

private:
    WiFiUDP udp;

    // Receive buffer variables
    char rxBuffer[64];
    int rxIndex;

    // Parsed target variables
    float cmdLinearX;
    float cmdAngularZ;
    bool newCommandFlag;
    
    char cmdControl;
    bool newControlFlag;
    
    float cmdCalLeft;
    float cmdCalRight;
    bool newCalFlag;

    /**
     * @brief Private helper to parse a complete message line once received.
     * 
     * @param line Null-terminated character string containing the message.
     */
    void parseLine(char* line);
};

#endif // COMM_PROTOCOL_H

