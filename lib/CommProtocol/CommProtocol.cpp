#include "CommProtocol.h"

CommProtocol::CommProtocol() 
    : rxIndex(0), cmdLinearX(0.0f), cmdAngularZ(0.0f), newCommandFlag(false),
      cmdControl('\0'), newControlFlag(false) {
    memset(rxBuffer, 0, sizeof(rxBuffer));
}

void CommProtocol::begin() {
    Serial.begin(SERIAL_BAUD_RATE);
    
    // =========================================================================
    // STEP 1: Connect to WiFi Network
    // =========================================================================
    // 1. Start WiFi connection: WiFi.begin(WIFI_SSID, WIFI_PASSWORD)
    // 2. Loop while WiFi.status() != WL_CONNECTED:
    //    - Print "." to the Serial monitor
    //    - Wait 500 milliseconds (delay(500))
    // 3. Once connected, print the ESP32's local IP address (WiFi.localIP()).
    
    Serial.print("[WiFi] Connecting to ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
    Serial.print("[WiFi] Connected to ");
    Serial.println(WiFi.localIP());
    
    // =========================================================================
    // STEP 2: Initialize UDP Port Listener
    // =========================================================================
    // Call udp.begin(UDP_RECV_PORT) to start listening for incoming packets.
    // Verify it starts successfully.

    if (udp.begin(UDP_RECV_PORT)) {
        Serial.print("[UDP] Listening on port ");
        Serial.println(UDP_RECV_PORT);
    } else {
        Serial.println("[UDP] Failed to start listening");
    }
}

void CommProtocol::update() {
    // =========================================================================
    // STEP 3: Read Incoming UDP Packets (Non-Blocking)
    // =========================================================================
    // 1. Check if a packet is available: int packetSize = udp.parsePacket()
    // 2. If packetSize is greater than 0:
    //    - Read packet contents into rxBuffer:
    //      int len = udp.read(rxBuffer, sizeof(rxBuffer) - 1)
    //    - Null-terminate the buffer: rxBuffer[len] = '\0'
    //    - Call parser: parseLine(rxBuffer)
    
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        int len = udp.read(rxBuffer, sizeof(rxBuffer) - 1);
        rxBuffer[len] = '\0';
        parseLine(rxBuffer);
    }
}

void CommProtocol::parseLine(char* line) {
    if (strlen(line) == 0) return;

    // =========================================================================
    // STEP 4: Packet Parsing (Command Velocity "V" or Control "C")
    // =========================================================================
    // Same as Serial parsing:
    // 1. Check if line starts with 'V' (Velocity command):
    //    - Expected format: "V,linear_x,angular_z" (e.g. "V,0.2,-0.5")
    //    - Use sscanf() to extract floats: 
    //      sscanf(line, "V,%f,%f", &cmdLinearX, &cmdAngularZ)
    //    - If parsed successfully, set newCommandFlag = true.
    //
    // 2. Check if line starts with 'C' (System Control command):
    //    - Expected format: "C,cmd_type" (e.g. "C,R" to reset odometry)
    //    - Use sscanf(line, "C,%c", &type) to extract the command character.
    //    - Handle 'R' (Reset), 'C' (Calibrate), 'S' (Stop).

    if (line[0] == 'V') {
        sscanf(line, "V,%f,%f", &cmdLinearX, &cmdAngularZ);
        newCommandFlag = true;
    } else if (line[0] == 'C') {
        char type;
        sscanf(line, "C,%c", &type);
        if (type == 'R' || type == 'C' || type == 'S') {
            cmdControl = type;
            newControlFlag = true;
        }
    }
}

bool CommProtocol::isNewCommandAvailable() {
    return newCommandFlag;
}

bool CommProtocol::isNewControlAvailable() {
    return newControlFlag;
}

char CommProtocol::getControlCommand() {
    newControlFlag = false;
    return cmdControl;
}

void CommProtocol::getCommandVelocity(float &outLinearX, float &outAngularZ) {
    outLinearX = cmdLinearX;
    outAngularZ = cmdAngularZ;
    newCommandFlag = false; // Clear flag on read
}

void CommProtocol::sendOdom(float x, float y, float theta, float linearVel, float angularVel) {
    // =========================================================================
    // STEP 5: Format and Transmit Odometry Telemetry over UDP
    // =========================================================================
    // 1. Format the data string into a buffer using snprintf:
    //    char txBuffer[128];
    //    snprintf(txBuffer, sizeof(txBuffer), "$ODOM,%.3f,%.3f,%.3f,%.3f,%.3f\n", x, y, theta, linearVel, angularVel);
    // 2. Start UDP packet: udp.beginPacket(HOST_IP, UDP_SEND_PORT)
    // 3. Write data: udp.write((uint8_t*)txBuffer, strlen(txBuffer))
    // 4. Send packet: udp.endPacket()
    
    char txBuffer[128];
    snprintf(txBuffer, sizeof(txBuffer), "$ODOM,%.3f,%.3f,%.3f,%.3f,%.3f\n", x, y, theta, linearVel, angularVel);
    udp.beginPacket(HOST_IP, UDP_SEND_PORT);
    udp.write((uint8_t*)txBuffer, strlen(txBuffer));
    udp.endPacket();
}

void CommProtocol::sendIMU(float ax, float ay, float az, float gx, float gy, float gz) {
    // =========================================================================
    // STEP 6: Format and Transmit IMU Telemetry over UDP
    // =========================================================================
    // Format: "$IMUR,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n"
    
    char txBuffer[128];
    snprintf(txBuffer, sizeof(txBuffer), "$IMUR,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", ax, ay, az, gx, gy, gz);
    udp.beginPacket(HOST_IP, UDP_SEND_PORT);
    udp.write((uint8_t*)txBuffer, strlen(txBuffer));
    udp.endPacket();
}

void CommProtocol::sendScan(int angle, int distanceMm) {
    // =========================================================================
    // STEP 7: Format and Transmit Scanner Sweep Telemetry over UDP
    // =========================================================================
    // Format: "$SCAN,%d,%d\n"
    
    char txBuffer[128];
    snprintf(txBuffer, sizeof(txBuffer), "$SCAN,%d,%d\n", angle, distanceMm);
    udp.beginPacket(HOST_IP, UDP_SEND_PORT);
    udp.write((uint8_t*)txBuffer, strlen(txBuffer));
    udp.endPacket();
}

