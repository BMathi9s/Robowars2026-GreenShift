#pragma once
#include <Arduino.h>

extern portMUX_TYPE sensorMux;

struct SharedSensorData {
    // IMU Data (Phase 1)
    float roll, pitch, yaw;
    float accX, accY, accZ;
    float gyrZ;
    bool freshIMUData; 
    int actualHz; 

    // Opponent Sensors (Phase 2) [cite: 4, 18]
    bool oppFL, oppFC, oppFR, oppRear;

    // Power & Current (Phase 2) [cite: 4, 35]
    float currentL, currentR;
    float batteryVoltage;

    // Interrupt Flags (Phase 3) [cite: 46, 73]
    volatile bool matchActiveFlag;
    
    // Four distinct edge flags for granular logic 
    volatile bool edgeFL1; 
    volatile bool edgeFL2;
    volatile bool edgeFR1;
    volatile bool edgeFR2;

    // Phase 5: Motor Status (For Telemetry)
    int motorL_Power; // -255 to 255
    int motorR_Power; // -255 to 255

    // Manual copy function to fix the 'volatile' compiler error
    void copyFrom(volatile SharedSensorData& src) {
        roll = src.roll; pitch = src.pitch; yaw = src.yaw;
        accX = src.accX; accY = src.accY; accZ = src.accZ;
        gyrZ = src.gyrZ;
        freshIMUData = src.freshIMUData;
        actualHz = src.actualHz;
        oppFL = src.oppFL; oppFC = src.oppFC; oppFR = src.oppFR; oppRear = src.oppRear;
        currentL = src.currentL; currentR = src.currentR;
        batteryVoltage = src.batteryVoltage;
        matchActiveFlag = src.matchActiveFlag;
        edgeFL1 = src.edgeFL1; edgeFL2 = src.edgeFL2;
        edgeFR1 = src.edgeFR1; edgeFR2 = src.edgeFR2;
        motorL_Power = src.motorL_Power;
        motorR_Power = src.motorR_Power;
    }
};

extern volatile SharedSensorData globalSensorData;