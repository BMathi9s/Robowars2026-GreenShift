#include <Arduino.h>
#include "SharedGlobals.h"
#include "IMU.h"
#include "Sensors.h"
#include "Telemetry.h"
#include "Motors.h"
#include "Strategy.h"

// --- CONFIGURATION FLAGS ---
const bool DEBUG_PRINT    = true;  
const bool DEBUG_MOTOR    = false;  
const bool PD_CONTROL = false;  //simple forward strong
const bool ispd_punch = true;   //main power
const bool slow_reliable = false; //main reaibale 
const bool slow_attack = false; //




namespace Start { void init(); }
namespace Edge { void init(); }

portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;
volatile SharedSensorData globalSensorData = {0};
unsigned long lastPrintTime = 0;

void setup() {
    if (DEBUG_PRINT) {
        Serial.begin(115200);
        delay(1000); 
        Serial.println("===========================================");
        Serial.println(" GREENSHIFT S3 - FULL DEBUG MODE ACTIVE ");
        Serial.println("===========================================");
    }

    IMU::init();       
    Sensors::init();   
    Telemetry::init(); 
    Motors::init();    
    Start::init();     
    Edge::init();      
}

void loop() {
    // 1. Thread-safe snapshot (Core 1) 
    SharedSensorData snap;
    portENTER_CRITICAL(&sensorMux);
    snap.copyFrom(globalSensorData);
    
    // Clear edge flags specifically after reading 
    globalSensorData.edgeFL1 = globalSensorData.edgeFL2 = false;
    globalSensorData.edgeFR1 = globalSensorData.edgeFR2 = false;
    globalSensorData.freshIMUData = false;
    portEXIT_CRITICAL(&sensorMux);

    // 2. Execute Strategy 
    bool runMotorTest = DEBUG_MOTOR && !PD_CONTROL;
    Strategy::execute(snap, PD_CONTROL, runMotorTest, ispd_punch, slow_reliable,slow_attack);

    // 3. Exhaustive Debug Print (20Hz)
    if (DEBUG_PRINT && (millis() - lastPrintTime >= 100)) {
        lastPrintTime = millis();
        
        // --- System & Motion Section ---
        Serial.printf("\n[SYS] Match:%d | Hz:%d | Bat:%4.2fV\n", 
                      snap.matchActiveFlag, snap.actualHz, snap.batteryVoltage);
        
        Serial.printf("[IMU] Yaw:%5.1f | Acc X:%5.2f Y:%5.2f Z:%5.2f\n", 
                      snap.yaw, snap.accX, snap.accY, snap.accZ);

        // --- Power Section --- 
        Serial.printf("[PWR] Amps L:%4.2fA | R:%4.2fA | Pwr L:%d R:%d\n", 
                      snap.currentL, snap.currentR, snap.motorL_Power, snap.motorR_Power);

        // --- Opponent Detection Section (JS200XF) --- [cite: 4, 34]
        Serial.printf("[OPP] FL:%d | FC:%d | FR:%d | RR:%d\n", 
                      snap.oppFL, snap.oppFC, snap.oppFR, snap.oppRear);

        // --- Edge Detection Section (Micro ML2) --- [cite: 4, 39]
        Serial.printf("[EDG] FL1:%d | FL2:%d | FR1:%d | FR2:%d\n", 
                      snap.edgeFL1, snap.edgeFL2, snap.edgeFR1, snap.edgeFR2);
        
        Serial.println("-------------------------------------------");
    }

    vTaskDelay(pdMS_TO_TICKS(1));
}