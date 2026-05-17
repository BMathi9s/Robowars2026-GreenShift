#include "IMU.h"
#include "SharedGlobals.h"
#include "PinConfig.h" // Ensures we use PIN_IMU_RX (18) and PIN_IMU_TX (17)

#include <EasyObjectDictionary.h>
#include <EasyProfile.h>

// Module-level variables hidden from the rest of the program
EasyObjectDictionary eOD;
EasyProfile eP(&eOD);
HardwareSerial IMUSerial(1);
TaskHandle_t IMUTaskHandle;

void IMU::init() {
    IMUSerial.setRxBufferSize(2048);
    // Uses the pins defined in PinConfig.h
    IMUSerial.begin(1000000, SERIAL_8N1, PIN_IMU_RX, PIN_IMU_TX); 

    // Spin up the IMU Task on Core 0
    xTaskCreatePinnedToCore(
        IMU_Processing_Task,  
        "IMU_Task",            
        4096,                  
        NULL,                  
        configMAX_PRIORITIES - 1, 
        &IMUTaskHandle,        
        0                      // Pin to Core 0
    );
}

void IMU::IMU_Processing_Task(void * pvParameters) {
    volatile int frameCounter = 0;
    unsigned long lastValidPacketTime = millis();
    unsigned long lastHzCalcTime = millis();

    for (;;) { 
        int availableBytes = IMUSerial.available();
        
        if (availableBytes > 0) {
            uint8_t rxBuffer[256]; 
            int bytesToRead = min(availableBytes, (int)sizeof(rxBuffer));
            IMUSerial.readBytes(rxBuffer, bytesToRead);
            
            Ep_Header header;
            int status = eP.On_RecvPkg((char*)rxBuffer, bytesToRead, &header);

            // Drain the internal queue
            while (status == EP_SUCC_) {
                lastValidPacketTime = millis(); 
                
                // Lock memory to safely update the globals
                portENTER_CRITICAL_ISR(&sensorMux);
                
                switch (header.cmd) {                                  
                    case EP_CMD_Raw_GYRO_ACC_MAG_:{
                        Ep_Raw_GyroAccMag raw;
                        if(EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&raw)){
                            globalSensorData.accX = raw.acc[0];
                            globalSensorData.accY = raw.acc[1];
                            globalSensorData.accZ = raw.acc[2];
                            globalSensorData.gyrZ = raw.gyro[2]; 
                        }
                    } break;

                    case EP_CMD_RPY_:{
                        Ep_RPY rpy;
                        if(EP_SUCC_ == eOD.Read_Ep_RPY(&rpy)){     
                            globalSensorData.roll  = rpy.roll;
                            globalSensorData.pitch = rpy.pitch;
                            globalSensorData.yaw   = rpy.yaw;
                            frameCounter++;
                        }
                    } break;
                } 
                
                globalSensorData.freshIMUData = true; // Signal Core 1
                portEXIT_CRITICAL_ISR(&sensorMux);    // Release memory lock
                
                // Force library to process remaining data
                status = eP.On_RecvPkg(nullptr, 0, &header);
            } 
        } else {
            // Yield Core 0 for 1 tick to prevent FreeRTOS watchdog crashes
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        // Calculate actual Hz once per second internally on Core 0
        if (millis() - lastHzCalcTime >= 1000) {
            portENTER_CRITICAL(&sensorMux);
            globalSensorData.actualHz = frameCounter;  
            frameCounter = 0;          
            portEXIT_CRITICAL(&sensorMux);
            lastHzCalcTime = millis();
        }

        // Core 0 Watchdog
        if (millis() - lastValidPacketTime > 1000) {
            while(IMUSerial.available()) {
                IMUSerial.read(); 
            }
            ets_printf("WARNING: IMU Stream lost! Core 0 flushing buffer...\n");
            lastValidPacketTime = millis(); 
        }
    }
}