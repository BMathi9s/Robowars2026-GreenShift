#include <Arduino.h>

// Motion Module Interface:
#include <EasyObjectDictionary.h>
#include <EasyProfile.h>

EasyObjectDictionary eOD;
EasyProfile          eP(&eOD);

HardwareSerial IMUSerial(1);

#define IMU_RX_PIN 18 
#define IMU_TX_PIN 17 

// --- GLOBAL SENSOR VARIABLES ---
// These are updated by Core 0 and read by Core 1
float latestRoll = 0, latestPitch = 0, latestYaw = 0;
float latestAccX = 0, latestAccY = 0, latestAccZ = 0;
float latestGyrZ = 0;

// --- MULTICORE SYNCHRONIZATION ---
// Spinlock to prevent Core 1 from reading data while Core 0 is writing it
portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool freshIMUData = false;
volatile int frameCounter = 0;       // Core 0 increments, Core 1 resets
unsigned long lastValidPacketTime = 0; 

// --- PRINTING & TRACKING (Core 1) ---
unsigned long lastPrintTime = 0;
unsigned long lastHzCalcTime = 0;
int actualHz = 0;

// Task Handle
TaskHandle_t IMUTaskHandle;

// Forward declaration of the FreeRTOS task
void IMU_Processing_Task(void * pvParameters);

void setup() {
  Serial.begin(115200); 
  
  IMUSerial.setRxBufferSize(2048);
  IMUSerial.begin(1000000, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  
  Serial.println("ESP32-S3 TM171 IMU - FreeRTOS Dual-Core Mode Started");

  // Spin up the IMU Task on Core 0
  xTaskCreatePinnedToCore(
      IMU_Processing_Task,   // Function to run
      "IMU_Task",            // Name of task
      4096,                  // Stack size in words (4KB is plenty)
      NULL,                  // Parameter to pass
      configMAX_PRIORITIES - 1, // High priority (runs above background tasks)
      &IMUTaskHandle,        // Task handle
      0);                    // Pin to Core 0
}

// ====================================================================
// CORE 0: IMU DEDICATED TASK
// Everything in here runs on its own isolated CPU core.
// ====================================================================
void IMU_Processing_Task(void * pvParameters) {
  for (;;) { // Infinite FreeRTOS loop
    
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
          
          // Lock memory briefly to safely update the globals
          portENTER_CRITICAL_ISR(&imuMux);
          
          switch (header.cmd) {                                  
              case EP_CMD_Raw_GYRO_ACC_MAG_:{
                  Ep_Raw_GyroAccMag raw;
                  if(EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&raw)){
                      latestAccX = raw.acc[0];
                      latestAccY = raw.acc[1];
                      latestAccZ = raw.acc[2];
                      latestGyrZ = raw.gyro[2]; 
                  }
              } break;

              case EP_CMD_RPY_:{
                  Ep_RPY rpy;
                  if(EP_SUCC_ == eOD.Read_Ep_RPY(&rpy)){     
                      latestRoll  = rpy.roll;
                      latestPitch = rpy.pitch;
                      latestYaw   = rpy.yaw;
                      frameCounter++;
                  }
              } break;
          } 
          
          freshIMUData = true; // Signal Core 1
          portEXIT_CRITICAL_ISR(&imuMux); // Release memory lock
          
          // Force library to process remaining data
          status = eP.On_RecvPkg(nullptr, 0, &header);
      } 
    } else {
      // If no bytes are available, yield Core 0 for 1 tick (~1ms) 
      // to prevent the FreeRTOS watchdog from crashing the ESP32.
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // --- CORE 0 WATCHDOG ---
    if (millis() - lastValidPacketTime > 1000) {
        while(IMUSerial.available()) {
            IMUSerial.read(); 
        }
        // Use a safe print from Core 0
        ets_printf("WARNING: IMU Stream lost! Core 0 flushing buffer...\n");
        lastValidPacketTime = millis(); 
    }
  }
}

// ====================================================================
// CORE 1: MAIN ARDUINO LOOP (PID & MOTOR CONTROL)
// ====================================================================
void loop() {
  
  // 1. SYNCHRONIZED CONTROL LOGIC
  if (freshIMUData) {
      // Create local variables to hold the safe snapshot of the physics data
      float myYaw, myAccX, myAccY;
      
      // Lock memory, grab copies of the data, reset flag, and unlock FAST.
      portENTER_CRITICAL(&imuMux);
      myYaw = latestYaw;
      myAccX = latestAccX;
      myAccY = latestAccY;
      freshIMUData = false;
      portEXIT_CRITICAL(&imuMux);
      
      // -> TODO: Run Wheel PID Calculations using myYaw
      // -> TODO: Run Stall detection using myAccX / myAccY
      // -> TODO: Command motor drivers
  }

  // 2. CALCULATE ACTUAL Hz (Once per second)
  if (millis() - lastHzCalcTime >= 1000) {
      portENTER_CRITICAL(&imuMux);
      actualHz = frameCounter;  
      frameCounter = 0;         
      portEXIT_CRITICAL(&imuMux);
      
      lastHzCalcTime = millis();
  }

  // 3. PRINT TO PC SLOWLY (20Hz)
  if (millis() - lastPrintTime >= 50) {
      lastPrintTime = millis();
      
      // Grab a quick safe snapshot just for printing
      float pYaw, pAccX, pAccY;
      portENTER_CRITICAL(&imuMux);
      pYaw = latestYaw; pAccX = latestAccX; pAccY = latestAccY;
      portEXIT_CRITICAL(&imuMux);

      Serial.print("[YAW] ");           Serial.print(pYaw, 1); 
      Serial.print("  |  [ACC] X: ");   Serial.print(pAccX, 2);
      Serial.print(" Y: ");             Serial.print(pAccY, 2);
      Serial.print("  |  [FREQ] ");     Serial.print(actualHz);
      Serial.println(" Hz");
  }
}