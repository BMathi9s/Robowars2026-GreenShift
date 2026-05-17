#include <Arduino.h>

// Motion Module Interface:
#include <EasyObjectDictionary.h>
#include <EasyProfile.h>

EasyObjectDictionary eOD;
EasyProfile          eP(&eOD);

HardwareSerial IMUSerial(1);

#define IMU_RX_PIN 18 
#define IMU_TX_PIN 17 

// --- GLOBAL VARIABLES ---
float latestRoll = 0, latestPitch = 0, latestYaw = 0;
float latestAccX = 0, latestAccY = 0, latestAccZ = 0;
float latestGyrZ = 0;

unsigned long lastPrintTime = 0;
unsigned long lastValidPacketTime = 0; // Watchdog timer

// --- FREQUENCY TRACKING VARIABLES ---
unsigned long lastHzCalcTime = 0;
int frameCounter = 0;
int actualHz = 0;

// Sync flag for motor control
volatile bool freshIMUData = false;

void setup() {
  Serial.begin(115200); 
  
  // 2048 is a good buffer for 1M baud, gives the ESP32 breathing room
  IMUSerial.setRxBufferSize(2048);
  IMUSerial.begin(1000000, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  
  Serial.println("ESP32-S3 TM171 IMU - Optimized Mode with Hz Tracking");
}

void loop() {
  
  // ---------------------------------------------------------
  // TASK 1: READ CHUNKS & DRAIN INTERNAL QUEUE
  // ---------------------------------------------------------
  int availableBytes = IMUSerial.available();
  
  if (availableBytes > 0) {
    uint8_t rxBuffer[256]; 
    
    // Constrain the read size to our local buffer to prevent overflows
    int bytesToRead = min(availableBytes, (int)sizeof(rxBuffer));
    
    // Hardware-optimized block read
    IMUSerial.readBytes(rxBuffer, bytesToRead);
    
    Ep_Header header;
    
    // Feed the chunk to the parser
    int status = eP.On_RecvPkg((char*)rxBuffer, bytesToRead, &header);

    // DRAIN LOOP: Process everything inside the EasyProtocol buffer
    while (status == EP_SUCC_) {
        lastValidPacketTime = millis(); 
        freshIMUData = true; // Signal the PID loop

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
                    
                    // Count RPY as a complete "frame" of data
                    frameCounter++;
                }
            } break;
        } 
        
        // Feed 0 bytes to force the library to process any remaining data in its queue
        status = eP.On_RecvPkg(nullptr, 0, &header);
    } 
  }

  // ---------------------------------------------------------
  // TASK 2: SYNCHRONIZED CONTROL LOGIC (PID & Motors)
  // ---------------------------------------------------------
  if (freshIMUData) {
      freshIMUData = false;
      
      // TODO: Wheel PID Calculations (Yaw control)
      // TODO: Stall detection (AccX/Y spikes or thresholds)
      // TODO: Send UART telemetry / PWM commands to motor drivers
  }

  // ---------------------------------------------------------
  // TASK 3: CALCULATE ACTUAL Hz (Once per second)
  // ---------------------------------------------------------
  if (millis() - lastHzCalcTime >= 1000) {
      actualHz = frameCounter;  // Store the count for printing
      frameCounter = 0;         // Reset for the next second
      lastHzCalcTime = millis();
  }

  // ---------------------------------------------------------
  // TASK 4: THE SOFTWARE WATCHDOG (Auto-recovery)
  // ---------------------------------------------------------
  if (millis() - lastValidPacketTime > 1000) {
      while(IMUSerial.available()) {
          IMUSerial.read(); 
      }
      Serial.println("WARNING: IMU Stream lost! Flushing buffer to re-sync...");
      lastValidPacketTime = millis(); 
  }

  // ---------------------------------------------------------
  // TASK 5: PRINT TO PC SLOWLY (20Hz)
  // ---------------------------------------------------------
  if (millis() - lastPrintTime >= 50) {
      lastPrintTime = millis();
      
      Serial.print("[YAW] ");           Serial.print(latestYaw, 1); 
      Serial.print("  |  [ACC] X: ");   Serial.print(latestAccX, 2);
      Serial.print(" Y: ");             Serial.print(latestAccY, 2);
      Serial.print("  |  [FREQ] ");     Serial.print(actualHz);
      Serial.println(" Hz");
  }
}