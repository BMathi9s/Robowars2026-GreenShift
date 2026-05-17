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

void setup() {
  Serial.begin(115200); 
  IMUSerial.setRxBufferSize(2048);
  IMUSerial.begin(1000000, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  
  Serial.println("ESP32-S3 TM171 IMU - Watchdog & Low Latency Mode Started");
}

void loop() {
  
  // ---------------------------------------------------------
  // TASK 1: READ THE ENTIRE BUFFER (No packet tearing!)
  // ---------------------------------------------------------
  // We explicitly capture how many bytes are waiting right NOW,
  // and process exactly that many. This guarantees we don't get trapped
  // in an infinite loop if the 1Mbps stream outpaces the processor.
  int bytesToProcess = IMUSerial.available();
  
  while (bytesToProcess > 0) {
    char rxByte = (char)IMUSerial.read();                      
    char* rxData = &rxByte;                                   
    int rxSize = 1;                                         
    Ep_Header header;

    if(EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header)){    
        
        lastValidPacketTime = millis(); // WE GOT A GOOD PACKET! Pet the watchdog.

        switch (header.cmd) {                                  
        
        case EP_CMD_Raw_GYRO_ACC_MAG_:{
            Ep_Raw_GyroAccMag raw;
            if(EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&raw)){
                latestAccX = raw.acc[0];
                latestAccY = raw.acc[1];
                latestAccZ = raw.acc[2];
                latestGyrZ = raw.gyro[2]; 
            }
        }break;

        case EP_CMD_RPY_:{
            Ep_RPY rpy;
            if(EP_SUCC_ == eOD.Read_Ep_RPY(&rpy)){     
                latestRoll  = rpy.roll;
                latestPitch = rpy.pitch;
                latestYaw   = rpy.yaw;
            }
        }break;
        
        } // End Switch
    } // End if complete package
    
    bytesToProcess--; 
  }

  // ---------------------------------------------------------
  // TASK 2: THE SOFTWARE WATCHDOG (Auto-recovery)
  // ---------------------------------------------------------
  if (millis() - lastValidPacketTime > 1000) {
      // If 1 second passes without a clean packet, the IMU likely rebooted
      // or the wire jiggled. The buffer is full of garbage framing.
      // Dump the entire buffer into the void to aggressively re-sync.
      while(IMUSerial.available()) {
          IMUSerial.read(); 
      }
      Serial.println("WARNING: IMU Stream lost or corrupted! Flushing buffer to re-sync...");
      lastValidPacketTime = millis(); // Reset timer so it doesn't spam
  }


  // ---------------------------------------------------------
  // TASK 3: PRINT TO PC SLOWLY (20Hz)
  // ---------------------------------------------------------
  if (millis() - lastPrintTime >= 50) {
      lastPrintTime = millis();
      
      Serial.print("[RPY] R: "); Serial.print(latestRoll, 1); 
      Serial.print(" P: ");      Serial.print(latestPitch, 1); 
      Serial.print(" Y: ");      Serial.print(latestYaw, 1); 
      
      Serial.print("   |   [ACC] X: "); Serial.print(latestAccX, 2);
      Serial.print(" Y: ");             Serial.print(latestAccY, 2);
      Serial.print(" Z: ");             Serial.println(latestAccZ, 2);
  }
}