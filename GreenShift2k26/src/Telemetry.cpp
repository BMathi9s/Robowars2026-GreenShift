#include "Telemetry.h"
#include "SharedGlobals.h"
#include "PinConfig.h"

HardwareSerial TelemetrySerial(2); // UART2 [cite: 24]
TaskHandle_t TelemetryTaskHandle;

void Telemetry::init() {
    // UART2 mapped to GPIO 8 (TX) and 9 (RX) 
    TelemetrySerial.begin(115200, SERIAL_8N1, PIN_TELEM_RX, PIN_TELEM_TX); 
    
    xTaskCreatePinnedToCore(
        Telemetry_Task,
        "Telem_Task",
        4096,
        NULL,
        configMAX_PRIORITIES - 3, // Lower priority than IMU/Sensors
        &TelemetryTaskHandle,
        0                         // Pinned to Core 0 (Nervous System) [cite: 30]
    );
}

void Telemetry::Telemetry_Task(void * pvParameters) {
    for (;;) {
        // 1. Take a safe snapshot of the current robot state
        SharedSensorData snap;
        portENTER_CRITICAL(&sensorMux);
        snap.copyFrom(globalSensorData);
        portEXIT_CRITICAL(&sensorMux);

        // 2. Format a telemetry packet for the C3 coprocessor
        // Using CSV format: START,Yaw,AccX,AccY,AccZ,Bat,CurL,CurR,MatchActive,EdgeFlags,END
        TelemetrySerial.printf("START,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d%d%d%d,END\n",
            snap.yaw, snap.accX, snap.accY, snap.accZ,
            snap.batteryVoltage, snap.currentL, snap.currentR,
            snap.matchActiveFlag,
            snap.edgeFL1, snap.edgeFL2, snap.edgeFR1, snap.edgeFR2
        );

        // 3. Send at 20Hz (50ms) - Sufficient for BLE/Screen updates
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}