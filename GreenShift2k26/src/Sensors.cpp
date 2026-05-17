#include "Sensors.h"
#include "SharedGlobals.h"
#include "PinConfig.h"

TaskHandle_t SensorTaskHandle;

// ==========================================
// ADC CONVERSION CONSTANTS (Tune these)
// ==========================================
const float ADC_RESOLUTION = 4095.0;
const float ADC_REF_VOLTAGE = 3.3; 
const float ACS724_SENSITIVITY = 0.050;   // e.g., 50mV/A for a 40A sensor
const float ACS724_OFFSET = 1.65;         // VCC/2 if powered by 3.3V
const float BATTERY_DIVIDER_RATIO = 8.2; // e.g., 10k & 1k divider (11:1 ratio)

void Sensors::init() {
    // 1. Configure Digital Opponent Sensors (JS200XF)
    pinMode(PIN_OPP_FL, INPUT);
    pinMode(PIN_OPP_FC, INPUT);
    pinMode(PIN_OPP_FR, INPUT);
    pinMode(PIN_OPP_REAR, INPUT);

    // 2. Configure ADC Attenuation for 3.3V range
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db); // Allows reading up to ~3.3V safely

    // 3. Spin up the Polling Task on Core 0
    xTaskCreatePinnedToCore(
        Sensor_Polling_Task,  
        "Sensor_Task",            
        2048,                     // 2KB stack is plenty for basic polling
        NULL,                  
        configMAX_PRIORITIES - 2, // Slightly lower priority than the 1Mbps IMU task
        &SensorTaskHandle,        
        0                         // Pin to Core 0
    );
}

void Sensors::Sensor_Polling_Task(void * pvParameters) {
    for (;;) {
        // 1. Read Digital Pins (JS200XF)
        bool fl = digitalRead(PIN_OPP_FL);
        bool fc = digitalRead(PIN_OPP_FC);
        bool fr = digitalRead(PIN_OPP_FR);
        bool rear = digitalRead(PIN_OPP_REAR);

        // 2. Read Analog Pins (ACS724 & Battery)
        int rawBat = analogRead(PIN_BAT_MONITOR);
        int rawCur1 = analogRead(PIN_ACS724_1);
        int rawCur2 = analogRead(PIN_ACS724_2);

        // 3. Convert raw ADC values to physical units
        float batVolts = (rawBat / ADC_RESOLUTION) * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;
        
        float cur1Volts = (rawCur1 / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
        float ampsL = (cur1Volts - ACS724_OFFSET) / ACS724_SENSITIVITY;

        float cur2Volts = (rawCur2 / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
        float ampsR = (cur2Volts - ACS724_OFFSET) / ACS724_SENSITIVITY;

        // 4. Lock memory and update the shared struct safely
        portENTER_CRITICAL(&sensorMux);
        globalSensorData.oppFL = fl;
        globalSensorData.oppFC = fc;
        globalSensorData.oppFR = fr;
        globalSensorData.oppRear = rear;
        globalSensorData.batteryVoltage = batVolts;
        globalSensorData.currentL = ampsL;
        globalSensorData.currentR = ampsR;
        portEXIT_CRITICAL(&sensorMux);

        // 5. Run this loop at ~100Hz (10ms delay) to prevent CPU hogging
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}