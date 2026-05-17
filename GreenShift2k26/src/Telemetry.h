#pragma once
#include <Arduino.h>

class Telemetry {
public:
    static void init();

private:
    // The Core 0 task that formats and sends data to the ESP32-C3
    static void Telemetry_Task(void * pvParameters);
};