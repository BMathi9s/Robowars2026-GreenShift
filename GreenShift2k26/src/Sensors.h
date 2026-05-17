#pragma once
#include <Arduino.h>

class Sensors {
public:
    static void init();

private:
    static void Sensor_Polling_Task(void * pvParameters);
};