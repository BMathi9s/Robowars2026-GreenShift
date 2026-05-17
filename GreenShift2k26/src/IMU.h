#pragma once
#include <Arduino.h>

class IMU {
public:
    static void init();

private:
    // The isolated FreeRTOS task function
    static void IMU_Processing_Task(void * pvParameters);
};