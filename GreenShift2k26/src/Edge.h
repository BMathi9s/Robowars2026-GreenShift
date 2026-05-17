#pragma once
#include <Arduino.h>

class EdgeSensors {
public:
    static void init();
    // Hardware Interrupt handlers for zero-latency avoidance [cite: 20]
    static void IRAM_ATTR frontLeftISR();
    static void IRAM_ATTR frontRightISR();
    static void IRAM_ATTR backLeftISR();
    static void IRAM_ATTR backRightISR();
};