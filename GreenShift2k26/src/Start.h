#pragma once
#include <Arduino.h>

class StartModule {
public:
    static void init();
    // Hardware ISR to instantly toggle match active/inactive [cite: 19]
    static void IRAM_ATTR isrHandler(); 
};