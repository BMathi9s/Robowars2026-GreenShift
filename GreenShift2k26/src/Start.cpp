#include "SharedGlobals.h"
#include "PinConfig.h"

// ISR must be in IRAM for speed
void IRAM_ATTR startISR() {
    // High = Active, Low = Stop (Check your specific module logic)
    globalSensorData.matchActiveFlag = digitalRead(PIN_START_MODULE);
}

namespace Start {
    void init() {
        pinMode(PIN_START_MODULE, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PIN_START_MODULE), startISR, CHANGE);
        
        // Initial state check
        globalSensorData.matchActiveFlag = digitalRead(PIN_START_MODULE);
    }
}