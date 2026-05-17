#include "SharedGlobals.h"
#include "PinConfig.h"

// Individual ISRs for ultra-granular detection [cite: 20]
void IRAM_ATTR edgeFL1_ISR() { globalSensorData.edgeFL1 = true; }
void IRAM_ATTR edgeFL2_ISR() { globalSensorData.edgeFL2 = true; }
void IRAM_ATTR edgeFR1_ISR() { globalSensorData.edgeFR1 = true; }
void IRAM_ATTR edgeFR2_ISR() { globalSensorData.edgeFR2 = true; }

namespace Edge {
    void init() {
        // Mapping GPIO 5, 6, 7, 10 to our 4 flags [cite: 7]
        pinMode(PIN_EDGE_FLL, INPUT);
        pinMode(PIN_EDGE_FL, INPUT);
        pinMode(PIN_EDGE_FR, INPUT);
        pinMode(PIN_EDGE_FRR, INPUT);
        
        attachInterrupt(digitalPinToInterrupt(PIN_EDGE_FLL), edgeFL1_ISR, RISING);
        attachInterrupt(digitalPinToInterrupt(PIN_EDGE_FL), edgeFL2_ISR, RISING);
        attachInterrupt(digitalPinToInterrupt(PIN_EDGE_FR), edgeFR1_ISR, RISING);
        attachInterrupt(digitalPinToInterrupt(PIN_EDGE_FRR), edgeFR2_ISR, RISING);
    }
}