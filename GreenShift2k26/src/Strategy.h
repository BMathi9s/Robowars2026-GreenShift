#pragma once
#include "SharedGlobals.h"

class Strategy {
public:
    static void execute(SharedSensorData &snap, bool isPDControl, bool isDebugMotor,bool ispd_punch, bool isslow_reliable, bool isslow_attack);

private:
    static void runStandardProtocol(SharedSensorData &snap);
    static void runPDPUNCH(SharedSensorData &snap);
    // Update this line to accept the snapshot:
    static void runMotorDebugTest(SharedSensorData &snap); 
    static void slow_reliable(SharedSensorData &snap);
    static void slow_attack(SharedSensorData &snap);
};