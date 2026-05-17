#include "Motors.h"
#include "PinConfig.h"
#include "SharedGlobals.h"

const int pwmFreq = 20000;
const int pwmRes = 8;
const int chanL = 0;
const int chanR = 1;

void Motors::init() {
    pinMode(PIN_MOTOR_L_DIR, OUTPUT);
    pinMode(PIN_MOTOR_R_DIR, OUTPUT);

    // Setup Hardware PWM Channels
    ledcSetup(chanL, pwmFreq, pwmRes);
    ledcSetup(chanR, pwmFreq, pwmRes);
    
    ledcAttachPin(PIN_MOTOR_L_PWM, chanL);
    ledcAttachPin(PIN_MOTOR_R_PWM, chanR);

    setSpeeds(0, 0);
}

void Motors::setSpeeds(int left, int right) {
    // Left Motor Direction & PWM
    digitalWrite(PIN_MOTOR_L_DIR, left >= 0 ? HIGH : LOW);
    ledcWrite(chanL, abs(left));

    // Right Motor Direction & PWM
    digitalWrite(PIN_MOTOR_R_DIR, right >= 0 ? HIGH : LOW);
    ledcWrite(chanR, abs(right));

    // Update Globals for Telemetry
    portENTER_CRITICAL(&sensorMux);
    globalSensorData.motorL_Power = left;
    globalSensorData.motorR_Power = right;
    portEXIT_CRITICAL(&sensorMux);
}