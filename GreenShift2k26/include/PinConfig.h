#pragma once

// ==========================================
// BATTERY & CURRENT SENSORS (ADC1) [cite: 7, 35]
// ==========================================
#define PIN_BAT_MONITOR 2
#define PIN_ACS724_1    1
#define PIN_ACS724_2    4

// ==========================================
// EDGE SENSORS (Micro ML2) - Digital IN [cite: 7, 38, 39]
// ==========================================
#define PIN_EDGE_FLL 14
#define PIN_EDGE_FL 13
#define PIN_EDGE_FR 12
#define PIN_EDGE_FRR 11

// ==========================================
// OPPONENT SENSORS (JS200XF) - Digital IN [cite: 7, 34]
// ==========================================
#define PIN_OPP_FL 10
#define PIN_OPP_FC 7
#define PIN_OPP_FR 5
#define PIN_OPP_REAR 6 

// ==========================================
// START MODULE (MicroStart) - Hardware ISR [cite: 7, 37]
// ==========================================
#define PIN_START_MODULE 15

// ==========================================
// UART COMMUNICATIONS [cite: 7, 31, 33]
// ==========================================
#define PIN_IMU_TX 17      // UART1 to TM171
#define PIN_IMU_RX 18      // UART1 to TM171
#define PIN_TELEM_TX 8     // UART2 to ESP32-C3
#define PIN_TELEM_RX 9     // UART2 to ESP32-C3

// ==========================================
// MOTOR DRIVERS (Cytron MD13S) [cite: 7, 43]
// ==========================================
#define PIN_MOTOR_L_PWM 40
#define PIN_MOTOR_L_DIR 39
#define PIN_MOTOR_R_PWM 41
#define PIN_MOTOR_R_DIR 42