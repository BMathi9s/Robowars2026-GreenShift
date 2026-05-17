#include <Arduino.h>

const int pwmPin = 40; 
const int dirPin = 39; 
//41 42 good
// 39 40 not good
// 40 39 good!

// PWM Settings
const int pwmFreq = 20000;    // 20kHz - Silent to human ears!
const int pwmResolution = 8;  // 8-bit resolution (0 to 255)
const int pwmChannel = 0;     // ESP32 v2 API requires a specific LEDC channel (0-15)

void setup() {
  Serial.begin(115200);
  
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW); 

  // ==========================================
  // ESP32 Hardware PWM Setup (LEDC v2.x API)
  // ==========================================
  // 1. Configure the PWM channel with our frequency and resolution
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  
  // 2. Attach our motor PWM pin to that specific channel
  ledcAttachPin(pwmPin, pwmChannel); 
  
  // 3. Initialize to 0 speed (write to the CHANNEL, not the pin)
  ledcWrite(pwmChannel, 0); 
  
  delay(2000); 
  Serial.println("Silent 20kHz Sequence Starting!");
}

void loop() {
  digitalWrite(dirPin, HIGH); 
  
  Serial.println("Ramping Up...");
  for (int speed = 0; speed <= 255; speed++) {
    // Write speed to the channel
    ledcWrite(pwmChannel, speed);
    delay(15); 
  }
  
  delay(5000); 

  Serial.println("Ramping Down...");
  for (int speed = 255; speed >= 0; speed--) {
    ledcWrite(pwmChannel, speed);
    delay(15); 
  }

  delay(1000);

  Serial.println("Firing Pulses...");
  for (int pulse = 0; pulse < 3; pulse++) {
    ledcWrite(pwmChannel, 255); 
    delay(150);               
    
    ledcWrite(pwmChannel, 0);   
    delay(250);               
  }

  Serial.println("Sequence Complete. Restarting...");
  delay(2000);


    digitalWrite(dirPin, LOW); 
  
  Serial.println("Ramping Up...");
  for (int speed = 0; speed <= 255; speed++) {
    // Write speed to the channel
    ledcWrite(pwmChannel, speed);
    delay(15); 
  }
  
  delay(1000); 

  Serial.println("Ramping Down...");
  for (int speed = 255; speed >= 0; speed--) {
    ledcWrite(pwmChannel, speed);
    delay(15); 
  }

  delay(1000);

  Serial.println("Firing Pulses...");
  for (int pulse = 0; pulse < 3; pulse++) {
    ledcWrite(pwmChannel, 255); 
    delay(150);               
    
    ledcWrite(pwmChannel, 0);   
    delay(250);               
  }

  Serial.println("Sequence Complete. Restarting...");
  delay(2000);



}