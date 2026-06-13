#include <Arduino.h>
#include "driver/ledc.h" // package for pwm output

#define pwmChannel0 0
#define pwmChannel1 1
#define pwmPin0 1
#define pwmPin1 46
#define speed8bit 230 // speed scales linear from 0 to 256

void changeDirectionBuffer() {
  ledcWrite(pwmChannel0, 0);
  ledcWrite(pwmChannel1, 0);
  delay(500); // this is a reasonable delay for switching direction
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("ESP32 is ready!");

  ledcSetup(pwmChannel0, 1000, 8); // (pwmchannel to use,  frequency in Hz, number of bits)
  ledcAttachPin(pwmPin0,pwmChannel0);
  ledcWrite(pwmChannel0, 0);

  ledcSetup(pwmChannel1, 1000, 8); // (pwmchannel to use,  frequency in Hz, number of bits)
  ledcAttachPin(pwmPin1,pwmChannel1);
  ledcWrite(pwmChannel1, 0);
}

void loop() {

  changeDirectionBuffer();

  ledcWrite(pwmChannel0, speed8bit);
  Serial.println("on");
  delay(5000);

  changeDirectionBuffer();

  ledcWrite(pwmChannel1, speed8bit);
  Serial.println("switch");
  delay(5000);

  changeDirectionBuffer();
  Serial.println("low");

  delay(5000);

}
