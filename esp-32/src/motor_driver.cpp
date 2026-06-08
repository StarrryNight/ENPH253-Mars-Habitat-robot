#include <cstdint>

#include "constants.h"
#include "motor_driver.h"


void moveForward(int8_t	speed) {
  changeDirectionBuffer();
  ledcWrite(LEFT_PWM_CHANNEL_0, speed);
  ledcWrite(RIGHT_PWM_CHANNEL_0, speed);
  current_state = MotorState
}
void moveBackward(int8_t 	speed) {
  changeDirectionBuffer();
  ledcWrite(LEFT_PWM_CHANNEL_1, speed);
  ledcWrite(RIGHT_PWM_CHANNEL_1, speed);
}
void rotateClockWise(int8_t	speed) {

  changeDirectionBuffer();
  ledcWrite(LEFT_PWM_CHANNEL_0, speed);
  ledcWrite(RIGHT_PWM_CHANNEL_1, speed);
}
void rotateCounterClockwise(int8_t	speed) {
  changeDirectionBuffer();
  ledcWrite(LEFT_PWM_CHANNEL_1, speed);
  ledcWrite(RIGHT_PWM_CHANNEL_0, speed);

}

void changeDirectionBuffer() {
  ledcWrite(LEFT_PWM_CHANNEL_0, 0);
  ledcWrite(LEFT_PWM_CHANNEL_1, 0);
  ledcWrite(RIGHT_PWM_CHANNEL_0, 0);
  ledcWrite(RIGHT_PWM_CHANNEL_1, 0);
  delay(500); // this is a reasonable delay for switching direction
}

