
#ifndef SERVO_H_
#define SERVO_H_

#include "../inc/Timer_A2_PWM.h"

void Servo_Init();

void Servo_SetAngle(uint16_t angle);

int map(int input);

#endif /* SERVO_H_ */
