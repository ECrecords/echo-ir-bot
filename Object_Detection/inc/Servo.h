/**
 * @file Servo.h
 * @brief Header file for Servo control using Timer_A2 PWM.
 *
 * This file declares functions for initializing and setting the angle of a servo motor.
 * It relies on Timer_A2 PWM functionalities to control the servo positioning.
 * @authors Elvis Chino-Islas, Hong Shen
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "../inc/Timer_A2_PWM.h"

/**
 * @brief Initializes the servo motor to a default position.
 *
 * Sets up the PWM using Timer A2 for controlling a servo and initializes the
 * servo to a 0-degree position.
 */
void Servo_Init();

/**
 * @brief Sets the servo motor to a specific angle.
 * @param angle The angle in degrees to set the servo. The valid range is 0 to 270 degrees.
 *
 * This function calculates the PWM duty cycle corresponding to the specified angle
 * and updates the Timer A2 to position the servo accordingly.
 */
void Servo_SetAngle(uint16_t angle);

/**
 * @brief Maps an integer input from one range to another range.
 * @param input The input value to be mapped.
 * @return int The mapped output value.
 *
 * Maps the input value from a defined input range to a corresponding output range.
 * This is primarily used to convert servo angle values to PWM duty cycle values.
 */
int map(int input);

#endif /* SERVO_H_ */
