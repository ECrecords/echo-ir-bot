/**
 * @file Servo.c
 * @brief Implementation for controlling a servo motor using Timer_A2 PWM.
 *
 * Implements functions to initialize the servo motor and set its angle using PWM signals.
 * This involves calculating the correct duty cycle for the PWM based on the servo's angle.
 * @authors Elvis Chino-Islas, Hong Shen
 */

#include "../inc/Servo.h"

/**
 * @brief Initializes the servo motor to a default position.
 * Initializes the PWM on Timer A2 and sets the servo to 0 degrees.
 */
void Servo_Init() {
    Timer_A2_PWM_Init(TIMER_A2_PERIOD_CONSTANT, 0, 0, 0);
    Servo_SetAngle(0);  // Move the servo to the 0 degree position
}

/**
 * @brief Sets the servo motor to a specific angle.
 * @param angle The angle in degrees to set the servo. The valid range is 0 to 270 degrees.
 *
 * This function calculates the PWM duty cycle corresponding to the specified angle
 * and updates Timer A2 to position the servo accordingly.
 */
void Servo_SetAngle(uint16_t angle) {
    uint16_t duty_cycle = map(angle);  // Convert the angle to a duty cycle value
    Timer_A2_Update_Duty_Cycle_1(duty_cycle);
}

/**
 * @brief Maps an integer input from one range to another range.
 * @param input The input value to be mapped.
 * @return int The mapped output value.
 *
 * Maps the input value from a defined input range to a corresponding output range,
 * specifically designed for translating servo angle values to PWM duty cycle values.
 */
int map(int input) {
    int input_start = 0;
    int input_end = 270;
    int output_start = 1000;
    int output_end = 6700;

    return output_start + ((output_end - output_start) * (input - input_start)) / (input_end - input_start);
}
