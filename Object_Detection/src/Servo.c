#include "../inc/Servo.h"

void Servo_Init() {
    // Move the servo to the 0 degree position
    Timer_A2_PWM_Init(TIMER_A2_PERIOD_CONSTANT, 0, 0, 0);
    Servo_SetAngle(0);
}

void Servo_SetAngle(uint16_t angle) {
    // Convert the angle to a duty cycle value
    // The duty cycle value is calculated as follows:
    // Duty Cycle % = (angle / 270) * 100
    // The duty cycle value is calculated as follows:
    // Duty Cycle Value = (Duty Cycle % * period_constant) / 100
    // The period constant is 60000

    // Map the angle to the duty cycle range
    uint16_t duty_cycle = map(angle);

    Timer_A2_Update_Duty_Cycle_1(duty_cycle);
}

int map(int input) {
    int input_start = 0;
    int input_end = 270;
    int output_start = 1000;
    int output_end = 6700;

    return output_start + ((output_end - output_start) * (input - input_start)) / (input_end - input_start);
}

