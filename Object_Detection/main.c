/**
 * @file main.c
 * @brief Main source code for the ADC program.
 *
 * This file contains the main entry point for the ADC program.
 * The main controller (Controller_1) demonstrates a Wall Follower robot that explores a simple maze.
 *
 * It interfaces the following peripherals using the ADC14 module to demonstrate object detection:
 *  - Three Sharp GP2Y0A21YK0F Analog Distance Sensor (10-80cm)
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (100 Hz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (2 kHz)
 *
 * @author Aaron Nanas
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Timer_A1_Interrupt.h"

#include "inc/US_100_UART.h"
#include "inc/Servo.h"

//#define CONTROLLER_1    1
#define CONTROLLER_2    1
//#define CONTROLLER_3    1

//#define DEBUG_ACTIVE    1

// Initialize constant distance values (in mm)
#define TOO_CLOSE_DISTANCE  200
#define TOO_FAR_DISTANCE    500
#define DESIRED_DISTANCE    250

// Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL         4000
#define PWM_SWING           1000
#define PWM_MIN             0
#define PWM_MAX             (PWM_NOMINAL + PWM_SWING)

#define START_MOTOR_PWM     1250
#define FINISH_MOTOR_PWD    6500

// Declare global variables used to store filtered distance values from the Analog Distance Sensor
uint32_t Filtered_Distance_Left;
uint32_t Filtered_Distance_Center;
uint32_t Filtered_Distance_Right;

// Declare global variables used to store converted distance values from the Analog Distance Sensor
int32_t Converted_Distance_Left;
int32_t Converted_Distance_Center;
int32_t Converted_Distance_Right;

// Declare global variable used to store the amount of error
int32_t Error;

// Proportional Controller Gain
int32_t Kp = 8;

// Initialize set point to 250 mm
int32_t Set_Point = 250;

// Declare global variables used to update PWM duty cycle values for the motors
uint16_t Duty_Cycle_Left;
uint16_t Duty_Cycle_Right;


/**
 * @brief This function is the handler for the SysTick periodic interrupt with a rate of 100 Hz.
 *
 * The SysTick_Handler generates a periodic interrupt that calls a specific controller function based on the selected
 * active configuration. Only one of the options can be defined at a time: CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3.
 *
 * @param None
 *
 * @return None
 */
void SysTick_Handler(void)
{
    // this is where we would put our logic
}

/**
 * @brief User-defined function executed by Timer A1 using a periodic interrupt at a rate of 2 kHz.
 *
 * The Timer_A1_Periodic_Task function generates a periodic interrupt with a rate of 2 kHz. It samples the
 * distance values measured by three Sharp GP2Y0A21YK0F analog distance sensors.
 *
 * @param None
 *
 * @return None
 */
void Timer_A1_Periodic_Task(void)
{
    // this is where we would sample the sensors
}

/**
 * @brief User-defined function executed by Timer A1 using a periodic interrupt at a rate of 2 kHz.
 *
 * The Timer_A1_Periodic_Task function generates a periodic interrupt with a rate of 2 kHz. It samples the
 * distance values measured by three Sharp GP2Y0A21YK0F analog distance sensors.
 *
 * @param None
 *
 * @return None
 */
void Timer_A2_Periodic_Task(void)
{
    // this is where we would sample the sensors
    static uint16_t last_capture = 0;
    uint16_t capture = TIMER_A0->CCR[1];
    uint16_t pulse_duration = capture - last_capture;
    last_capture = capture;
    printf("pulse_duration : %5d \n", pulse_duration);
}



uint16_t Get_Distance()
{
    char US_100_UART_Buffer[US_100_UART_BUFFER_SIZE] = {0};
    US_100_UART_OutChar(0x55);
    US_100_UART_Buffer[0] = US_100_UART_InChar();
    US_100_UART_Buffer[1] = US_100_UART_InChar();

    uint16_t distance_value = US_100_UART_Buffer[1] | (US_100_UART_Buffer[0] << 8);
    // printf("Distance: %d mm\n", distance_value);

    // Clock_Delay1ms(20);
    return distance_value;
}

typedef struct {
    uint16_t distance;
    uint16_t angle;
} measurment_t;

measurment_t Full_Scan_Min_Distance() {
    static bool count_down = false;
    static uint16_t angle = 0; 

    const int step = 10;
    uint16_t min_distance = UINT16_MAX;
    uint16_t angle_for_min_distance = 0;

    while (1) {
        if (count_down) {
            if (angle == 0) {
                count_down = false;
                break;
            }
            angle-=step;
        } else {
            if (angle == 270) {
                count_down = true;
                break;

            }
            angle+=step;
        }

        Servo_SetAngle(angle);
        uint16_t current_distance = Get_Distance();
        printf("Angle: %d, Distance: %d mm\n", angle, current_distance);
        if (current_distance < min_distance) {
            min_distance = current_distance;
            angle_for_min_distance = angle;
        }
        Clock_Delay1ms(10);  // Delay to allow servo to move and settle
    }

//    Servo_SetAngle(angle_for_min_distance); // Point to the closest object
    measurment_t res = {min_distance, angle_for_min_distance};
    return res;
}

// PID controller variables
float integral = 0;
float previous_error = 0;

uint16_t PID_Controller(int desired_distance, int measured_distance) {
    float kp = 0.5;   // Proportional gain
//    float ki = 0.01;  // Integral gain
//    float kd = 0.05;  // Derivative gain

    // Calculate error
    float error = desired_distance - measured_distance;

    // Integral term calculation
    integral += error;

    // Derivative term calculation
    float derivative = error - previous_error;

    // Calculate PID output
    uint16_t output = (uint16_t)(kp * error); // + ki * integral + kd * derivative);

    // Update previous error
    previous_error = error;

    return output;
}


int main(void)
{
    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize EUSCI_A0_UART to use the printf function
    EUSCI_A0_UART_Init_Printf();

    // Initialize the DC motors
    Motor_Init();

    // Initialize the US-100 Ultrasonic Distance Sensor module
    US_100_UART_Init();


    // Initialize motor duty cycle values
    Duty_Cycle_Left  = PWM_NOMINAL;
    Duty_Cycle_Right = PWM_NOMINAL;

    // Initialize SysTick periodic interrupt with a rate of 100 Hz
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize Timer A1 with interrupts enabled and an interrupt rate of 2 kHz
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Timer_A2_Interrupt_Init(&Timer_A2_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    
    // Initialize the Servo motor
    Servo_Init();

    // Enable the interrupts used by Timer A1 and other modules
    EnableInterrupts();

    while(1) {
        measurment_t min_mes = Full_Scan_Min_Distance();  // Find closest object once initially

        uint16_t duty_cycle_adjustment = PID_Controller(DESIRED_DISTANCE, min_mes);

        // Adjust motor speed based on PID output
        Duty_Cycle_Left = PWM_NOMINAL + duty_cycle_adjustment;
        Duty_Cycle_Right = PWM_NOMINAL - duty_cycle_adjustment;

        // Apply the duty cycle to the motors
        Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
        printf("Left: %d, Right: %d\n", Duty_Cycle_Left, Duty_Cycle_Right);
        printf("Duty Cycle Adjustment: %d\n", duty_cycle_adjustment);

        printf("Following object at angle %d with distance %d mm\n", min_mes.angle, min_mes.distance);
        Clock_Delay1ms(1000);
        Motor_Stop();

        // Clock_Delay1ms(50);  // Adjust the frequency of control adjustments
    }
}
