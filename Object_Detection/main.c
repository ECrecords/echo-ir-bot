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
#include "inc/PicoW.h"

// Initialize constant distance values (in mm)
#define TOO_CLOSE_DISTANCE  200
#define TOO_FAR_DISTANCE    500
#define DESIRED_DISTANCE    250
#define DESIRED_ANGLE       270 >> 1

// Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL         3000
#define PWM_SWING           1000
#define PWM_MIN             0
#define PWM_MAX             (PWM_NOMINAL + PWM_SWING)

#define START_MOTOR_PWM     1250
#define FINISH_MOTOR_PWD    6500

// Declare global variable used to store the amount of error
int32_t Error;

// Proportional Controller Gain
int32_t Kp = 8;

// Initialize set point to 250 mm
int32_t Set_Point = 250;

// Declare global variables used to update PWM duty cycle values for the motors
uint16_t Duty_Cycle_Left;
uint16_t Duty_Cycle_Right;

typedef struct {
    uint32_t distance;
    uint32_t angle;
} measurment_t;

typedef union {
    measurment_t data;
    uint8_t buffer[8];
} spi_transmission_t;

measurment_t mes;

void PID_Controller(measurment_t des, measurment_t mes);

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
    measurment_t set_point = {DESIRED_DISTANCE, DESIRED_ANGLE};
    PID_Controller(set_point, mes);
}

uint16_t Get_Distance()
{
    char US_100_UART_Buffer[US_100_UART_BUFFER_SIZE] = {0};
    US_100_UART_OutChar(0x55);
    US_100_UART_Buffer[0] = US_100_UART_InChar();
    US_100_UART_Buffer[1] = US_100_UART_InChar();

    uint16_t distance_value = US_100_UART_Buffer[1] | (US_100_UART_Buffer[0] << 8);

    return distance_value;
}


measurment_t Full_Scan_Min_Distance() {
    const int step = 10;

    static bool count_down = false;
    static uint16_t angle = 0; 

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
//        printf("distance : %d mm Angle : %d angle\n", current_distance, angle);
        if (current_distance < min_distance) {
            min_distance = current_distance;
            angle_for_min_distance = angle;
        }
        Clock_Delay1ms(2);
    }

    measurment_t res = {min_distance, angle_for_min_distance};
    return res;
}

// PID controller coefficients for distance
float kp_distance = 3.0;   // Proportional gain for distance

// PID controller coefficients for angle
float kp_angle = 25.0;      // Proportional gain for angle

// PID controller variables for distance
float integral_distance = 0.0;
float derivative_distance;
float previous_error_distance = 0.0;

// PID controller variables for angle
float previous_error_angle = 0.0;

/**
 * PID_Controller - Calculate motor duty cycles using PID logic based on target and measured values
 * @param des: Target measurements (desired state)
 * @param mes: Current measurements (measured state)
 */
void PID_Controller(measurment_t des, measurment_t mes) {
    // Calculate errors for distance and angle
    float error_distance = des.distance - mes.distance;
    float error_angle = des.angle - mes.angle;


    // Calculate PID outputs for distance and angle
    int adjustment_distance = (int)(kp_distance * error_distance );
    int adjustment_angle = (int)(kp_angle * error_angle );

    // Adjust motor duty cycles based on PID outputs
    Duty_Cycle_Left = PWM_NOMINAL + adjustment_distance - adjustment_angle;
    Duty_Cycle_Right = PWM_NOMINAL + adjustment_distance + adjustment_angle;

    // Enforce PWM bounds to ensure motor speeds remain within allowable range
    Duty_Cycle_Left = fmax(PWM_MIN, fmin(Duty_Cycle_Left, PWM_MAX));
    Duty_Cycle_Right = fmax(PWM_MIN, fmin(Duty_Cycle_Right, PWM_MAX));

    // Update previous errors for the next control cycle
    previous_error_distance = error_distance;
    previous_error_angle = error_angle;
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

//    // Initialize SysTick periodic interrupt with a rate of 100 Hz
//    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize Timer A1 with interrupts enabled and an interrupt rate of 2 kHz
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE_1HZ);


    // Initialize the Servo motor
    Servo_Init();

    // Initialize SPI for PicoW Communication
    PicoW_Init();

    // Enable the interrupts used by Timer A1 and other modules
    EnableInterrupts();

    

    while(1) {
        mes = Full_Scan_Min_Distance();  // Find closest object once initially

        PicoW_Transmit_Bytes(8, ((spi_transmission_t)mes).buffer);

        // Debug outputs
        printf("Duty Cycle Left: %d, Duty Cycle Right: %d\n", Duty_Cycle_Left, Duty_Cycle_Right);

        Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
    }
}
