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
#include <math.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Timer_A1_Interrupt.h"

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

    // Initialize motor duty cycle values
    Duty_Cycle_Left  = PWM_NOMINAL;
    Duty_Cycle_Right = PWM_NOMINAL;

    // Initialize SysTick periodic interrupt with a rate of 100 Hz
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize Timer A1 with interrupts enabled and an interrupt rate of 2 kHz
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Enable the interrupts used by Timer A1 and other modules
    EnableInterrupts();

    while(1)
    {
        // prints to debug here
    }
}
