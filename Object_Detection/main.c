/**
 * @file main.c
 * @brief Main source code for the Autonomous Distance Controlled Robot.
 *
 * Implements a control system for a robot that uses an ultrasonic sensor to detect the distance to the nearest object.
 * 
 * ### Timers used:
 * - SysTick: Generates periodic interrupts at 100 Hz for system timing.
 * - Timer A0: Drives PWM signals for DC motor control.
 * - Timer A2: Drives PWM signals for servo motor control.
 * 
 * ### eUSCI modules used:
 * - eUSCI_A0: UART communication for debugging and output.
 * - eUSCI_B0: SPI communication for interfacing with the PicoW module.
 *
 * @authors Elvis Chino-Islas, Hong Shen
 * @date 2024-05-10
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
#include "inc/US_100_UART.h"
#include "inc/Servo.h"
#include "inc/PicoW.h"

/// Constants defining sensor threshold distances in millimeters.
#define TOO_CLOSE_DISTANCE  200
#define TOO_FAR_DISTANCE    500
#define DESIRED_DISTANCE    250
#define DESIRED_ANGLE       135  // Half of 270 degrees for servo orientation.

/// Motor PWM constants to define duty cycles and limits.
#define PWM_NOMINAL         3500
#define PWM_SWING           1000
#define PWM_MIN             0
#define PWM_MAX             (PWM_NOMINAL + PWM_SWING)

#define START_MOTOR_PWM     1250
#define FINISH_MOTOR_PWD    6500

/// Custom type for storing sensor measurements.
typedef struct {
    uint32_t distance;
    uint32_t angle;
} mes_t;

/// Union to facilitate SPI transmission of measurements.
typedef union {
    mes_t data;
    uint8_t buffer[8];
} spi_trans_t;

/// Global variable holding the current sensor measurements.
mes_t current_measurement;

/// Motor duty cycle global variables for dynamic adjustment.
static uint16_t Duty_Cycle_Left;
static uint16_t Duty_Cycle_Right;

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
void PID_Controller(mes_t des, mes_t mes) {
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

/**
 * @brief SysTick interrupt handler.
 * 
 * Invoked at 100 Hz to perform control adjustments based on sensor input.
 */
void SysTick_Handler(void) {
    mes_t set_point = {DESIRED_DISTANCE, DESIRED_ANGLE};
    PID_Controller(set_point, current_measurement);
    Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
}

/**
 * @brief Retrieves the current distance measurement from the US-100 ultrasonic sensor.
 * 
 * @return uint16_t The measured distance to the nearest object in millimeters.
 */
uint16_t Get_Distance() {
    char US_100_UART_Buffer[US_100_UART_BUFFER_SIZE] = {0};
    US_100_UART_OutChar(0x55);  // Command to measure distance.
    US_100_UART_Buffer[0] = US_100_UART_InChar();
    US_100_UART_Buffer[1] = US_100_UART_InChar();

    uint16_t distance_value = (US_100_UART_Buffer[0] << 8) | US_100_UART_Buffer[1];
    return distance_value;
}

/**
 * @brief Performs a full scan to find the minimum distance to an object across a range of angles.
 * 
 * @return mes_t The measurement containing the minimum distance and corresponding angle.
 */
mes_t Full_Scan_Min_Distance() {
    const int step = 10;  // Angle step size for scanning.

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

            angle -= step;
        } else {
            if (angle == 270) {
                count_down = true;
                break;
            }

            angle += step;
        }

        Servo_SetAngle(angle);
        uint16_t current_distance = Get_Distance();

        spi_trans_t temp;
        temp.data.angle = angle;
        temp.data.distance = current_distance;
        PicoW_Transmit_Bytes(8, temp.buffer);

        if (current_distance < min_distance) {
            min_distance = current_distance;
            angle_for_min_distance = angle;
        }
        Clock_Delay1ms(10);
    }

    mes_t res = {min_distance, angle_for_min_distance};
    return res;
}

/**
 * @brief Main function where initialization and main loop are executed.
 */
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
    Duty_Cycle_Left = PWM_NOMINAL;
    Duty_Cycle_Right = PWM_NOMINAL;

    // Initialize SysTick periodic interrupt with a rate of 100 Hz
    // SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize the Servo motor
    Servo_Init();

    // Initialize SPI for PicoW Communication
    PicoW_Init();

    // Enable the interrupts used by Timer A1 and other modules
    EnableInterrupts();

    while (1) {
        // Find closest object once initially
        current_measurement = Full_Scan_Min_Distance();

        // Debug outputs
        printf("Duty Cycle Left: %d, Duty Cycle Right: %d\n", Duty_Cycle_Left, Duty_Cycle_Right);
    }
}
