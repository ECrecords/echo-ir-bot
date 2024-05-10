/**
 * @file US_100_UART.h
 * @brief Header file for the US_100_UART driver.
 *
 * This file contains the function definitions for the US_100_UART driver.
 *
 * It interfaces with the US-100 Ultrasonic Distance Sensor, which uses the UART communication protocol.
 *  - Product Link: https://www.adafruit.com/product/4019
 *
 * The following connections must be made:
 *  - US 100 VCC            (Pin 1)     <-->  MSP432 LaunchPad VCC (3.3V)
 *  - US 100 Trigger / TX   (Pin 2)     <-->  MSP432 LaunchPad Pin P9.7 (PM_UCA3TXD)
 *  - US 100 Echo / RX      (Pin 3)     <-->  MSP432 LaunchPad Pin P9.6 (PM_UCA3RXD)
 *  - US 100 GND            (Pin 4)     <-->  MSP432 LaunchPad GND
 *  - US 100 GND            (Pin 5)     <-->  MSP432 LaunchPad GND
 *
 * @note Assumes that the necessary pin configurations for UART communication have been performed
 *       on the corresponding pins. P9.6 is used for UART RX while P9.7 is used for UART TX.
 *
 * @note For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_US_100_UART_H_
#define INC_US_100_UART_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "msp.h"
#include "Clock.h"

#define US_100_UART_BUFFER_SIZE 32

void US_100_UART_Init();

uint8_t US_100_UART_InChar();

void US_100_UART_OutChar(uint8_t data);

#endif /* INC_US_100_UART_H_ */
