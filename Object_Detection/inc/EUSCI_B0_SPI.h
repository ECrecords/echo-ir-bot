/**
 * @file EUSCI_B0_SPI.h
 * @brief Header file for the EUSCI_B0_SPI driver.
 *
 * This file contains the function definitions for the EUSCI_B0_SPI driver.
 *
 * @note This function assumes that the necessary pin configurations for SPI communication have been performed
 *       on the corresponding pins.
 *       - P3.0 (SCE, Slave Chip Enable)
 *       - P1.5 (SCLK, Slave Clock)
 *       - P1.6 (MOSI, Master Out Slave In)
 *       - P1.7 (MISO, Master In Slave Out)
 *
 * @note For more information regarding the eUSCI_SPI registers used, refer to the eUSCI_B SPI Registers section (25.5)
 * of the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_EUSCI_B0_SPI_H_
#define INC_EUSCI_B0_SPI_H_

#include <stdio.h>
#include <stdint.h>
#include "msp.h"
#include "Clock.h"

#define EUSCI_B0_SPI_CLK_1_MHZ       1000000

/**
 * @brief Initializes the SPI module EUSCI_B0 for communication.
 *
 * This function configures the EUSCI_B0 module to enable SPI communication
 * with the following configuration:
 *
 * - CTLW0 Register Configuration:
 *
 *  Bit(s)      Field       Value       Description
 *  -----       -----       -----       -----------
 *   15         UCCKPH       0x1        Data is captured on the first edge and changed on the following edge
 *   14         UCCKPL       0x0        Clock is low when inactive
 *   13         UCMSB        0x1        MSB first
 *   12         UC7BIT       0x0        8-bit data
 *   11         UCMST        0x1        Master mode is selected
 *   10-9       UCMODEx      0x2        4-pin SPI with active low UCSTE
 *   8          UCSYNC       0x1        Synchronous mode
 *   7-6        UCSSELx      0x2        eUSCI clock source is SMCLK
 *   5-2        Reserved     0x0        Reserved
 *   1          UCSTEM       0x1        UCSTE pin is used to generate signal for 4-wire slave
 *   0          UCSWRST      0x0        eUSCI logic held low in non-reset state
 *
 * @param uint8_t chip_select_mode Selects between automatic toggle mode (0x01)
 * manual toggle mode (0x00) for the chip select signal.
 *
 * @param uint32_t clock_frequency Configures the frequency of the SPI clock.
 *
 * @return None
 */
void EUSCI_B0_SPI_Init(uint32_t clock_frequency);

/**
 * @brief Receives a single character over SPI using the EUSCI_B0 module.
 *
 * This function receives a single character over SPI using the EUSCI_B0 module.
 * It waits until a character is available in the SPI receive buffer and then reads
 * the received data.
 *
 * @param None
 *
 * @return The received unsigned 8-bit data from the MISO line.
 */
uint8_t EUSCI_B0_SPI_Data_Read();

/**
 * @brief Transmits a single byte over SPI using the EUSCI_B0 module.
 *
 * This function transmits a single character over SPI using the EUSCI_B0 module.
 * It waits until the transmit buffer is ready to accept new data and then writes the provided data
 * to the transmit buffer for transmission.
 *
 * @param data The unsigned 8-bit data to be transmitted over the MOSI line.
 *
 * @return None
 */
void EUSCI_B0_SPI_Data_Write(uint8_t data);

/**
 * @brief Controls the SPI chip select line for the EUSCI_B0 module.
 *
 * This function controls the state of the chip select (CS) line based on the input parameter, chip_select_enable.
 * The CS line is configured to be active low. It clears the output of the P3.0 to 0 to enable the CS line
 * or sets it to 1 to disable it.
 *
 * @param chip_select_enable Determines the state of the chip select line.
 *
 * @return None
 */
void EUSCI_B0_Control_Chip_Select(uint8_t chip_select_enable);

#endif /* INC_EUSCI_B0_SPI_H_ */
