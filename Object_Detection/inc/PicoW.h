/**
 * @file PicoW.h
 * @brief Header for PicoW module providing SPI communication functionalities.
 *
 * This header declares functions for initializing and handling SPI communications
 * through the EUSCI_B0 SPI interface. It includes functions for transmitting single
 * and multiple bytes over SPI.
 *
 * @date May 4, 2024
 * @author Elvis Chino-Islas
 */

#ifndef INC_PICOW_
#define INC_PICOW_

#include "../inc/EUSCI_B0_SPI.h"

/**
 * @brief Initializes the SPI communication via EUSCI_B0.
 *
 * This function sets up the EUSCI_B0 SPI interface at a specified baud rate,
 * preparing it for data transmission with connected SPI devices.
 */
void PicoW_Init();

/**
 * @brief Transmits a single byte over SPI.
 * @param data The byte to be transmitted.
 *
 * This function sends a single byte of data using the EUSCI_B0 SPI interface.
 */
void PicoW_Transmit_Byte(uint8_t data);

/**
 * @brief Transmits multiple bytes over SPI.
 * @param n The number of bytes to transmit.
 * @param mes Pointer to the array of bytes to be transmitted.
 *
 * This function manages the transmission of multiple bytes over the SPI,
 * handling the chip select and synchronization necessary for communication
 * with SPI devices.
 */
void PicoW_Transmit_Bytes(int n, uint8_t *mes);

#endif /* INC_PICOW_ */
