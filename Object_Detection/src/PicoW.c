/**
 * @file PicoW.c
 * @brief Implementation for PicoW SPI communication functions.
 *
 * Provides functions to initialize SPI communication and transmit data to
 * connected devices using the EUSCI_B0_SPI interface. Includes handling for
 * chip select and necessary delays between transmissions.
 * @author Elvis Chino-Islas
 * @date 2024-05-10
 */

#include "../inc/PicoW.h"

/**
 * @brief Initializes the SPI communication via EUSCI_B0.
 *
 * Sets up the EUSCI_B0 SPI interface with a specific baud rate for communication.
 * The baud rate is set to 1 MHz.
 */
void PicoW_Init() {
    EUSCI_B0_SPI_Init(1000 * 1000); // Initialize SPI at 1 MHz
}

/**
 * @brief Transmits a single byte over SPI.
 * @param data The byte to transmit.
 *
 * Sends a single byte of data using the EUSCI_B0_SPI interface by writing it
 * to the SPI data register.
 */
void PicoW_Transmit_Byte(uint8_t data) {
    EUSCI_B0_SPI_Data_Write(data); // Write single byte to SPI
}

/**
 * @brief Transmits multiple bytes over SPI.
 * @param n The number of bytes to transmit.
 * @param data Pointer to the array of bytes to be transmitted.
 *
 * Transmits a sequence of bytes over SPI. This function manages the chip select line
 * to ensure proper communication setup and teardown with the target device. It incorporates
 * necessary delays for device synchronization.
 */
void PicoW_Transmit_Bytes(int n, uint8_t *data) {
    EUSCI_B0_Control_Chip_Select(0x00); // Enable chip select line

    for (int i = 0; i < n; i++) {
        PicoW_Transmit_Byte(data[i]); // Transmit each byte
    }

    Clock_Delay1ms(30); // Delay to ensure transmission completion

    EUSCI_B0_Control_Chip_Select(0x01); // Disable chip select line

    Clock_Delay1us(25); // Short delay before next possible communication
}
