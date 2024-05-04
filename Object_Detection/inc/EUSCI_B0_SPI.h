#ifndef INC_EUSCI_B0_SPI_H_
#define INC_EUSCI_B0_SPI_H_

#include <stdint.h>
#include "msp.h"

/**
 * @brief Initializes the SPI module EUSCI_B2 for communication.
 *

 *
 * - CTWL0 Register Configuration:
 *
 *   Bit(s)      Field       Value       Description
 *   -----       -----       -----       -----------
 *    15         UCA10        0x0        Own address is a 7-bit address
 *    14         UCSLA10      0x0        Address slave has a 7-bit address
 *    13         UCMM         0x0        Single master
 *    12         Reserved     0x0        Reserved
 *    11         UCMST        0x1        Master mode is selected
 *    10-9       UCMODEx      0x3        I2C mode is selected
 *    8          UCSYNC       0x1        Synchronous mode
 *    7-6        UCSSELx      0x3        eUSCI clock source is SMCLK
 *    5          UCTXACK      0x0        Transmit ACK condition in slave mode
 *    4          UCTR         0x0        Transmitter / Receiver
 *    3          UCTXNACK     0x0        Transmit NACK condition in slave mode
 *    2          UCTXSTP      0x0        Transmit STOP condition in master mode
 *    1          UCTXSTT      0x1        Transmit START condition in master mode
 *    0          UCSWRST      0x1        eUSCI logic held in reset state
 *
 * - CTWL1 Register Configuration:
 *
 *   Bit(s)      Field       Value       Description
 *   -----       -----       -----       -----------
 *    15-9       Reserved     0x0        Reserved
 *    8          UCETXINT     0x0        Early UCTXIFG0 flag in slave mode
 *    7-6        UCCLTO       0x0        Disable clock low timeout
 *    5          UCSTPNACK    0x0        Send NACK before STOP condition in master receiver mode
 *    4          UCSWACK      0x0        Address acknowledge of slave is controlled by eUSCI module
 *    3-2        UCASTPx      0x0        No automatic STOP generation in slave mode when UCBCNTIFG is available
 *    1-0        UCGLITx      0x0        Deglitch time of 50 ns
 *
 * For setting the SCL frequency, the clock source used is 12 MHz.
 * To achieve a 400 kHz SCL frequency, the BRW value is set to 30.
 *
 * For more information regarding the registers used, refer to the EUSCI_B I2C Registers section
 * of the MSP432Pxx Microcontrollers Technical Reference Manual.
 *
 * @note This function assumes that the necessary pin configurations for I2C communication have been performed:
 *       - P6.4 (SDA, Serial Data)
 *       - P6.5 (SCL, Serial Clock)
 *
 * @return None
 */
void EUSCI_B0_SPI_Init();

void EUSCI_B0_SPI_Send_A_Byte(uint8_t data);


#endif /* INC_EUSCI_B0_SPI_H_ */
