/**
 * @file EUSCI_B0_SPI.c
 * @brief Source code for the EUSCI_B0_SPI driver.
 *
 * This file contains the function definitions for the EUSCI_B0_SPI driver.
 * The EUSCI_B0_SPI driver uses busy-wait implementation.
 *
 * @note This function assumes that the necessary pin configurations for SPI communication have been performed
 *       on the corresponding pins. The output from the pins will be observed using an oscilloscope.
 *       - P1.5 (CLK)
 *       - P1.6 (MISO)
 *       - P1.7 (MOSI)
 *       - P5.2 (CS)
 *
 * For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Elvis Chino-Islas
 *
 */
#include "../inc/EUSCI_B0_SPI.h"

void EUSCIB0_IRQHandler(void) {
    // Check if the TX interrupt flag is set
    if(EUSCI_B0->IFG & 0x0002)
    {
        // Clear the TX interrupt flag
        EUSCI_B0->IFG &= ~0x0002;
    }
}

void EUSCI_B0_SPI_Init()
{
    // Hold the EUSCI_B0 module in reset mode
    EUSCI_B0_SPI->CTLW0 |= 0x0001

    //     CTWL0 Register Configuration
    //
    //     Bit(s)      Field       Value       Description
    //     -----       -----       -----       -----------
    //      15         UCCKPH       0x0        Data is changed on the first UCLK edge and captured on the following edge
    //      14         UCCKPL       0x0        The inactive state is low
    //      13         UCMSB        0x1        MSB first
    //      12         UC7BIT       0x0        8-bit data
    //      11         UCMST        0x1        Master mode is selected
    //      10-9       UCMODEx      0x2        4-pin SPI with UCxSTE active high: Slave enabled when UCxSTE = 1
    //      8          UCSYNC       0x1        Synchronous mode
    //      7-6        UCSSELx      0x3        eUSCI clock source is SMCLK
    //      5-2        Reserved     0x0        Reserved
    //      1          UCSTEM       0x1        STE pin is used to prevent conflicts with other masters
    //      0          UCSWRST      0x1        eUSCI logic held in reset state
    EUSCI_B0_SPI->CTLW0 = 0x2DC3;

    // Set the SPI frequency. Since SMCLK is selected as the clock source,
    // the frequency used is 12 MHz
    // Choose 1 MHz for the SCK frequency:
    // N = 12 MHz / 1 MHz = 12
    // N = 12
    EUSCI_B0_SPI->BRW = 12;

    // Configre pins P1.5 (CLK), P1.6 (SIMO), P1.7 (SOMI), and P5.2 (STE)
    P1->SEL0 |= 0xE0;
    P1->SEL1 &= ~0xE0;
    P1->DIR |= 0xE0;

    P5->SEL0 |= 0x04;
    P5->SEL1 &= ~0x04;
    P5->DIR |= 0x04;

    // Enable receive interrupt but disable transmit interrupt
    EUSCI_B0_SPI->IE = 0x0001;

    // Set the priority of the EUSCI_B0 interrupt to 2
    NVIC->IP[5] = (NVIC->IP[5] & 0xFFFFF1FF) | 0x00000040;

    // Enable the interrupt in the NVIC
   NVIC->ISER[0] |= 0x00100000;
    
    //  Release the EUSCI_B0 module from reset
    EUSCI_B0_SPI->CTLW0 &= ~0x0001;
}

void EUSCI_B0_SPI_Send_A_Byte(uint8_t data)
{
    // Wait until the transmit buffer is empty
    while((EUSCI_B0->IFG & 0x0002) == 0);

    // Write the data to the TX buffer
    EUSCI_B0->TXBUF = data;

    // Wait until the transmit buffer is empty
    while((EUSCI_B0->IFG & 0x0002) == 0);
}

