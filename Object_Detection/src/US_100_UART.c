/**
 * @file US_100_UART.c
 * @brief Source code for the US_100_UART driver.
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

#include "../inc/US_100_UART.h"

void US_100_UART_Init()
{
    // Configure pins P9.6 (PM_UCA3RXD) and P9.7 (PM_UCA3TXD) to use the primary module function
    // by setting Bits 7 and 6 in the SEL0 register for P9
    // and clearing Bits 7 and 6 in the SEL1 register for P9
    P9->SEL0 |= 0xC0;
    P9->SEL1 &= ~0xC0;

    // Hold the EUSCI_A3 module in the reset state by setting the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A3->CTLW0 |= 0x01;

    // Clear all of the bits in the Modulation Control Word (MCTLW) register
    EUSCI_A3->MCTLW &= ~0xFF;

    // Disable the parity bit by clearing the UCPEN bit (Bit 15) in the CTLW0 register
    EUSCI_A3->CTLW0 &= ~0x8000;

    // Select odd parity for the parity bit by clearing the UCPAR bit (Bit 14) in the CTLW0 register
    // Note that the UCPAR bit is not used when parity is disabled
    EUSCI_A3->CTLW0 &= ~0x4000;

    // Set the bit order to Least Significant Bit (LSB) first by clearing the UCMSB bit (Bit 13) in the CTLW0 register
    EUSCI_A3->CTLW0 &= ~0x2000;

    // Select 8-bit character length by clearing the UC7BIT bit (Bit 12) in the CTLW0 register
    EUSCI_A3->CTLW0 &= ~0x1000;

    // Select one stop bit by clearing the UCSPB bit (Bit 11) in the CTLW0 register
    EUSCI_A3->CTLW0 &= ~0x0800;

    // Enable UART mode by writing 00b to the UCMODEx field (Bits 10-9) in the CTLW0 register
    EUSCI_A3->CTLW0 &= ~0x0600;

    // Disable synchronous mode by clearing the UCSYNC bit (Bit 8) in the CTLW0 register
    EUSCI_A3->CTLW0 &= ~0x0100;

    // Configure the EUSCI_A3 module to use SMCLK as the clock source by
    // writing a value of 10b to the UCSSELx field (Bits 7 to 6) in the CTLW0 register
    EUSCI_A3->CTLW0 |= 0x00C0;

    // Set the baud rate value by writing to the UCBRx field (Bits 15 to 0)
    // in the BRW register
    // N = (Clock Frequency) / (Baud Rate) = (12,000,000 / 9600) = 1250
    // Use only the integer part, so N = 1250
    EUSCI_A3->BRW = 1250;

    // Disable the following interrupts by clearing the
    // corresponding bits in the IE register:
    // - Transmit Complete Interrupt (UCTXCPTIE, Bit 3)
    // - Start Bit Interrupt (UCSTTIE, Bit 2)
    EUSCI_A3->IE &= ~0x0C;

    // Enable the following interrupts by setting the
    // corresponding bits in the IE register
    // - Transmit Interrupt (UCTXIE, Bit 1)
    // - Receive Interrupt (UCRXIE, Bit 0)
    EUSCI_A3->IE |= 0x03;

    // Release the EUSCI_A3 module from the reset state by clearing the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A3->CTLW0 &= ~0x01;
}

uint8_t US_100_UART_InChar()
{
    // Check the Receive Interrupt flag (UCRXIFG, Bit 0)
    // in the IFG register and wait if the flag is not set
    // If the UCRXIFG is set, then the Receive Buffer (UCAxRXBUF) has
    // received a complete character
    while((EUSCI_A3->IFG & 0x01) == 0);

    // Return the data from the Receive Buffer (UCAxRXBUF)
    // Reading the UCAxRXBUF will reset the UCRXIFG flag
    return EUSCI_A3->RXBUF;
}

void US_100_UART_OutChar(uint8_t data)
{
    // Check the Transmit Interrupt flag (UCTXIFG, Bit 1)
    // in the IFG register and wait if the flag is not set
    // If the UCTXIFG is set, then the Transmit Buffer (UCAxTXBUF) is empty
    while((EUSCI_A3->IFG & 0x02) == 0);

    // Write the data to the Transmit Buffer (UCAxTXBUF)
    // Writing to the UCAxTXBUF will clear the UCTXIFG flag
    EUSCI_A3->TXBUF = data;
}
