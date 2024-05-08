/**
 * @file EUSCI_B0_SPI.c
 * @brief Source code for the EUSCI_B0_SPI driver.
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

#include "../inc/EUSCI_B0_SPI.h"

void EUSCI_B0_SPI_Init(uint32_t clock_frequency)
{
    // Configure the following pins to use the primary module function
    // by setting Bits 7 to 5 in the SEL0 register and
    // clearing Bits 7 to 5 in the SEL1 register
    //   - P1.7 (UCB0SOMI) [MISO]
    //   - P1.6 (UCB0SIMO) [MOSI]
    //   - P1.5 (UCB0CLK)  [SCLK]
    P1->SEL0 |= 0xE0;
    P1->SEL1 &= ~0xE0;

    // Configure the P3.0 pin (Chip Select) as an output GPIO pin by clearing Bit 0 in the
    // SEL0 and SEL1 registers and setting Bit 0 in the DIR register
    P3->SEL0 &= ~0x01;
    P3->SEL1 &= ~0x01;
    P3->DIR |= 0x01;

    // Initialize the output of the Chip Select signal (active low) to high
    // by setting Bit 0 in the OUT register
    P3->OUT |= 0x01;

    // Hold the EUSCI_B0 module in the reset state by setting the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_B0->CTLW0 |= 0x01;

    // Configure the EUSCI_B0 module to operate in SPI Mode 0.
    // Set the UCCKPH bit (Bit 15) to allow data to be captured on the first SPI clock edge and
    // changed on the following edge. Then, clear the UCCKPL bit (Bit 14) to configure
    // the SPI clock to be low when it is inactive
    EUSCI_B0->CTLW0 |= 0x8000;
    EUSCI_B0->CTLW0 &= ~0x4000;

    // Set the bit order to Most Significant Bit (MSB) first by setting the UCMSB bit (Bit 13) in the CTLW0 register
    EUSCI_B0->CTLW0 |= 0x2000;

    // Select 8-bit character length by clearing the UC7BIT bit (Bit 12) in the CTLW0 register
    EUSCI_B0->CTLW0 &= ~0x1000;

    // Select master mode by setting the UCMST bit (Bit 11) in the CTLW0 register
    EUSCI_B0->CTLW0 |= 0x0800;

    // Configure the mode of the EUSCI_B0 module to be 4-pin SPI with active low UCSTE (chip select)
    // by writing 10b to the UCMODEx field (Bits 10-9). This can be done by setting Bit 10
    // and clearing Bit 9 in the CTLW0 register
    EUSCI_B0->CTLW0 |= 0x0400;
    EUSCI_B0->CTLW0 &= ~0x0200;

    // Enable synchronous mode to allow the EUSCI_B0 module to use SPI by
    // setting the UCSYNC bit (Bit 8) in the CTLW0 register
    EUSCI_B0->CTLW0 |= 0x0100;

    // Select the eUSCI clock source to SMCLK by writing 11b to the UCSSELx field (Bits 7-6).
    // This can be done by setting Bit 7 and Bit 6 in the CTLW0 register
    EUSCI_B0->CTLW0 |= 0x00C0;

    // Configure the UCSTE pin to be used to generate the enable signal (chip select) for a 4-wire slave
    EUSCI_B0->CTLW0 |= 0x0002;

    // Set the baud rate value by writing to the UCBRx field (Bits 15 to 0) in the BRW register
    // N = (Clock Frequency) / (Baud Rate) = (12,000,000 / 1,000,000)
    // Use only the integer part, so N = 12
    EUSCI_B0->BRW = (12000000 / clock_frequency);

    // Disable the following interrupts by clearing the
    // corresponding bits in the IE register:
    // - Transmit Complete Interrupt (UCTXCPTIE, Bit 3)
    // - Start Bit Interrupt (UCSTTIE, Bit 2)
    EUSCI_B0->IE &= ~0x0C;

    // Enable the following interrupts by setting the
    // corresponding bits in the IE register
    // - Transmit Interrupt (UCTXIE, Bit 1)
    // - Receive Interrupt (UCRXIE, Bit 0)
    EUSCI_B0->IE |= 0x03;

    // Release the EUSCI_B0 module from the reset state by clearing the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_B0->CTLW0 &= ~0x01;
}

uint8_t EUSCI_B0_SPI_Data_Read()
{
    // Check the Receive Interrupt flag (UCRXIFG, Bit 0)
    // in the IFG register and wait if the flag is not set
    // If the UCRXIFG is set, then the Receive Buffer (UCAxRXBUF) has
    // received a complete character
    while((EUSCI_B0->IFG & 0x01) == 0);

    // Return the data from the Receive Buffer (UCAxRXBUF)
    // Reading the UCAxRXBUF will reset the UCRXIFG flag
    return EUSCI_B0->RXBUF;
}

void EUSCI_B0_SPI_Data_Write(uint8_t data)
{
    // Check the Transmit Interrupt flag (UCTXIFG, Bit 1)
    // in the IFG register and wait if the flag is not set
    // If the UCTXIFG is set, then the Transmit Buffer (UCAxTXBUF) is empty
    while((EUSCI_B0->IFG & 0x02) == 0);

    // Write the data to the Transmit Buffer (UCAxTXBUF)
    // Writing to the UCAxTXBUF will clear the UCTXIFG flag
    EUSCI_B0->TXBUF = data;
}

void EUSCI_B0_Control_Chip_Select(uint8_t chip_select_enable)
{
    // Clear P3.0 to 0 when chip select is enabled
    if (chip_select_enable == 0x00)
    {
        P3->OUT &= ~0x01;
    }

    // Set P3.0 to 1 when chip select is disabled
    else
    {
        P3->OUT |= 0x01;
    }
}
