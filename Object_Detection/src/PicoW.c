#include "../inc/PicoW.h"


void PicoW_Init() {
    EUSCI_B0_SPI_Init(1000 * 1000);
}

void PicoW_Transmit_Byte(uint8_t data) {
    EUSCI_B0_SPI_Data_Write(data);
}

void PicoW_Transmit_Bytes(int n, uint8_t *data) {
    // Enable the chip select line
    EUSCI_B0_Control_Chip_Select(0x00);

    for (int i = 0; i < n; i++)
    {
        // Write to the TX buffer and transmit the data from cmd_buffer to the PMOD JSTK2 module
        PicoW_Transmit_Byte(data[i]);
    }

    Clock_Delay1ms(30);

    // Disable the chip select line
    EUSCI_B0_Control_Chip_Select(0x01);

    // At least 25 us is required before chip select can be driven low to initiate another communication session
    Clock_Delay1us(25);
}


