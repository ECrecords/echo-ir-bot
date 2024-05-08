/*
 * PicoW.
 *
 *  Created on: May 4, 2024
 *      Author: elvis
 */

#ifndef INC_PICOW_
#define INC_PICOW_

#include "../inc/EUSCI_B0_SPI.h"

void PicoW_Init();

void PicoW_Transmit_Byte(uint8_t data) ;

void PicoW_Transmit_Bytes(int n, uint8_t *mes);

#endif /* INC_PICOW_C_ */
