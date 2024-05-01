
#ifndef _PICOW_I2C_SLAVE_H
#define _PICOW_I2C_SLAVE_H

#include "EUSCI_B1_I2C.h"

typedef struct {
    uint8_t slaveAddress;
} picow_instance_t;

typedef union {
    struct {
        uint32_t distance;
        uint32_t angle;
    };
    uint8_t bytes[8];
} radar_data_t;

void PicoW_Init();

void PicoW_SendRadarData(radar_data_t *data);

void PicoW_I2C_Slave_ReceiveData(uint8_t *data, uint8_t length);

#endif // _PICOW_I2C_SLAVE_H
