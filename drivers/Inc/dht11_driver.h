#ifndef DHT11_DRIVER_H
#define DHT11_DRIVER_H

#include "stm32f411xx.h"
#include <stdint.h>


typedef struct {
    uint8_t humidity_int;    // humidity integer part
    uint8_t humidity_dec;    // huimdity decimal part
    uint8_t temperature_int;
    uint8_t temperature_dec;
    uint8_t checksum;
} DHT11_Data_t;

void DHT11_Init(GPIO_RegDef_t *port, uint8_t pin);
uint8_t DHT11_Read(GPIO_RegDef_t *port, uint8_t pin, DHT11_Data_t *data);

#endif //DHT11_DRIVER_H
