#ifndef DHT11_DRIVER_H
#define DHT11_DRIVER_H

#include "stm32f411xx.h"
#include <stdint.h>

// pins
#define DHT11_PORT                GPIOB
#define DHT11_PIN                 GPIO_PIN_NO_0

// status codes
#define DHT11_OK                  0
#define DHT11_ERROR               1
#define DHT11_TIMEOUT_ERROR       2

#define DHT11_TIMEOUT_MS          1000

typedef struct {
    uint8_t humidity_int;    // humidity integer part
    uint8_t humidity_dec;    // huimdity decimal part
    uint8_t temperature_int;
    uint8_t temperature_dec;
    uint8_t checksum;
} DHT11_Data_t;

void DHT11_Init(void);
uint8_t DHT11_Read(DHT11_Data_t *data);

#endif //DHT11_DRIVER_H
