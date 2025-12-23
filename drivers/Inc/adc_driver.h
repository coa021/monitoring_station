#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "stm32f411xx.h"
#include <stdint.h>

void ADC_Init(void);
uint16_t ADC_Read(uint8_t channel);


#endif //ADC_DRIVER_H
