#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "stm32f411xx.h"
#include <stdint.h>

// TODO: since i didnt get to MCU2 from fastbit, ill do a quick implementation of ADC peripheral drivers,
// but ill revisit this once i complete that part of the course

void ADC_Init(void);
uint16_t ADC_Read(uint8_t channel);


#endif //ADC_DRIVER_H
