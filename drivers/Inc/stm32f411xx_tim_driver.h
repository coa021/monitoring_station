#ifndef INC_STM32F411XX_TIM_DRIVER_H_
#define INC_STM32F411XX_TIM_DRIVER_H_

#include "stm32f411xx.h"


// timer config structure
typedef struct {
    uint32_t TIM_Prescaler;                 // clock prescaler
    uint32_t TIM_Period;                    // auto reload value, 16 or 32bits
    uint8_t TIM_CounterMode;                // @TIM_CounterMode
    uint8_t TIM_AutoReloadPreload;          // @Time_AutoReloadPreload
} TIM_Config_t;

// timer handle structure
typedef struct {
    TIM_RegDef_t *pTIMx;            // base addr of timer peripheral
    TIM_Config_t TIM_Config_t;      // timer config settings
} TIM_Handle_t;


#endif /* INC_STM32F411XX_TIM_DRIVER_H_ */