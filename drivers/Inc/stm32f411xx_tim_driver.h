#ifndef INC_STM32F411XX_TIM_DRIVER_H_
#define INC_STM32F411XX_TIM_DRIVER_H_

#include "stm32f411xx.h"

// timer config structure
typedef struct {
  uint32_t TIM_Prescaler;        // clock prescaler
  uint32_t TIM_Period;           // auto reload value, 16 or 32bits
  uint8_t TIM_CounterMode;       // @TIM_CounterMode
  uint8_t TIM_AutoReloadPreload; // @Time_AutoReloadPreload
} TIM_Config_t;

// timer handle structure
typedef struct {
  TIM_RegDef_t *pTIMx;       // base addr of timer peripheral
  TIM_Config_t TIM_Config; // timer config settings
} TIM_Handle_t;

/*
 * @TIM_CounterMode
 */
#define TIM_COUNTERMODE_UP 0
#define TIM_COUNTERMODE_DOWN 1
#define TIM_COUNTERMODE_CENTER_ALIGNED1 2
#define TIM_COUNTERMODE_CENTER_ALIGNED2 3
#define TIM_COUNTERMODE_CENTER_ALIGNED3 4

/*
 * @TIM_AutoReloadPreload
 */
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0
#define TIM_AUTORELOAD_PRELOAD_ENABLE 1

// API section

// clock
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnOrDi);

// init deinit
void TIM_Init(TIM_Handle_t *pTIMHandle);
void TIM_DeInit(TIM_RegDef_t *pTIMx);

// timer control
void TIM_Start(TIM_RegDef_t *pTIMx);
void TIM_Stop(TIM_RegDef_t *pTIMx);
uint32_t TIM_GetCounter(TIM_RegDef_t *pTIMx);
void TIM_SetCounter(TIM_RegDef_t *pTIMx, uint32_t value);

// interrupt config and handling
void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void TIM_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void TIM_ClearITPendingBit(TIM_RegDef_t *pTIMx, uint16_t TIM_IT);

void TIM_ITConfig(TIM_RegDef_t *pTIMx, uint16_t TIM_IT, uint8_t EnOrDi);

// flag status
uint8_t TIM_GetFlagStatus(TIM_RegDef_t *pTIMx, uint16_t TIM_FLAG);

// Some APIs im gonna need for this, i dont want to define them in main
void TIM2_Delay_Init(void);
void TIM2_Delay_us(uint32_t us);
void TIM2_Delay_ms(uint32_t ms);

// periodic interrupt setup using tim3
void TIM3_PeriodicInterrupt_Init(uint32_t period_ms);
void TIM3_SetPeriod(uint32_t period_ms);

// flags
#define TIM_IT_UPDATE (1 << TIM_DIER_UIE)
#define TIM_IT_CC1 (1 << TIM_DIER_CC1IE)
#define TIM_IT_CC2 (1 << TIM_DIER_CC2IE)
#define TIM_IT_CC3 (1 << TIM_DIER_CC3IE)
#define TIM_IT_CC4 (1 << TIM_DIER_CC4IE)
#define TIM_IT_TRIGGER (1 << TIM_DIER_TIE)

#define TIM_FLAG_UPDATE (1 << TIM_SR_UIF)
#define TIM_FLAG_CC1 (1 << TIM_SR_CC1IF)
#define TIM_FLAG_CC2 (1 << TIM_SR_CC2IF)
#define TIM_FLAG_CC3 (1 << TIM_SR_CC3IF)
#define TIM_FLAG_CC4 (1 << TIM_SR_CC4IF)
#define TIM_FLAG_TRIGGER (1 << TIM_SR_TIF)

#endif /* INC_STM32F411XX_TIM_DRIVER_H_ */