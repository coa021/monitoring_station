#include "stm32f411xx_tim_driver.h"

#define SYSTEM_CLOCK_HZ 16000000U
#define APB1_TIMER_CLOCK_HZ SYSTEM_CLOCK_HZ
#define APB2_TIMER_CLOCK_HZ SYSTEM_CLOCK_HZ

// API section

// clock
/*
 * @fn          TIM_PeriClockControl
 * @brief       Enables or disables peripheral clock for the given timer
 * @param[in]   pTIMx - Base address of timer peripheral
 * @param[in]   EnOrDi - ENABLE or DISABLE macros
 * @return      none
 */
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnOrDi) {
  if (EnOrDi == ENABLE) {
    if (pTIMx == TIM1)
      TIM1_PCLK_EN();
    else if (pTIMx == TIM2)
      TIM2_PCLK_EN();
    else if (pTIMx == TIM3)
      TIM3_PCLK_EN();
    else if (pTIMx == TIM4)
      TIM4_PCLK_EN();
    else if (pTIMx == TIM5)
      TIM5_PCLK_EN();
    else if (pTIMx == TIM9)
      TIM9_PCLK_EN();
    else if (pTIMx == TIM10)
      TIM10_PCLK_EN();
    else if (pTIMx == TIM11)
      TIM11_PCLK_EN();
  } else {
    if (pTIMx == TIM1)
      TIM1_PCLK_DI();
    else if (pTIMx == TIM2)
      TIM2_PCLK_DI();
    else if (pTIMx == TIM3)
      TIM3_PCLK_DI();
    else if (pTIMx == TIM4)
      TIM4_PCLK_DI();
    else if (pTIMx == TIM5)
      TIM5_PCLK_DI();
    else if (pTIMx == TIM9)
      TIM9_PCLK_DI();
    else if (pTIMx == TIM10)
      TIM10_PCLK_DI();
    else if (pTIMx == TIM11)
      TIM11_PCLK_DI();
  }
}

// init deinit
/*
 * @fn          TIM_Init
 * @brief       Initialize the timer with given configuration
 * @param[in]   pTIMHandle - Pointer to timer handle structure
 * @return      none
 */
void TIM_Init(TIM_Handle_t *pTIMHandle) {
  TIM_PeriClockControl(pTIMHandle->pTIMx, ENABLE);

  // disable timer during config
  pTIMHandle->pTIMx->CR1 &= ~(1 << TIM_CR1_CEN);

  // configure prescaler
  pTIMHandle->pTIMx->PSC = pTIMHandle->TIM_Config.TIM_Prescaler;

  // configure auto reload vlaue
  pTIMHandle->pTIMx->ARR = pTIMHandle->TIM_Config.TIM_Period;

  // configure counter mode
  pTIMHandle->pTIMx->CR1 &= ~(1 << TIM_CR1_DIR); // clear direction bit
  pTIMHandle->pTIMx->CR1 &= ~(3 << TIM_CR1_CMS); // clear center aligned bits

  // up down center aligned
  if (pTIMHandle->TIM_Config.TIM_CounterMode == TIM_COUNTERMODE_DOWN)
    pTIMHandle->pTIMx->CR1 |= (1 << TIM_CR1_DIR);
  else if (pTIMHandle->TIM_Config.TIM_CounterMode >=
           TIM_COUNTERMODE_CENTER_ALIGNED1) {
    uint8_t val = pTIMHandle->TIM_Config.TIM_CounterMode -
                  TIM_COUNTERMODE_CENTER_ALIGNED1 + 1;
    pTIMHandle->pTIMx->CR1 |= (val << TIM_CR1_CMS);
  }

  // configure auto reload preload
  if (pTIMHandle->TIM_Config.TIM_AutoReloadPreload ==
      TIM_AUTORELOAD_PRELOAD_ENABLE)
    pTIMHandle->pTIMx->CR1 |= (1 << TIM_CR1_ARPE);
  else
    pTIMHandle->pTIMx->CR1 &= ~(1 << TIM_CR1_ARPE);

  // generate update event to load prescaler and ARR values
  pTIMHandle->pTIMx->EGR |= (1 << TIM_EGR_UG);

  // clear update interrupt flag
  pTIMHandle->pTIMx->SR &= ~(1 << TIM_SR_UIF);
}

/*
 * @fn          TIM_DeInit
 * @brief       Reset the timer peripheral
 * @param[in]   pTIMx - Base address of timer peripheral
 * @return      none
 */
void TIM_DeInit(TIM_RegDef_t *pTIMx) {
  // resetting timers
  if (pTIMx == TIM1) {
    RCC->APB2RSTR |= (1 << 0);
    RCC->APB2RSTR &= ~(1 << 0);
  } else if (pTIMx == TIM2) {
    RCC->APB1RSTR |= (1 << 0);
    RCC->APB1RSTR &= ~(1 << 0);
  } else if (pTIMx == TIM3) {
    RCC->APB1RSTR |= (1 << 1);
    RCC->APB1RSTR &= ~(1 << 1);
  } else if (pTIMx == TIM4) {
    RCC->APB1RSTR |= (1 << 2);
    RCC->APB1RSTR &= ~(1 << 2);
  } else if (pTIMx == TIM5) {
    RCC->APB1RSTR |= (1 << 3);
    RCC->APB1RSTR &= ~(1 << 3);
  } else if (pTIMx == TIM9) {
    RCC->APB2RSTR |= (1 << 16);
    RCC->APB2RSTR &= ~(1 << 16);
  } else if (pTIMx == TIM10) {
    RCC->APB2RSTR |= (1 << 17);
    RCC->APB2RSTR &= ~(1 << 17);
  } else if (pTIMx == TIM11) {
    RCC->APB2RSTR |= (1 << 18);
    RCC->APB2RSTR &= ~(1 << 18);
  }
}

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