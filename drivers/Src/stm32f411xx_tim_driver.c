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

  // start the timer
  pTIMHandle->pTIMx->CR1 |= (1 << TIM_CR1_CEN);
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

/*
 * @fn          TIM_Start
 * @brief       Start the timer counter
 * @param[in]   pTIMx - Base address of timer peripheral
 * @return      none
 */
void TIM_Start(TIM_RegDef_t *pTIMx) { pTIMx->CR1 |= (1 << TIM_CR1_CEN); }

/*
 * @fn          TIM_Stop
 * @brief       Stop the timer counter
 * @param[in]   pTIMx - Base address of timer peripheral
 * @return      none
 */
void TIM_Stop(TIM_RegDef_t *pTIMx) { pTIMx->CR1 &= ~(1 << TIM_CR1_CEN); }

/*
 * @fn          TIM_GetCounter
 * @brief       Get current counter value
 * @param[in]   pTIMx - Base address of timer peripheral
 * @return      Current counter value
 */
uint32_t TIM_GetCounter(TIM_RegDef_t *pTIMx) { return pTIMx->CNT; }

/*
 * @fn          TIM_SetCounter
 * @brief       Set counter value
 * @param[in]   pTIMx - Base address of timer peripheral
 * @param[in]   value - Value to set
 * @return      none
 */
void TIM_SetCounter(TIM_RegDef_t *pTIMx, uint32_t value) { pTIMx->CNT = value; }

// interrupt config and handling

/*
 * @fn          TIM_IRQInterruptConfig
 * @brief       Enable or disable timer interrupt in NVIC
 * @param[in]   IRQNumber - IRQ number
 * @param[in]   EnOrDi - ENABLE or DISABLE
 * @return      none
 */
void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
  if (EnOrDi == ENABLE) {
    // we are enabling
    if (IRQNumber <= 31) {
      // program ISER0 register
      *NVIC_ISER0 |= (1 << IRQNumber);
    } else if (IRQNumber > 31 && IRQNumber < 64) {
      // program ISER1 register
      *NVIC_ISER1 |= (1 << (IRQNumber % 32));

    } else if (IRQNumber >= 64 && IRQNumber < 96) {
      // program ISER2 register
      *NVIC_ISER2 |= (1 << (IRQNumber % 64));
    }
  } else {
    // we are cleaning/disabling
    if (IRQNumber <= 31) {
      // program ICER0 register
      *NVIC_ICER0 |= (1 << IRQNumber);
    } else if (IRQNumber > 31 && IRQNumber < 64) {
      // program ICER1 register
      *NVIC_ICER1 |= (1 << (IRQNumber % 32));
    } else if (IRQNumber >= 64 && IRQNumber < 96) {
      // program ICER2 register
      *NVIC_ICER2 |= (1 << (IRQNumber % 64)); // Bug fixed 32=>64
    }
  }
}

/*
 * @fn          TIM_IRQPriorityConfig
 * @brief       Configure interrupt priority
 * @param[in]   IRQNumber - IRQ number
 * @param[in]   IRQPriority - Priority value (0-15 for 4 bits implemented)
 * @return      none
 */
void TIM_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
  // find IPR register
  uint8_t iprx_index = IRQNumber / 4;
  uint8_t iprx_section = IRQNumber % 4;

  uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
  *(NVIC_PR_BASE_ADDR + iprx_index) |= (IRQPriority << shift_amount);
}

/*
 * @fn          TIM_ClearITPendingBit
 * @brief       Clear interrupt pending bit
 * @param[in]   pTIMx - Base address of timer peripheral
 * @param[in]   TIM_IT - Interrupt flag to clear
 * @return      none
 */
void TIM_ClearITPendingBit(TIM_RegDef_t *pTIMx, uint16_t TIM_IT) {
  pTIMx->SR &= ~TIM_IT;
}

/*
 * @fn          TIM_ITConfig
 * @brief       Enable or disable timer interrupt sources
 * @param[in]   pTIMx - Base address of timer peripheral
 * @param[in]   TIM_IT - Interrupt source (TIM_IT_UPDATE, etc.)
 * @param[in]   EnOrDi - ENABLE or DISABLE
 * @return      none
 */
void TIM_ITConfig(TIM_RegDef_t *pTIMx, uint16_t TIM_IT, uint8_t EnOrDi) {
  if (EnOrDi == ENABLE)
    pTIMx->DIER |= TIM_IT;
  else
    pTIMx->DIER &= ~TIM_IT;
}

// flag status

/*
 * @fn          TIM_GetFlagStatus
 * @brief       Get status of a timer flag
 * @param[in]   pTIMx - Base address of timer peripheral
 * @param[in]   TIM_FLAG - Flag to check
 * @return      FLAG_SET or FLAG_RESET
 */
uint8_t TIM_GetFlagStatus(TIM_RegDef_t *pTIMx, uint16_t TIM_FLAG) {
  if (pTIMx->SR & TIM_FLAG)
    return FLAG_SET;
  return FLAG_RESET;
}

/*
 * @fn          TIM_ClearFlag
 * @brief       Clear a timer status flag
 * @param[in]   pTIMx - Base address of TIM peripheral
 * @param[in]   TIM_FLAG - Flag to clear
 * @return      none
 */
void TIM_ClearFlag(TIM_RegDef_t *pTIMx, uint16_t TIM_FLAG) {
  pTIMx->SR &= ~TIM_FLAG;
}

// Some APIs im gonna need for this, i dont want to define them in main
static volatile uint8_t tim2_initialized = 0;

/*
 * @fn          TIM2_Delay_Init
 * @brief       Initialize TIM2 for microsecond delays
 * @param       none
 * @return      none
 */
void TIM2_Delay_Init(void) {
  if (tim2_initialized)
    return;

  TIM_Handle_t tim2_handle;
  tim2_handle.pTIMx = TIM2;
  tim2_handle.TIM_Config.TIM_Prescaler =
      15; // (16MHz / (15+1)) = 1MHz = 1us ticks
  tim2_handle.TIM_Config.TIM_Period = 0xffffffff;
  tim2_handle.TIM_Config.TIM_CounterMode = TIM_COUNTERMODE_UP;
  tim2_handle.TIM_Config.TIM_AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  TIM_Init(&tim2_handle);

  // start the timer
  TIM2->CR1 |= (1 << TIM_CR1_CEN);

  tim2_initialized = 1;
}

/*
 * @fn          TIM2_Delay_us
 * @brief       Blocking delay in microseconds
 * @param[in]   us - Delay in microseconds
 * @return      none
 */
void TIM2_Delay_us(uint32_t us) {
  if (!tim2_initialized)
    TIM2_Delay_Init();

  uint32_t start = TIM2->CNT;

  while ((TIM2->CNT - start) < us)
    ; // wait
}

/*
 * @fn          TIM2_Delay_ms
 * @brief       Blocking delay in milliseconds
 * @param[in]   ms - Delay in milliseconds
 * @return      none
 */
void TIM2_Delay_ms(uint32_t ms) {
  while (ms--)
    TIM2_Delay_us(1000);
}

// periodic interrupt setup using tim3

/*
 * @fn          TIM3_PeriodicInterrupt_Init
 * @brief       Initialize TIM3 for periodic interrupts
 * @param[in]   period_ms - Period in milliseconds (1 to 65535)
 * @return      none
 */
void TIM3_PeriodicInterrupt_Init(uint32_t period_ms) {
  TIM_InitPeriodicInterrupt(TIM3, period_ms, 2);
}

void TIM_InitPeriodicInterrupt(TIM_RegDef_t *pTIMx, uint32_t period_ms,
                               uint8_t priority) {
  if (period_ms > 65535)
    period_ms = 65535;
  if (period_ms == 0)
    period_ms = 1;

  TIM_Handle_t tim;
  tim.pTIMx = pTIMx;
  tim.TIM_Config.TIM_Prescaler = 15999;
  tim.TIM_Config.TIM_Period = period_ms - 1;
  tim.TIM_Config.TIM_CounterMode = TIM_COUNTERMODE_UP;
  tim.TIM_Config.TIM_AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  TIM_Init(&tim);

  // enable update interrupt
  TIM_ITConfig(pTIMx, TIM_IT_UPDATE, ENABLE);

  // configure NVIC
  uint8_t irq_number;
  if (pTIMx == TIM2)
    irq_number = IRQ_NO_TIM2;
  else if (pTIMx == TIM3)
    irq_number = IRQ_NO_TIM3;
  else if (pTIMx == TIM4)
    irq_number = IRQ_NO_TIM4;
  else if (pTIMx == TIM5)
    irq_number = IRQ_NO_TIM5;
  else
    return; // i dont have that timer xd

  TIM_IRQInterruptConfig(irq_number, ENABLE);
  TIM_IRQPriorityConfig(irq_number, priority);

  // Start timer
  TIM_Start(pTIMx);
}

void TIM3_SetPeriod(uint32_t period_ms);
