#include "adc_driver.h"

// API Section

// Peripheral clock control
void ADC_PeriClockControl(ADC_TypeDef *pADCx, uint8_t EnOrDi) {
  if (EnOrDi == ENABLE) {
    if (pADCx == ADC1)
      ADC1_PCLK_EN();
  } else {
    if (pADCx == ADC1)
      ADC1_PCLK_DI();
  }
}

/*
 * @fn          ADC_Init
 * @brief       Initialize ADC with given configuration
 * @param[in]   pADCHandle - Pointer to ADC handle structure
 * @return      none
 */
void ADC_Init(ADC_Handle_t *pADCHandle) {
  uint32_t tempreg = 0;

  ADC_PeriClockControl(pADCHandle->pADCx, ENABLE);

  // configure C1
  tempreg = pADCHandle->pADCx->CR1;
  // resolution
  tempreg &= ~(3 << ADC_CR1_RES);
  tempreg |= (pADCHandle->ADC_Config.ADC_Resolution << ADC_CR1_RES);

  // scan mode
  if (pADCHandle->ADC_Config.ADC_ScanMode == ADC_SCAN_ENABLE)
    tempreg |= (1 << ADC_CR1_SCAN);
  else
    tempreg &= ~(1 << ADC_CR1_SCAN);

  pADCHandle->pADCx->CR1 = tempreg;

  tempreg = pADCHandle->pADCx->CR2;

  // data align
  if (pADCHandle->ADC_Config.ADC_DataAlign == ADC_DATAALIGN_LEFT)
    tempreg |= (1 << ADC_CR2_ALIGN);
  else
    tempreg &= ~(1 << ADC_CR2_ALIGN);

  // continuous mode
  if (pADCHandle->ADC_Config.ADC_ContinuousMode == ADC_CONTINUOUS_ENABLE)
    tempreg |= (1 << ADC_CR2_CONT);
  else
    tempreg &= ~(1 << ADC_CR2_CONT);

  // external trigger, software trigger = 00
  tempreg &= ~(3 << ADC_CR2_EXTEN);
  if (pADCHandle->ADC_Config.ADC_ExternalTrigger != ADC_EXTERNALTRIG_NONE)
    tempreg |= (pADCHandle->ADC_Config.ADC_ExternalTrigger << ADC_CR2_EXTEN);

  // end of conversion selection
  tempreg &= ~(1 << ADC_CR2_EOCS);
  pADCHandle->pADCx->CR2 = tempreg;

  // configure num of conversions in regular sequence
  tempreg = pADCHandle->pADCx->SQR1;
  tempreg &= ~(0xF << 20);
  tempreg |= ((pADCHandle->ADC_Config.ADC_NumOfConversion - 1) << 20);
  pADCHandle->pADCx->SQR1 = tempreg;

  // configure ADC prescaler
  ADC->CCR &= (3 << 16);
}

/*
 * @fn          ADC_DeInit
 * @brief       Reset the ADC peripheral
 * @param[in]   pADCx - Base address of ADC peripheral
 * @return      none
 */
void ADC_DeInit(ADC_TypeDef *pADCx) {
  if (pADCx == ADC1) {
    RCC->APB1RSTR |= (1 << 8);
    RCC->APB1RSTR &= ~(1 << 8);
  }
}

// channel config
void ADC_ConfigChannel(ADC_TypeDef *pADCx, ADC_ChannelConfig_t *pChannelConfig);

// ADC control
void ADC_Enable(ADC_TypeDef *pADCx);
void ADC_Disable(ADC_TypeDef *pADCx);
void ADC_StartConversion(ADC_TypeDef *pADCx);
uint16_t ADC_GetConversionValue(ADC_TypeDef *pADCx);

// polling mode read
uint16_t ADC_ReadChannel(ADC_TypeDef *pADCx, uint8_t channel);

// interrupt config
void ADC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void ADC_ITConfig(ADC_TypeDef *pADCx, uint32_t ADC_IT, uint8_t EnOrDi);

// flag status
uint8_t ADC_GetFlagStatus(ADC_TypeDef *pADCx, uint32_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef *pADCx, uint32_t ADC_FLAG);

// simplified api helpers

// quick init
void ADC_SimpleInit(void);

// read single channel
uint16_t ADC_Read(uint8_t channel);