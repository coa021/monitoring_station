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
void ADC_ConfigChannel(ADC_TypeDef *pADCx,
                       ADC_ChannelConfig_t *pChannelConfig) {
  uint8_t channel = pChannelConfig->ADC_Channel;
  uint8_t sampleTime = pChannelConfig->ADC_SamplingTime;
  uint8_t rank = pChannelConfig->ADC_Rank;

  // configure sampling time
  if (channel < 10) {
    // channels 0-9 use smpr2
    pADCx->SMPR2 &= ~(7 << (3 * channel));
    pADCx->SMPR2 |= (sampleTime << (3 * channel));
  } else {
    // channels 10-18 use smpr1
    pADCx->SMPR1 &= ~(7 << (3 * (channel - 10)));
    pADCx->SMPR1 |= (sampleTime << (3 * (channel - 10)));
  }

  // configure sequence register
  // rank 1-6 is sqr3
  // rank 7-12 is sqr2
  // rank 13-16 is sqr1
  if (rank <= 6) {
    pADCx->SQR3 &= ~(0x1F << (5 * (rank - 1)));
    pADCx->SQR3 |= (channel << (5 * (rank - 1)));
  } else if (rank <= 12) {
    pADCx->SQR2 &= ~(0x1F << (5 * (rank - 7)));
    pADCx->SQR2 |= (channel << (5 * (rank - 7)));
  } else {
    pADCx->SQR1 &= ~(0x1F << (5 * (rank - 13)));
    pADCx->SQR1 |= (channel << (5 * (rank - 13)));
  }

  // enable temperature sensor and vrefint
  if (channel == ADC_CHANNEL_16 || channel == ADC_CHANNEL_17)
    ADC->CCR |= (1 << 23); // tsvrefe

  if (channel == ADC_CHANNEL_18)
    ADC->CCR |= (1 << 22); // vbate
}

// ADC control
/*
 * @fn          ADC_Enable
 * @brief       Enable the ADC peripheral
 * @param[in]   pADCx - Base address of ADC peripheral
 * @return      none
 */
void ADC_Enable(ADC_TypeDef *pADCx) {
  pADCx->CR2 |= (1 << ADC_CR2_ADON);

  // wait a bit for stabilization
  for (volatile int i = 0; i < 200; i++)
    ;
}

/*
 * @fn          ADC_Disable
 * @brief       Disable the ADC peripheral
 * @param[in]   pADCx - Base address of ADC peripheral
 * @return      none
 */
void ADC_Disable(ADC_TypeDef *pADCx) { pADCx->CR2 &= ~(1 << ADC_CR2_ADON); }

/*
 * @fn          ADC_StartConversion
 * @brief       Start ADC conversion (software trigger)
 * @param[in]   pADCx - Base address of ADC peripheral
 * @return      none
 */
void ADC_StartConversion(ADC_TypeDef *pADCx) {
  pADCx->CR2 &= ~(1 << ADC_CR2_ADON);
}

/*
 * @fn          ADC_GetConversionValue
 * @brief       Get ADC conversion result
 * @param[in]   pADCx - Base address of ADC peripheral
 * @return      Conversion result (12-bit max)
 */
uint16_t ADC_GetConversionValue(ADC_TypeDef *pADCx) {
  return (uint16_t)pADCx->DR;
}

// polling mode read
/*
 * @fn          ADC_ReadChannel
 * @brief       Read a single ADC channel (blocking)
 * @param[in]   pADCx - Base address of ADC peripheral
 * @param[in]   channel - Channel number (0-18)
 * @return      Conversion result
 */
uint16_t ADC_ReadChannel(ADC_TypeDef *pADCx, uint8_t channel) {
  // config for rank 1
  ADC_ChannelConfig_t channelConfig;
  channelConfig.ADC_Channel = channel;
  channelConfig.ADC_SamplingTime = ADC_SAMPLETIME_84CYCLES;
  channelConfig.ADC_Rank = 1;
  ADC_ConfigChannel(pADCx, &channelConfig);

  // start conversion
  ADC_StartConversion(pADCx);

  // wait for end of conversion
  while (!(pADCx->SR & ADC_FLAG_EOC))

    return ADC_GetConversionValue(pADCx);
}

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