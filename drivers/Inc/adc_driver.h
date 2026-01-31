#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "stm32f411xx.h"
#include <cstdint>
#include <stdint.h>

// ADC configuration structure
typedef struct {
  uint8_t ADC_Resolution;      // @ADC_Resolution
  uint8_t ADC_ScanMode;        // @ADC_ScanMode
  uint8_t ADC_ContinuousMode;  // @ADC_ContinuousMode
  uint8_t ADC_DataAlign;       // @ADC_DataAlign
  uint8_t ADC_NumOfConversion; // @ADC_NumOfConversion
  uint8_t ADC_ExternalTrigger; // @ADC_ExternalTrigger
} ADC_Config_t;

typedef struct {
  ADC_TypeDef *pADCx;        // Base address of ADC peripheral
  ADC_Config_t ADC_Config;   // ADC config settings
} ADC_Handle_t;

typedef struct {
	uint8_t ADC_Channel;			// @ADC_Channel
	uint8_t ADC_SamplingTime;		// @ADC_SamlingTime
	uint8_t ADC_Rank;				// @ADC_Rank
} ADC_ChannelConfig_t;


/*
 * @ADC_Resolution
 */
#define ADC_RESOLUTION_12BIT		0
#define ADC_RESOLUTION_10BIT		1
#define ADC_RESOLUTION_8BIT			2
#define ADC_RESOLUTION_6BIT			3

/*
 * @ADC_ScanMode
 */
#define ADC_SCAN_DISABLE			0
#define ADC_SCAN_ENABLE				1

/*
 * @ADC_ContinuousMode
 */
#define ADC_CONTINUOUS_DISABLE		0
#define ADC_CONTINUOUS_ENABLE		1

/*
 * @ADC_DataAlign
 */
#define ADC_DATAALIGN_RIGHT			0
#define ADC_DATAALIGN_LEFT			1

/*
 * @ADC_ExternalTrigger
 */
#define ADC_EXTERNALTRIG_NONE		0
#define ADC_EXTERNALTRIG_RISING		1
#define ADC_EXTERNALTRIG_FALLING	2
#define ADC_EXTERNALTRIG_BOTH		3

/*
 * @ADC_Channel
 */
#define ADC_CHANNEL_0				0
#define ADC_CHANNEL_1				1
#define ADC_CHANNEL_2				2
#define ADC_CHANNEL_3				3
#define ADC_CHANNEL_4				4
#define ADC_CHANNEL_5				5
#define ADC_CHANNEL_6				6
#define ADC_CHANNEL_7				7
#define ADC_CHANNEL_8				8
#define ADC_CHANNEL_9				9
#define ADC_CHANNEL_10				10
#define ADC_CHANNEL_11				11
#define ADC_CHANNEL_12				12
#define ADC_CHANNEL_13				13
#define ADC_CHANNEL_14				14
#define ADC_CHANNEL_15				15
#define ADC_CHANNEL_16				16		// Internal temperature sensor
#define ADC_CHANNEL_17				17		// Internal VREFINT
#define ADC_CHANNEL_18				18		// VBAT/4

/*
 * @ADC_SamplingTime 
 */
#define ADC_SAMPLETIME_3CYCLES		0
#define ADC_SAMPLETIME_15CYCLES		1
#define ADC_SAMPLETIME_28CYCLES		2
#define ADC_SAMPLETIME_56CYCLES		3
#define ADC_SAMPLETIME_84CYCLES		4
#define ADC_SAMPLETIME_112CYCLES	5
#define ADC_SAMPLETIME_144CYCLES	6
#define ADC_SAMPLETIME_480CYCLES	7

/*
 * ADC flags
 */
#define ADC_FLAG_AWD				(1 << ADC_SR_AWD)
#define ADC_FLAG_EOC				(1 << ADC_SR_EOC)
#define ADC_FLAG_JEOC				(1 << ADC_SR_JEOC)
#define ADC_FLAG_JSTRT				(1 << ADC_SR_JSTRT)
#define ADC_FLAG_STRT				(1 << ADC_SR_STRT)
#define ADC_FLAG_OVR				(1 << ADC_SR_OVR)

// API Section

// Peripheral clock control
void ADC_PeriClockControl(ADC_TypeDef *pADCx, uint8_t EnOrDi);

// inti deinit
void ADC_Init(ADC_Handle_t *pADCHandle);
void ADC_DeInit(ADC_TypeDef *pADCx);

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

#endif // ADC_DRIVER_H
