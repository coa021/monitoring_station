#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "stm32f411xx.h"
#include <stdint.h>


// ADC configuration structure
typedef struct {
	uint8_t ADC_Resolution;					// @ADC_Resolution
	uint8_t ADC_ScanMode;					// @ADC_ScanMode
	uint8_t ADC_ContinuousMode;				// @ADC_ContinuousMode
	uint8_t ADC_DataAlign;					// @ADC_DataAlign
	uint8_t ADC_NumOfConversion;			// @ADC_NumOfConversion
	uint8_t ADC_ExternalTrigger;			// @ADC_ExternalTrigger
} ADC_Config_t;

typedef struct 
{
	ADC_TypeDef *pADCx;			// Base address of ADC peripheral
	ADC_Config_t ADC_Config_t;	// ADC config settings
} ADC_Handle_t
;



#endif //ADC_DRIVER_H
