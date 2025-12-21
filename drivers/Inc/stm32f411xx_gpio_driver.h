#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

// Config struct for GPIO pin
typedef struct {
	uint8_t GPIO_PinNumber;					// Possible values @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;					// Possible values @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;					// Possible values @GPIO_SPEED
	uint8_t GPIO_PinPuPdControl;			// Possible values @GPIO_PU_PD_CONFIG
	uint8_t GPIO_PinOPType;					// Possible values @GPIO_PIN_OP_TYPE
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

// Handle struct for GPIO Pin
typedef struct {
	GPIO_RegDef_t *pGPIOx; 				// base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	// holds GPIO pin config settings
} GPIO_Handle_t;


// @GPIO_PIN_NUMBERS
// GPIO pin numbers
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


// @GPIO_PIN_MODES
// GPIO pin possible modes
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALT_FUN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4	// Falling Edge
#define GPIO_MODE_IT_RT		5 	// Rising Edge
#define GPIO_MODE_IT_RFT	6	// Rising edge, Falling edge trigger

// @GPIO_PIN_OP_TYPE
// GPIO pin possible output types
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

// @GPIO_SPEED
// GPIO pin possible speeds
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

// @GPIO_PU_PD_CONFIG
// GPIO pin possible Pull up and Pull down configurations
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

// APIs section

// clock
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

// init, deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// irq config and handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
