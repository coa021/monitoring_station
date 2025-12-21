#include "stm32f411xx_gpio_driver.h"

// APIs section

// clock
/*
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- Enables or Disables peripheral clock for the given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macro
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {		// enable
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	} else {					// disable
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}

// init, deinit
/*
 * @fn				- GPIO_Init
 *
 * @brief			- Configures given GPIO pin
 *
 * @param[in]		- Base Address and user configuration of GPIO peripheral
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	// enable the peri clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	uint32_t temp = 0;		// temp register
	// configure the mode of gpio pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// clearing the bits before setting
		pGPIOHandle->pGPIOx->MODER |= temp;	// setting the bits
	} else {
		// TODO: interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// configure both
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the RTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// configure gpio port selection in SYSCFG_EXTICR

		uint8_t extiRegisterIndex = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber
				/ 4);
		uint8_t extiBitIndex = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);

		uint8_t portcode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[extiRegisterIndex] = portcode << (extiBitIndex * 4);

		// enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// configure speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// clearing the bits before setting
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;	// set
	temp = 0;

	// conf pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// clearing the bits before setting
	pGPIOHandle->pGPIOx->PUPDR |= temp;	// set

	temp = 0;

	// conf op type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// clearing the bits before setting

	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// conf alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FUN) {
		// TODO:
		uint8_t afrIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t bitIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[afrIndex] &= ~(0xF << (4 * bitIndex));	// clear

		pGPIOHandle->pGPIOx->AFR[afrIndex] |=
				pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode
						<< (4 * bitIndex); // set
	}

	temp = 0;
}

/*
 * @fn				- GPIO_DeInit
 *
 * @brief			- Resets register for a given GPIO port
 *
 * @param[in]		- Address of GPIO peripheral
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

// data read and write

/*
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- Reads a value of GPIO pin for a given Port and Pin number
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin Number
 *
 * @return			- uint8_t, return read data from the input pin
 *
 * @Note			- None
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- Reads the given GPIO Port
 *
 * @param[in]		- Base address of the GPIO peripheral
 *
 * @return			- uint8_16t, returns read data from port
 *
 * @Note			- None
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t) pGPIOx->IDR;
}

/*
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- Writes a value to a given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin Number
 * @param[in]		- Value to write
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value) {

	if (Value == GPIO_PIN_SET) {
		// write 1
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		// write 0
		pGPIOx->ODR &= ~(1 << PinNumber);

	}
}

/*
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- Writes a value to a register for a given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Value to write
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/*
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- Toggles a pin for a given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin Number
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

// irq config and handling

/*
 * @fn				- GPIO_IRQInterruptConfig
 *
 * @brief			- Configures an exception
 *
 * @param[in]		- IRQ number of an exception
 * @param[in]		- ENABLE or DISABLE flag
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
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
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}

/*
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- Handles an exception for a given GPIO pin
 *
 * @param[in]		- Pin number
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
	// clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber)) {
		// clear
		EXTI->PR |= (1 << PinNumber);
	}
}

/*
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			- Handles an exception for a given GPIO pin
 *
 * @param[in]		- IRQ number of an exception
 * @param[in]		- IRQ priority of an exception
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// find IPR register
	uint8_t iprx_index = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx_index) |= (IRQPriority << shift_amount);

}

