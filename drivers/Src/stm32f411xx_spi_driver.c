#include "stm32f411xx.h"
#include "stm32f411xx_spi_driver.h"

// APIs section

// helper fn's
// private helper functions, not declared in header

/*
 *  @fn				-	SPI_TXE_Interrupt_Handle
 *
 *  @brief			-	Helper function of SPI TXE interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle) {
	// Check DFF bit in cr1
	if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
		// 16 bit
		// load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		// we sent 2 bits of info
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*) pSPIHandle->pTxBuffer++;	// moves by 2
	} else {
		// 8 bit
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		// we sent 1 bit  of info
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;	// moves by 1
	}

	if (!pSPIHandle->TxLen) {
		// TxLen is zero, close SPI communicatoin; inform the app that Tx is over
		// deactivate Txeie bit
		SPI_CloseTransmission(pSPIHandle);

		// inform the app through callback
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}
}

/*
 *  @fn				-	SPI_RXNE_Interrupt_Handle
 *
 *  @brief			-	helper function of SPI RXNE interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle) {
	// do Rx as per the DFF
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16bit
		*((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
	} else {
		// 8bit
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->RxLen) {
		// Rx is complete
		// turn off RXNEIE interrupt
		SPI_CloseReception(pSPIHandle);

		// inform the app through callback
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/*
 *  @fn				-	SPI_OVR_Err_Interrupt_Handle
 *
 *  @brief			-	Helper function of SPI OVR interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void SPI_OVR_Err_Interrupt_Handle(SPI_Handle_t *pSPIHandle) {
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	// inform the app
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

// clock
/*
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- Enables or Disables peripheral clock for the given SPIx
 *
 * @param[in]		- Base address of the SPIx peripheral
 * @param[in]		- ENABLE or DISABLE macro
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {		// enable
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		} else if (pSPIx == SPI5) {
			SPI5_PCLK_EN();
		}
	} else {					// disable
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		} else if (pSPIx == SPI5) {
			SPI5_PCLK_DI();
		}
	}
}

// init, deinit
/*
 * @fn				- SPI_Init
 *
 * @brief			- Configures given SPIx
 *
 * @param[in]		- Base Address and user configuration of SPIx peripheral
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {

	// enable the clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempReg = 0;

	//TODO: Define bit macros for SPI

	// configure device mode
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// configure bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// bidir mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// bidir mode should be set
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig
			== SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// bidir mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		// rxonly bit must be set
		tempReg |= (1 << SPI_CR1_RXONLY);
	}

	// configure spi serial clock speed
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// configure dff
	tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// configure CPOL
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// configure CPHA
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// configure ssm
	tempReg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// set
	pSPIHandle->pSPIx->CR1 = tempReg;
}

/*
 * @fn				- SPI_DeInit
 *
 * @brief			- Resets register for a given SPIx
 *
 * @param[in]		- Address of SPI peripheral
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	} else if (pSPIx == SPI5) {
		SPI5_REG_RESET();
	}
}

// helper function to get the flag status
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName)
		return FLAG_SET;
	return FLAG_RESET;
}

// Data send and receive
/*
 * @fn				- SPI_SendData
 *
 * @brief			- Send's data with certain Length from the given SPIx peripheral using Tx buffer
 *
 * @param[in]		- Address of SPI peripheral
 * @param[in]		- Tx buffer
 * @param[in]		- Data length
 *
 * @return			- None
 *
 * @Note			- Blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length) {
	while (Length > 0) {
		// wait for tx buffer to be empty
		// wait until TXE bit is 1
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		// Check DFF bit in cr1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			// 16 bit
			// load the data in to the DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			// we sent 2 bits of info
			Length--;
			Length--;
			(uint16_t*) pTxBuffer++;	// moves by 2
		} else {
			// 8 bit
			pSPIx->DR = *pTxBuffer;
			// we sent 1 bit  of info
			Length--;
			pTxBuffer++;	// moves by 1
		}
	}
}

/*
 * @fn				- SPI_ReceiveData
 *
 * @brief			- Receives the data from Rx buffer for SPIx
 *
 * @param[in]		- Address of SPI peripheral
 * @param[in]		- Rx buffer
 * @param[in]		- Data length
 *
 * @return			- None
 *
 * @Note			- None
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length) {
	while (Length > 0) {
		// wait until RXNE bit is 1
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
			;

		// Check DFF bit in cr1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			// 16 bit
			// load the data from DR ot Rx buffer address
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			// we sent 2 bits of info
			Length--;
			Length--;
			(uint16_t*) pRxBuffer++;	// moves by 2
		} else {
			// 8 bit
			*pRxBuffer = pSPIx->DR;
			// we sent 1 bit  of info
			Length--;
			pRxBuffer++;	// moves by 1
		}
	}
}

/*
 * @fn				- SPI_SendDataIT
 *
 * @brief			- Receives the data from Rx buffer for SPIx with Interrupt/IRQ
 *
 * @param[in]		- Address of SPI Handle
 * @param[in]		- Rx buffer
 * @param[in]		- Data length
 *
 * @return			- None
 *
 * @Note			- None
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,
		uint32_t Length) {

	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_BUSY_IN_TX) {

		// save tx buffer addr and length info
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Length;

		// mark the spi state as busy in Tx so that other code cant take over same SPI peripheral until Tx is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// enable TXEIE control bit to get interrupt whnever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}

/*
 * @fn				- SPI_ReceiveDataIT
 *
 * @brief			- Receives the data from Rx buffer for SPIx with Interrupt/IRQ
 *
 * @param[in]		- Address of SPI handle
 * @param[in]		- Rx buffer
 * @param[in]		- Data length
 *
 * @return			- None
 *
 * @Note			- None
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,
		uint32_t Length) {

	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX) {

		// save tx buffer addr and length info
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Length;

		// mark the spi state as busy in Tx so that other code cant take over same SPI peripheral until Tx is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// enable TXEIE control bit to get interrupt whnever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}

// IRQ config and ISR handling
/*
 *  @fn				-	SPI_IRQInterruptConfig
 *
 *  @brief			-	This function configures an exception
 *
 *  @param[in]		-	IRQ number of an exception
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
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
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));	// Bug fixed 32=>64
		}
	}
}

/*
 *  @fn				-	SPI_IRQHandling
 *
 *  @brief			-	This function handles an exception for a given SPI pin
 *
 *  @param[in]		-	Pin number
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

	// why irq happened? check TXE
	uint8_t status = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	uint8_t status_ie = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	// if both true, then irq is because of setting txe flag
	if (status && status_ie) {
		// handle txe
		SPI_TXE_Interrupt_Handle(pSPIHandle);	// TODO: helper fn
	}

	// check for RXNE
	status = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	status_ie = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	// if both true, then irq is because of setting RXNE flag
	if (status && status_ie) {
		// handle RXNE
		SPI_RXNE_Interrupt_Handle(pSPIHandle);	// TODO: helper fn
	}

	// check for Overrun error/flag
	status = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	status_ie = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	// if both true, then irq is because of setting overrun flag
	if (status && status_ie) {
		// handle OVRRN (?)
		SPI_OVR_Err_Interrupt_Handle(pSPIHandle);	// TODO: helper fn
	}
}

/*
 *  @fn				-	SPI_IRQPriorityConfig
 *
 *  @brief			-	This function configures a priority of an exception
 *
 *  @param[in]		-	IRQ number of an exception
 *  @param[in]		-	IRQ priority of an exception
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// find IPR register
	uint8_t iprx_index = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx_index) |= (IRQPriority << shift_amount);

}

// enable, disable SPIx
/*
 *  @fn				-	SPI_PeripheralControl
 *
 *  @brief			-	This function enables/disables SPIx peripheral
 *
 *  @param[in]		-	SPIx peripheral
 *  @param[in]		-	ENABLE or DISABLE flag
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE)
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

/*
 *  @fn				-	SPI_SSIConfig
 *
 *  @brief			-	SSI config; Makes NSS signal internally high and avoids MODF error
 *
 *  @param[in]		-	SPIx peripheral
 *  @param[in]		-	ENABLE or DISABLE flag
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE)
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}

/*
 *  @fn				-	SPI_SSOEConfig
 *
 *  @brief			-	SSOE config; Makes NSS signal internally high and avoids MODF error
 *
 *  @param[in]		-	SPIx peripheral
 *  @param[in]		-	ENABLE or DISABLE flag
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE)
		// bug fix? cr1 to cr2 TODO:
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	else
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	// clear OVR flag
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);// prevent interrupts from setting up txe flag
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

//
__weak void SPI_ApplicationEventCallback(
		SPI_Handle_t *pSPIHandle, uint8_t AppEv) {
	// weak impl. the app my override this

}
