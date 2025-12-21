/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: Dec 16, 2025
 *      Author: coa
 */

#include "stm32f411xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint16_t APB1_PreScaler[4] = { 2, 4, 8, 16 };

// static helpers declaration
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

// static helpers definition
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr <<= 1;	// make space for R/W
	SlaveAddr &= ~(1);	// slave address + read/write bit
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr <<= 1;	// make space for R/W
	SlaveAddr |= 1;	// slave address + read/write bit
	pI2Cx->DR = SlaveAddr;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	uint8_t dummy_read;
	// check device mode
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {	// MASTER MODE
		// check app state
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if (pI2CHandle->RxSize == 1) {
				// first disable ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

			}
		}
	}

	// clear the addr flag by reading SR1 then SR2
	dummy_read = pI2CHandle->pI2Cx->SR1;
	dummy_read = pI2CHandle->pI2Cx->SR1;
	(void) dummy_read;
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {
//	if (pI2CHandle->RxLen > 0) {  // Only process if data remaining

	// handle data reception
	if (pI2CHandle->RxSize == 1) {
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1) {
		if (pI2CHandle->RxLen == 2) {
			// clear ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}
		// read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxLen == 0) {
		// close i2c data reception and notify the app
		// generate stop condition
		if (pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// close i2c rx
		I2C_CloseReceiveData(pI2CHandle);

		// notify the app
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
//	}
//else {
//		// RxLen is 0 but RXNE still firing - disable ITBUFEN
//		pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
//	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) {

	if (pI2CHandle->TxLen > 0) {
		// load the data in DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		// decrement TxLen
		pI2CHandle->TxLen--;
		// increment buffer addr
		pI2CHandle->pTxBuffer++;
	}
//	else {		// TODO: Temp fix
//		// TxLen is 0 but TXE still firing - disable ITBUFEN temporarily
//		// BTF handler will re-enable if needed
//		pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
//	}
}

// APIs section
// clock
/*
 * @fn				- I2C_PeriClockControl
 *
 * @brief			- Enables or Disables peripheral clock for the given I2Cx
 *
 * @param[in]		- Base address of the I2Cx peripheral
 * @param[in]		- ENABLE or DISABLE macro
 *
 * @return			- None
 *
 * @Note			- None
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {		// enable
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {				// disable
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

uint32_t RCC_GetPLLOutputClock(void) {
	return 16000000; // 16Mhz
}

uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, SystemClk;

	uint8_t clkSrc = ((RCC->CFGR >> 2) & 0x3);
	uint8_t prescalerTemp, ahbp, apb1;

	if (clkSrc == 0) {
		// hsi
		SystemClk = 16000000;	// 16Mhz
	} else if (clkSrc == 1) {
		// hse
		SystemClk = 8000000;	// 8Mhz
	} else if (clkSrc == 2) {
		// decided by pll
		SystemClk = RCC_GetPLLOutputClock();
	}

	// calculating AHB1 prescaler
	prescalerTemp = ((RCC->CFGR >> 4) & 0xF); // HPRE: AHB prescaler bit

	if (prescalerTemp < 8) {
		// system clock not divided
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[prescalerTemp - 8];
	}

	// reusing variable to calculate APB1 prescaler
	prescalerTemp = ((RCC->CFGR >> 10) & 0x7); // PPRE1: APB1 prescaler bit

	if (prescalerTemp < 4) {
		// system clock not divided
		apb1 = 1;
	} else {
		apb1 = APB1_PreScaler[prescalerTemp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1;

	return pclk1;
}

// init, deinit
/*
 * @fn				- I2C_Init
 *
 * @brief			- Configures given I2Cx
 *
 * @param[in]		- Base Address and user configuration of I2Cx peripheral
 *
 * @return			- None
 *
 * @Note			- None
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	// ! remember to disable the peripheral before configuring anything

	uint32_t tempreg = 0;

	// enable clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// ack ctrl bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);

	// configure freq field in CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U; // Divide with 1Mhz to get the 16 or whatever only
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);	// mask out 0-5 bits only

	// program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1; // we dont care for first bit, bcz we are sending 7 bit info
	tempreg |= (1 << 14); // we need to keep bit 14 as 1 always as per manual->18.6.3 I2C_OAR1 section
	pI2CHandle->pI2Cx->OAR1 = tempreg;
	// we dont configure ADDMODE of OAR1, bcz its always zero by default which is needed

	// we dont configure CR2, not needed
	// CCR now
	uint16_t ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// standard mode
		// bit 15 already 0 here, no need to configure
		ccr_value = RCC_GetPCLK1Value()
				/ (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	} else {
		// fast mode
		// set bit 15 to fast mode
		tempreg |= (1 << I2C_CCR_FS);
		// get DUTY bit
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		// calculate ccr value based on formulas
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = RCC_GetPCLK1Value()
					/ (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else {
			ccr_value = RCC_GetPCLK1Value()
					/ (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
	tempreg = 0;
	// TRISE register
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;	// dividing by 1 Mhz
	} else {
		// fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U);	// div by 1 nano
	}
	pI2CHandle->pI2Cx->TRISE = tempreg & 0x3F;
}

/*
 * @fn				- I2C_DeInit
 *
 * @brief			- Resets register for a given I2Cx
 *
 * @param[in]		- Address of I2Cx peripheral
 *
 * @return			- None
 *
 * @Note			- None
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

// Data send and receive
// TODO: later

/*
 *  @fn				-	I2C_MasterSendData
 *
 *  @brief			-	TBD
 *
 *  @param[in]		-	TBD
 *  @param[in]		-	TBD
 *  @param[in]		-	TBD
 *  @param[in]		-	TBD
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	// generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm that start generation is completed by checking the SB flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		// we read sr1, now whats left is to write to DR
		;

	// send the address of the slave with read/write bit se to write (0)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// confirm address phase is completed by checking ADDR flag
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;
	// clear the addr flag
	I2C_ClearADDRFlag(pI2CHandle);

	// send data until Len = 0
	while (Len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
			;	// wait for TXE to be set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	// Len = 0, close communication
	// wait for txe=1 and btf=1 before generating STOP condition
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
		;	// wait for TXE to be set
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF))
		;	// wait for BTF to be set

	// generate STOP condition
	if (Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/*
 *  @fn				-	I2C_MasterReceiveData
 *
 *  @brief			-	Used for receiving data
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *  @param[in]		-	Slave address
 *
 *  @return			-	none
 *
 *  @Note			-	This is blocking call
 *
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint8_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	// generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// check sr1->sb_flag to confirm start generation is completed
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		;

	// send the addr of slave with r/w bit set r-1
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// wait until addr phase is completed by checking sr1->addr_flag
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;

	// procedure to read only 1 byte from slave
	if (Len == 1) {
		// disable acking
//		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// clear the addr flag
		I2C_ClearADDRFlag(pI2CHandle);

		// wait until rxne is 1
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
			;

		// bug fix
		// generate stop condition
		if (Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		// read data to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if (Len > 1) {
		// clear addr flag
		I2C_ClearADDRFlag(pI2CHandle);

		// read until Len = 0;
		while (Len > 0) {
			// wait until rxne is 1
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
				;

			if (Len == 2) {
				// clear ack bit, disable it
//				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// generate stop condition
				if (Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// read data from DR in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			// increment buffer addr
			pRxBuffer++;
			// decrement len
			Len--;
		}

	}

	// re-enable acking
	//	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/*
 *  @fn				-	I2C_MasterSendDataIT
 *
 *  @brief			-	Used for receiving data
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *  @param[in]		-	Slave address
 *
 *  @return			-	none
 *
 *  @Note			-	This is blocking call
 *
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	uint8_t busy_state = pI2CHandle->TxRxState;

	if ((busy_state != I2C_BUSY_IN_TX) && (busy_state != I2C_BUSY_IN_RX)) {
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// enable itbufen control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// enable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// enable iterren control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busy_state;
}

/*
 *  @fn				-	I2C_MasterReceiveDataIT
 *
 *  @brief			-	Used for receiving data
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *  @param[in]		-	Slave address
 *
 *  @return			-	none
 *
 *  @Note			-	This is blocking call
 *
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint8_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	uint8_t busy_state = pI2CHandle->TxRxState;

	if ((busy_state != I2C_BUSY_IN_TX) && (busy_state != I2C_BUSY_IN_RX)) {
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;	// used in ISR code to manage data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// enable itbufen control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// enable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// enable iterren control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busy_state;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data) {
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx) {
	return (uint8_t) pI2Cx->DR;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi == I2C_ACK_ENABLE) {

		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {

		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
// IRQ config and ISR handling
/*
 *  @fn				-	I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
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
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*
 *  @fn				-	I2C_IRQPriorityConfig
 *
 *  @brief			-	This function configures priority of an exception
 *
 *  @param[in]		-	IRQ number of an exception
 *  @param[in]		-	IRQ priority of an exception
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * IPRx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IPRx) |= (IRQPriority << shift_amount);
}

/*
 *  @fn				-	I2C_EV_IRQHandling
 *
 *  @brief			-	Handles an exception of events for a given I2C pin
 *
 *  @param[in]		-	Base address and user configuration of the I2Cx peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)	// event
{
	// interrupt handling for both master and slave mode of device
	uint32_t itevten_flag, itbufen_flag, temp_flag;
	itevten_flag = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	itbufen_flag = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	// handle for interrupt generated by SB event; SB only applicable in master mode
//	temp_flag = pI2CHandle->pI2Cx->SR1; // temp read to reset, dont read this line because i just added it now after your comment
	temp_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	if (itevten_flag && temp_flag) {
		// sb flag is set
		// execute address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,
					pI2CHandle->DevAddr);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	// handle for ir generated by ADDR event
	// for master mode, addr is sent; when slave mode, addr matched with own addr
	temp_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if (itevten_flag && temp_flag) {
		// ADDR flag is set
		// clear the flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// handle for ir generated by BTF event
	temp_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if (itevten_flag && temp_flag) {
		// BTF flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			// make sure TXE is also set
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) {
				if (pI2CHandle->TxLen == 0) {// make sure we sent all the data
					// btf AND txe is set, now close transmission
					// generate stop condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					// reset all member elemnets of handle structure
					I2C_CloseSendData(pI2CHandle);

					// notify the app Tx is complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);

					//TODO:
				}
			}
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			// nothing to do here, dont close anything
		}
	}

	// handle for interrupt generated by STOPF event
	// stop detetion flag is applicable only in slave mode
	temp_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if (itevten_flag && temp_flag) {
		// STOPF flag is set
		// clear stopf by reading SR1 and writing to CR1
		// we already did reading of SR1 above, now write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// notify app that STOP is detected by master
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	// handle for interrupt generated by TXE event
	temp_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if (itevten_flag && itbufen_flag && temp_flag) {
		// ONLY do this if device is in master mode
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			// TXE flag is set
			// do data transmission
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
//			else {
//				// State mismatch - disable ITBUFEN and reset
//				pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
//				pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
//				pI2CHandle->TxRxState = I2C_READY;
//			}
		} else {
			// slave mode
			if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) {
				// make sure we are in tx mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}

		}
	}

	// handle for interrupt generated by RXNE event
	temp_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if (itevten_flag && itbufen_flag && temp_flag) {
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {

			// RXNE flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		} else {
			// slave mode
			if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))) {
				// make sure we are in rx mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}

}

/*
 *  @fn				-	I2C_ER_IRQHandling
 *
 *  @brief			-	Handles an exception of errors for a given I2C pin
 *
 *  @param[in]		-	Base address and user configuration of the I2Cx peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)	// error
{
	uint32_t flag, iterren_flag;

	// get status of ITERREN bit
	iterren_flag = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	// CHECK FOR BUS ERROR
	flag = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if (flag && iterren_flag) {
		// bus error
		// clear buss error flag
		(pI2CHandle->pI2Cx->SR1) &= ~(1 << I2C_SR1_BERR);
		// notify the app about error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	// CHECK ARBITRATION LOST ERROR
	flag = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if (flag && iterren_flag) {
		// arbitration lost error
		// clear arbitration lost error flag
		(pI2CHandle->pI2Cx->SR1) &= ~(1 << I2C_SR1_ARLO);
		// notify the app about error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	// ACK FAILURE ERROR
	flag = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if (flag && iterren_flag) {
		// Ack failure error
		// clear ack failure flag
		(pI2CHandle->pI2Cx->SR1) &= ~(1 << I2C_SR1_AF);
		// notify the app about error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	// OVERRUN/UNDERRUN ERROR
	flag = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if (flag && iterren_flag) {
		// overrun underrun error
		// clear the flag
		(pI2CHandle->pI2Cx->SR1) &= ~(1 << I2C_SR1_OVR);
		// notify the app about error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	// TIME OUT ERROR
	flag = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if (flag && iterren_flag) {
		// timeout error
		// clear timeout error flag
		(pI2CHandle->pI2Cx->SR1) &= ~(1 << I2C_SR1_TIMEOUT);
		// notify the app about error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

// other peripheral control APIs

// enable, disable I2Cx
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

// application callback
//void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {
//
//}

// helpers
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
	if (pI2Cx->SR1 & FlagName)
		return FLAG_SET;
	return FLAG_RESET;
}

/*
 *  @fn				-	I2C_CloseReceiveData
 *
 *  @brief			-	This function closes I2C reception
 *
 *  @param[in]		-	Base address and user configuration of the I2Cx peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
	// disable ITBUFEN bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// disable ITEVTEN bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// reset state
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
}

/*
 *  @fn				-	I2C_CloseSendData
 *
 *  @brief			-	This function closes I2C transmission
 *
 *  @param[in]		-	Base address and user configuration of the I2Cx peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
	// disable ITBUFEN bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// disable ITEVTEN bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// reset state
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	} else {
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}
