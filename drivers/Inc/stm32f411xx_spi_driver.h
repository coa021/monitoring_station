#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

// Configuration structure for SPIx peripheral
typedef struct {
	// TODO: Check if i did this correctly, im looking at Control Register 1, did i miss something?
	uint8_t SPI_DeviceMode;			// MSTR, 0 = slave, 1 = Master
	uint8_t SPI_BusConfig;// Bus config, controls bits 15, 14, 10 (BIDIMODE, BIDIOE, RXONLY)=>(Full-duplex, half-duplex, or simplex)
	uint8_t SPI_SclkSpeed;    		// Clock prescaler value (0-7)
	uint8_t SPI_DFF;          		// DFF - 0 = 8-bit, 1 = 16-bit
	uint8_t SPI_CPOL;         		// Clock polarity
	uint8_t SPI_CPHA;         		// Clock phase
	uint8_t SPI_SSM;          		// Software slave management
} SPI_Config_t;

// Handle structure for SPIx peripheral
typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
} SPI_Handle_t;

// SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0
// SPI_BusConfig
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY		3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		4
// SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7
// SPI_DFF
#define SPI_DFF_8BITS			0
#define SPI_DFF_16BITS			1
// SPI_CPOL
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0
// SPI_CPHA
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0
// SPI_SSM
#define SPI_SSM_EN			1
#define SPI_SSM_DI			0

// SPI related status flags
#define SPI_TXE_FLAG			( 1<< SPI_SR_TXE )
#define SPI_RXNE_FLAG			( 1<< SPI_SR_RXNE )
#define SPI_BUSY_FLAG			( 1<< SPI_SR_BSY )

// Possible SPI application states
#define SPI_READY				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2

// possible SPI applcation events
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4

//
// APIs section
// clock
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// init, deinit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,
		uint32_t Length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,
		uint32_t Length);

// IRQ config and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

// other peripheral control APIs

// enable, disable SPIx
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// application callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

// helpers
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
