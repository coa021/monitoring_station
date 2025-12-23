#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stddef.h>
#include <stdint.h>
#define __vo volatile
#define __weak __attribute__((weak))


// ARM Cortex Mx processor NVIC ISERx register addresses
#define NVIC_ISER0 ( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1 ( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2 ( (__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3 ( (__vo uint32_t*) 0xE000E10C )

// ARM Cortex Mx processor NVIC ICERx register addresses
#define NVIC_ICER0 ( (__vo uint32_t*) 0xE000E180 )
#define NVIC_ICER1 ( (__vo uint32_t*) 0xE000E184 )
#define NVIC_ICER2 ( (__vo uint32_t*) 0xE000E188 )
#define NVIC_ICER3 ( (__vo uint32_t*) 0xE000E18C )


//todo: FINISH THIS
#define NVIC_PRIO_15		15

// ARM Cortex Mx processor NVIC priority register address calculation
#define NVIC_PR_BASE_ADDR	( (__vo uint32_t*) 0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED 		4


// base addresses of flash and sram
#define FLASH_BASEADDR  	0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		// don't have
#define ROM_BASE_ADDR		0x1FFF0000U
#define OTP_AREA			0x1FFF7800U
#define SRAM 				SRAM1_BASEADDR

#define PERIPH_BASE_ADDR			0X40000000U
#define AHB1PERIPH_BASE_ADDR		0x40020000U
#define AHB2PERIPH_BASE_ADDR 		0x50000000U
#define APB1PERIPH_BASE_ADDR		PERIPH_BASE_ADDR
#define APB2PERIPH_BASE_ADDR		0x40010000U

#define RCC_BASE_ADDR				(AHB1PERIPH_BASE_ADDR + 0x3800)

// Base addresses of peripherals on AHB1 bus
// GPIO's
#define GPIOA_BASE_ADDR				(AHB1PERIPH_BASE_ADDR + 0x00)
#define GPIOB_BASE_ADDR				(AHB1PERIPH_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR				(AHB1PERIPH_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR				(AHB1PERIPH_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR				(AHB1PERIPH_BASE_ADDR + 0x1000)
#define GPIOH_BASE_ADDR				(AHB1PERIPH_BASE_ADDR + 0x1C00)

// Base addresses of peripherals on APB1 bus
#define TIM2_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x00)
#define TIM3_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x0400)
#define TIM4_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x0800)
#define TIM5_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x0C00)
#define RTC_BKP_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x2800)
#define WWDG_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x2C00)
#define IWDG_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x3000)
#define I2S2ext_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x3400)
#define SPI2_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x3800)
#define I2S2_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x3C00)
#define I2S3_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x3C00)
#define I2S3ext_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x4000)
#define USART2_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x4400)
#define I2C1_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x5C00)
#define PWR_BASE_ADDR				(APB1PERIPH_BASE_ADDR + 0x7000)

// Base addresses of peripherals on APB2 bus

#define TIM1_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x00)
#define USART1_BASE_ADDR			(APB2PERIPH_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR			(APB2PERIPH_BASE_ADDR + 0x1400)
#define ADC1_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x2000)
#define SDIO_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x2C00)
#define SPI1_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x3000)
#define I2S1_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x3000)
#define SPI4_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x3400)
#define I2S4_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x3400)
#define SYSCFG_BASE_ADDR			(APB2PERIPH_BASE_ADDR + 0x3800)
#define EXTI_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x3C00)
#define TIM9_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x4000)
#define TIM10_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x4400)
#define TIM11_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x4800)
#define SPI5_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x5000)		// TODO: Check if the splitting was ok thing to do, probably not but still
#define I2S5_BASE_ADDR				(APB2PERIPH_BASE_ADDR + 0x5000)		// TODO: same for other SPIx_I2Sx base addresses

// Peripheral register definition structures

// GPIO
typedef struct {
	__vo uint32_t MODER;	//	GPIO mode register 											0x00
	__vo uint32_t OTYPER;	//	GPIO Port output type register 								0x04
	__vo uint32_t OSPEEDR;	//	GPIO port output speed register 							0x08
	__vo uint32_t PUPDR;	//	GPIO port pull up pull down register 						0x0c
	__vo uint32_t IDR;		//	GPIO port input data register 								0x10
	__vo uint32_t ODR;		//	GPIO port output data register 								0x14
	__vo uint32_t BSRR;		//	GPIO port bit set/reset register 							0x18
	__vo uint32_t LCKR;		//	GPIO port configuration lock register 						0x1c
	__vo uint32_t AFR[2];	//	GPIO alternate function low AFR[0] / high AFR[1]  register 	0x20-0x24
} GPIO_RegDef_t;

// RCC
typedef struct {
	__vo uint32_t CR;				// Clock Control register 									0x00
	__vo uint32_t PLLCFGR;			// PLL configuration register 								0x04
	__vo uint32_t CFGR;				// Clock configuration register								0x08
	__vo uint32_t CIR;				// Clock interrupt register 								0x0c
	__vo uint32_t AHB1RSTR;			// AHB1 peripheral reset register 							0x10
	__vo uint32_t AHB2RSTR;			// AHB2 peripheral reset register 							0x14
	uint32_t _reserved0;			// 															0x18
	uint32_t _reserved1;			// 															0x1c
	__vo uint32_t APB1RSTR;			// APB1 peripheral reset register 							0x20
	__vo uint32_t APB2RSTR;			// APB2 peripheral reset register 							0x24
	uint32_t _reserved2;			// 															0x28
	uint32_t _reserved3;			// 															0x2c
	__vo uint32_t AHB1ENR;			// AHB1 peripheral clock enable 							0x30
	__vo uint32_t AHB2ENR;			// AHB2 peripheral clock enable 							0x3c
	uint32_t _reserved4;			// 															0x38
	uint32_t _reserved5;			// 															0x3c
	__vo uint32_t APB1ENR;			// APB1 peripheral clock enable 							0x40
	__vo uint32_t APB2ENR;			// APB2 peripheral clock enable 							0x44
	uint32_t _reserved6;			// 															0x48
	uint32_t _reserved7;			// 															0x4c
	__vo uint32_t AHB1LPENR;		// AHB1 peripheral clock enable in low power mode register 	0x50
	__vo uint32_t AHB2LPENR;		// AHB2 peripheral clock enable in low power mode register 	0x54
	uint32_t _reserved8;			// 															0x58
	uint32_t _reserved9;			// 															0x5c
	__vo uint32_t APB1LPENR;		// APB1 peripheral clock enable in low power mode register 	0x60
	__vo uint32_t APB2LPENR;		// APB2 peripheral clock enable in low power mode register 	0x64
	uint32_t _reserved10;			// 															0x68
	uint32_t _reserved11;			// 															0x6c
	__vo uint32_t BDCR;				// Backup domain control register 							0x70
	__vo uint32_t CSR;				// Clock control & status register 							0x74
	uint32_t _reserved12;			// 															0x78
	uint32_t _reserved13;			// 															0x7c
	__vo uint32_t SSCGR;			// Spread spectrum clock generation register 				0x80
	__vo uint32_t PLLI2SCFGR;		// PLLI2S configuration register 							0x84
	uint32_t _reserved14;			// 															0x88
	__vo uint32_t DCKCFGR;			// Dedicated Clocks configuration register 					0x8c
} RCC_RegDef_t;


// Peripheral register definition structure for EXTI
typedef struct {
	__vo uint32_t IMR;		//	Interrupt mask register									0x00
	__vo uint32_t EMR;		//	Event mask register										0x04
	__vo uint32_t RTSR;		//	Rising trigger selection register						0x08
	__vo uint32_t FTSR;		//	Falling trigger selection register						0x0c
	__vo uint32_t SWIER;	//	Software interrupt event register						0x10
	__vo uint32_t PR;		//	Pending register			 							0x14
} EXTI_RegDef_t;


// Peripheral register definition structure for SYSCFG
typedef struct {
	__vo uint32_t CR1;			// Control Register 1 					0x00
	__vo uint32_t CR2;			// Control Register 2 					0x04
	__vo uint32_t SR;			// Status Register						0x08
	__vo uint32_t DR;			// Data register						0x0c
	__vo uint32_t CRCPR;		// CRC polynomial register				0x10
	__vo uint32_t RXCRCR;		// RX CRC register						0x14
	__vo uint32_t TXCRCR;		// TX CRC register						0x18
	__vo uint32_t I2SCFGR;		// I2S configuration register			0x1c
	__vo uint32_t I2SPR;		// I2S prescaler register				0x20
} SPI_RegDef_t;

// Peripheral register definition structure for SYSCFG
typedef struct {
	__vo uint32_t MEMRMP;		//	SYSCFG memory remap register									0x00
	__vo uint32_t PMC;			//	SYSCFG peripheral mode ocnfiguration register 					0x04
	__vo uint32_t EXTICR[4];	//	SYSCFG external interrupt configuration register 1-4			0x08, 0x0c, 0x10, 0x14
	uint32_t _reserved0;		// 	Reserved														0x18
	uint32_t _reserved1;		// 	Reserved														0x1c
	__vo uint32_t CMPCR;		// Compensation cell control register								0x20
} SYSCFG_RegDef_t;

// Peripheral register definition structure for I2C
typedef struct {
	__vo uint32_t CR1;			// Control register 1												0x00
	__vo uint32_t CR2;			// Control register 2												0x04
	__vo uint32_t OAR1;			// Own Address Register 1											0x08
	__vo uint32_t OAR2;			// Own Address Register 2											0x0C
	__vo uint32_t DR;			// Data register													0x10
	__vo uint32_t SR1;			// Status register 1												0x14
	__vo uint32_t SR2;			// Status register 2												0x18
	__vo uint32_t CCR;			// Clock control register											0x1C
	__vo uint32_t TRISE;		// TRISE Register (rise time)										0x20
	__vo uint32_t FLTR;			// FLTR register (digital/analog filter register)					0x24
} I2C_RegDef_t;



// TODO: temp solution, will revisit once i finish whole chapter
// ADC individual peripheral registers
typedef struct {
    __vo uint32_t SR;      // ADC status register                       0x00
    __vo uint32_t CR1;     // ADC control register 1                    0x04
    __vo uint32_t CR2;     // ADC control register 2                    0x08
    __vo uint32_t SMPR1;   // ADC sample time register 1                0x0C
    __vo uint32_t SMPR2;   // ADC sample time register 2                0x10
    __vo uint32_t JOFR1;   // ADC injected channel offset 1             0x14
    __vo uint32_t JOFR2;   // ADC injected channel offset 2             0x18
    __vo uint32_t JOFR3;   // ADC injected channel offset 3             0x1C
    __vo uint32_t JOFR4;   // ADC injected channel offset 4             0x20
    __vo uint32_t HTR;     // ADC watchdog high threshold               0x24
    __vo uint32_t LTR;     // ADC watchdog low threshold                0x28
    __vo uint32_t SQR1;    // ADC regular sequence register 1           0x2C
    __vo uint32_t SQR2;    // ADC regular sequence register 2           0x30
    __vo uint32_t SQR3;    // ADC regular sequence register 3           0x34
    __vo uint32_t JSQR;    // ADC injected sequence register            0x38
    __vo uint32_t JDR1;    // ADC injected data register 1              0x3C
    __vo uint32_t JDR2;    // ADC injected data register 2              0x40
    __vo uint32_t JDR3;    // ADC injected data register 3              0x44
    __vo uint32_t JDR4;    // ADC injected data register 4              0x48
    __vo uint32_t DR;      // ADC regular data register                 0x4C
} ADC_TypeDef;

// adc common registers, shared between all ADC peripherals
typedef struct {
    __vo uint32_t CSR;     // ADC Common status register            0x00
    __vo uint32_t CCR;     // ADC Common control register           0x04
    __vo uint32_t CDR;     // ADC Common regular data register      0x08
} ADC_Common_TypeDef;

// Peripheral definitions
#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASE_ADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASE_ADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASE_ADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASE_ADDR)
#define SPI4				((SPI_RegDef_t*)SPI4_BASE_ADDR)
#define SPI5				((SPI_RegDef_t*)SPI5_BASE_ADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASE_ADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASE_ADDR)

// TODO: Temp solution, will revisit this in near future
#define ADC1                ((ADC_TypeDef *)ADC1_BASE_ADDR)
#define ADC_COMMON_BASE     (APB2PERIPH_BASE_ADDR + 0x2300)
#define ADC                 ((ADC_Common_TypeDef *)ADC_COMMON_BASE)

// Clock enable macro for GPIOx
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1<<4) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1<<7) )


// Clock enable for I2Cx peripherals
#define I2C1_PCLK_EN()		( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= (1<<23) )

// Clock enable for SPIx peripherals
#define SPI1_PCLK_EN()		( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= (1<<15) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= (1<<13) )
#define SPI5_PCLK_EN()		( RCC->APB2ENR |= (1<<20) )

// Clock enable for USARTx peripherals
#define USART1_PCLK_EN()	( RCC->APB2ENR |= (1<<4) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= (1<<5) )

// Clock enable for SYSCFG peripheral
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1<<14) )


// Clock disable macros for GPIOx peripherals
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~(1<<4) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~(1<<7) )

// Clock disable for I2Cx peripherals
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~(1<<22) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~(1<<23) )

// Clock disable for SPIx peripherals
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~(1<<12) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~(1<<14) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~(1<<15) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~(1<<13) )
#define SPI5_PCLK_DI()		( RCC->APB2ENR &= ~(1<<20) )

// Clock disable for USARTx peripherals
#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~(1<<4) )
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~(1<<5) )

// Clock disable for SYSCFG peripheral
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1<<14) )



// Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1<<0) );  ( RCC->AHB1RSTR &= ~(1<<0) ); } while(0)
#define GPIOB_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1<<1) );  ( RCC->AHB1RSTR &= ~(1<<1) ); } while(0)
#define GPIOC_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1<<2) );  ( RCC->AHB1RSTR &= ~(1<<2) ); } while(0)
#define GPIOD_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1<<3) );  ( RCC->AHB1RSTR &= ~(1<<3) ); } while(0)
#define GPIOE_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1<<4) );  ( RCC->AHB1RSTR &= ~(1<<4) ); } while(0)
#define GPIOH_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1<<7) );  ( RCC->AHB1RSTR &= ~(1<<7) ); } while(0)

// Macros to reset SPIx peripherals
#define SPI1_REG_RESET()		do{ ( RCC->APB2RSTR |= (1<<12) );  ( RCC->APB2RSTR &= ~(1<<12) ); } while(0)
#define SPI2_REG_RESET()		do{ ( RCC->APB1RSTR |= (1<<14) );  ( RCC->APB1RSTR &= ~(1<<14) ); } while(0)
#define SPI3_REG_RESET()		do{ ( RCC->APB1RSTR |= (1<<15) );  ( RCC->APB1RSTR &= ~(1<<15) ); } while(0)
#define SPI4_REG_RESET()		do{ ( RCC->APB2RSTR |= (1<<13) );  ( RCC->APB2RSTR &= ~(1<<13) ); } while(0)
#define SPI5_REG_RESET()		do{ ( RCC->APB2RSTR |= (1<<20) );  ( RCC->APB2RSTR &= ~(1<<20) ); } while(0)

// Macros to reset I2Cx peripherals
// TODO: FINISH THIS, ITS JUST SPIx REWRITTEN, BITS ARE NOT CORRECT!
#define I2C1_REG_RESET()		do{ ( RCC->APB1RSTR |= (1<<21) );  ( RCC->APB1RSTR &= ~(1<<21) ); } while(0)
#define I2C2_REG_RESET()		do{ ( RCC->APB1RSTR |= (1<<22) );  ( RCC->APB1RSTR &= ~(1<<22) ); } while(0)
#define I2C3_REG_RESET()		do{ ( RCC->APB1RSTR |= (1<<23) );  ( RCC->APB1RSTR &= ~(1<<23) ); } while(0)

#define GPIO_BASE_ADDR_TO_CODE(x) 	( 	(x==GPIOA) ? 0 :\
										(x==GPIOB) ? 1 :\
										(x==GPIOB) ? 2 :\
										(x==GPIOB) ? 3 :\
										(x==GPIOB) ? 4 :\
										(x==GPIOB) ? 5 :\
										(x==GPIOB) ? 6 :\
										(x==GPIOB) ? 7 : 0 )


// IRQ, interrupt requests, numbers of stm32f411ceux MCU
// TODO: add here as is needed
// GPIO IRQ's
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

// spi IRQ's
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_SPI4				84
#define IRQ_NO_SPI5				85

// I2C IRQ's
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73

// Generic macros section
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET


// Bit position macros of SPI peripheral
// CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15
//CR2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7
//SR
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8


// Bit position macros of I2C peripheral
// I2C_CR1
#define I2C_CR1_PE					0
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK 				10
#define I2C_CR1_SWRST				15
// TODO: add others if needed
// I2C_CR2
#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10

// I2C_OAR1
#define I2C_OAR1_ADD0				0
#define I2C_OAR1_ADD71				1
#define I2C_OAR1_ADD98				8
#define I2C_OAR1_ADDMODE			15

// I2C_OAR2
#define I2C_OAR2_ENDUAL				0
#define I2C_OAR2_ADD2				1

// I2C_SR1
#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR				12
#define I2C_SR1_TIMEOUT				14
#define I2C_SR1_SMBALERT			15

// I2C_SR2
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_SMBDEFAULT			5
#define I2C_SR2_SMBHOST				6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC					8

// CCR
#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15


// include drivers
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_i2c_driver.h"
#endif /* INC_STM32F411XX_H_ */
