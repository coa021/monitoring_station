#include "adc_driver.h"


void ADC_Init(void){
    // TODO: Clear this function up once i finish the course!
//    SPI1_PCLK_EN();    // i mean, technically i can reuse this XD
	RCC->APB2ENR |= (1 << 8);

    GPIO_Handle_t adc;
    adc.pGPIOx = GPIOA;
    adc.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    adc.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
    adc.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&adc);

    ADC->CCR &= ~(3 << 16);  // Clear prescaler bits
    ADC->CCR |= (0 << 16);   // ADCPRE = 00 (divide by 2)

    // Configure ADC1
    ADC1->CR1 = 0;  // Clear CR1 register
    ADC1->CR2 = 0;  // Clear CR2 register

    // Set resolution to 12-bit (00)
    ADC1->CR1 &= ~(3 << 24);

    // Set scan mode off (single channel)
    ADC1->CR1 &= ~(1 << 8);

    // Set continuous conversion mode off
    ADC1->CR2 &= ~(1 << 1);

    // Right data alignment
    ADC1->CR2 &= ~(1 << 11);

    // Set sampling time for channel 3
    ADC1->SMPR2 &= ~(7 << 9);  // Clear channel 3 sampling time bits
    ADC1->SMPR2 |= (7 << 9);   // 480 cycles sampling time

    // Power on ADC
    ADC1->CR2 |= (1 << 0);  // ADON bit

    // Wait for ADC to stabilize (a few microseconds)
    for (volatile int i = 0; i < 1000; i++);
}

uint16_t ADC_Read(uint8_t channel){
    // Set the channel in the regular sequence
    ADC1->SQR3 = channel;  // First conversion in regular sequence

    // Start conversion
    ADC1->CR2 |= (1 << 30);  // SWSTART bit

    // Wait for conversion to complete
    while (!(ADC1->SR & (1 << 1)));  // Wait for EOC (End of Conversion) flag

    // Read and return the conversion result
    return ADC1->DR;
}
