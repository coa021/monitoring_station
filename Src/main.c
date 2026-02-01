#include <stdio.h>

#include "adc_driver.h"
#include "dht11_driver.h"
#include "lcd_driver.h"
#include "stm32f411xx.h"

volatile uint16_t readings[10];

int main(void) {
	RCC->AHB1ENR |= (1 << 0);
	GPIOA->MODER |= (3 << 0);

	ADC_SimpleInit();

	for (int i = 0; i < 10; i++) {
		readings[i] = ADC_Read(0);
		for (volatile int d = 0; d < 10000; d++)
			;  // Small delay
	}

	while (1)
		;
}
