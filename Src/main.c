#include <stdio.h>

#include "stm32f411xx.h"

static uint32_t tick_count = 0;

void TIM3_IRQHandler(void) {
	if (TIM_GetFlagStatus(TIM3, TIM_FLAG_UPDATE)) {
		tick_count++;

		// Toggle LED or do periodic task
		GPIOC->ODR ^= (1 << 13);

		TIM_ClearFlag(TIM3, TIM_FLAG_UPDATE);
	}
}

int main(void) {

	RCC->AHB1ENR |= (1 << 2);
	GPIOC->MODER |= (1 << 26);

	// 1500ms periodic interrupt
	TIM_InitPeriodicInterrupt(TIM3, 1500, 2);

	while (1) {
	}
}
