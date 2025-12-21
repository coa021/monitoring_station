#include "stm32f411xx.h"

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1000; i++);
}

void LED_Init(void) {
    GPIO_Handle_t led;

    // LED on PC13 (onboard)
    led.pGPIOx = GPIOC;
    led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&led);

    // external LED on PA8
    led.pGPIOx = GPIOA;
    led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    GPIO_Init(&led);
}

int main(void)
{
    LED_Init();

    while (1) {
        GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);

        delay_ms(500);
    }

    return 0;
}
