//
// Created by coa on 12/22/2025.
//

#ifndef HELPERS_H
#define HELPERS_H

#include <stdint.h>

void delay_us(uint32_t us) {
    // 4 is used because 16mhz
    for (volatile uint32_t i = 0; i < us * 4; i++)
        ;	//
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++)
        delay_us(1000);
}

#endif //HELPERS_H
