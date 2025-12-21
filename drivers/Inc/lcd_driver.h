#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include "stm32f411xx.h"
#include <stdint.h>

// lcd functions
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);
void LCD_PrintNumber(int32_t num);


#endif //LCD_DRIVER_H
