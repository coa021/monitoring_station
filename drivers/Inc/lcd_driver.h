#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include "stm32f411xx.h"
#include <stdint.h>

// pins
#define LCD_RS_PIN  GPIO_PIN_NO_0
#define LCD_EN_PIN  GPIO_PIN_NO_1
#define LCD_D4_PIN  GPIO_PIN_NO_5
#define LCD_D5_PIN  GPIO_PIN_NO_6
#define LCD_D6_PIN  GPIO_PIN_NO_7
#define LCD_D7_PIN  GPIO_PIN_NO_8
#define LCD_PORT    GPIOA

// lcd Commands
#define LCD_CMD_CLEAR_DISPLAY       0x01
#define LCD_CMD_RETURN_HOME         0x02
#define LCD_CMD_ENTRY_MODE_SET      0x04
#define LCD_CMD_DISPLAY_CONTROL     0x08
#define LCD_CMD_FUNCTION_SET        0x20
#define LCD_CMD_SET_DDRAM_ADDR      0x80

// entry Mode flags
#define LCD_ENTRY_LEFT              0x02
#define LCD_ENTRY_SHIFT_DECREMENT   0x00

// display Control flags
#define LCD_DISPLAY_ON              0x04
#define LCD_CURSOR_OFF              0x00
#define LCD_BLINK_OFF               0x00

// function Set flags
#define LCD_4BIT_MODE               0x00
#define LCD_2LINE                   0x08
#define LCD_5x8_DOTS                0x00

#define LCD_INIT_8BIT_MODE			0x03
#define LCD_INIT_4BIT_MODE			0x02

// row addresses
#define LCD_ROW_0_ADDR              0x00
#define LCD_ROW_1_ADDR              0x40

// lcd functions
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);
void LCD_PrintNumber(int32_t num);

#endif //LCD_DRIVER_H
