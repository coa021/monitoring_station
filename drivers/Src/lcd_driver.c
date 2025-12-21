#include "lcd_driver.h"
#include "stm32f411xx_gpio_driver.h"


// pins
#define LCD_RS_PIN        GPIO_PIN_NO_0
#define LCD_EN_PIN        GPIO_PIN_NO_1
#define LCD_D4_PIN        GPIO_PIN_NO_4
#define LCD_D5_PIN        GPIO_PIN_NO_5
#define LCD_D6_PIN        GPIO_PIN_NO_6
#define LCD_D7_PIN        GPIO_PIN_NO_7
#define LCD_PORT          GPIOB

// static helpers declare
static void LCD_Delay_us(uint32_t us);
static void LCD_Delay_ms(uint32_t ms);

static void LCD_Enable_Pulse(void);
static void LCD_Send_Nibble(uint8_t nibble);
static void LCD_Send_Command(uint8_t cmd);
static void LCD_Send_Data(uint8_t data);
static void LCD_GPIO_Init(void);


// static helpers definition
static void LCD_Delay_us(uint32_t us){

}
static void LCD_Delay_ms(uint32_t ms){

}

static void LCD_Enable_Pulse(void){

}
static void LCD_Send_Nibble(uint8_t nibble){

}
static void LCD_Send_Command(uint8_t cmd){

}
static void LCD_Send_Data(uint8_t data){

}
static void LCD_GPIO_Init(void){

}

// functions
void LCD_Init(void){

}
void LCD_Clear(void){

}
void LCD_SetCursor(uint8_t row, uint8_t col){

}
void LCD_Print(char *str){

}
void LCD_PrintNumber(int32_t num){

}
