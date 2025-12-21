#include "lcd_driver.h"
#include "stm32f411xx_gpio_driver.h"

// static helpers declare
static void LCD_Delay_us(uint32_t us);
static void LCD_Delay_ms(uint32_t ms);

static void LCD_Enable_Pulse(void);
static void LCD_Send_Nibble(uint8_t nibble);
static void LCD_Send_Command(uint8_t cmd);
static void LCD_Send_Data(uint8_t data);
static void LCD_GPIO_Init(void);

static void LCD_Send_Byte(uint8_t byte, uint8_t rs_value);

// static helpers definition
static void LCD_Delay_us(uint32_t us) {
	for (volatile uint32_t i = 0; i < us * 4; i++)
		;	//
}
static void LCD_Delay_ms(uint32_t ms) {
	for (uint32_t i = 0; i < ms; i++)
		LCD_Delay_us(1000);

}
// TODO: document all the functions
static void LCD_Enable_Pulse(void) {
	// very important
	GPIO_WriteToOutputPin(LCD_PORT, LCD_EN_PIN, GPIO_PIN_RESET);  // low
	LCD_Delay_us(1);
	GPIO_WriteToOutputPin(LCD_PORT, LCD_EN_PIN, GPIO_PIN_SET);    // rising edge
	LCD_Delay_us(1);
	GPIO_WriteToOutputPin(LCD_PORT, LCD_EN_PIN, GPIO_PIN_RESET); // falling edge
	LCD_Delay_us(100);
}

static void LCD_Send_Byte(uint8_t byte, uint8_t rs_value) {
	GPIO_WriteToOutputPin(LCD_PORT, LCD_RS_PIN, rs_value);
	// send upper nibble
	LCD_Send_Nibble(byte >> 4);
	// send lower nibble
	LCD_Send_Nibble(byte & 0x0F);
}

// im using 4bit mode so that i have enough pins left XD
// that is why im sending data in nibble (half byte)
static void LCD_Send_Nibble(uint8_t nibble) {
	GPIO_WriteToOutputPin(LCD_PORT, LCD_D4_PIN, (nibble >> 0) & 1);
	GPIO_WriteToOutputPin(LCD_PORT, LCD_D5_PIN, (nibble >> 1) & 1);
	GPIO_WriteToOutputPin(LCD_PORT, LCD_D6_PIN, (nibble >> 2) & 1);
	GPIO_WriteToOutputPin(LCD_PORT, LCD_D7_PIN, (nibble >> 3) & 1);
	LCD_Enable_Pulse();
}

static void LCD_Send_Command(uint8_t cmd) {
	LCD_Send_Byte(cmd, GPIO_PIN_RESET);    //RS=0 for command
}

static void LCD_Send_Data(uint8_t data) {
	LCD_Send_Byte(data, GPIO_PIN_SET);    // RS=1 for data
}

static void LCD_GPIO_Init(void) {
	GPIO_Handle_t lcd;

	lcd.pGPIOx = LCD_PORT;
	lcd.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcd.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// init pins
	lcd.GPIO_PinConfig.GPIO_PinNumber = LCD_RS_PIN;
	GPIO_Init(&lcd);
	lcd.GPIO_PinConfig.GPIO_PinNumber = LCD_EN_PIN;
	GPIO_Init(&lcd);
	lcd.GPIO_PinConfig.GPIO_PinNumber = LCD_D4_PIN;
	GPIO_Init(&lcd);
	lcd.GPIO_PinConfig.GPIO_PinNumber = LCD_D5_PIN;
	GPIO_Init(&lcd);
	lcd.GPIO_PinConfig.GPIO_PinNumber = LCD_D6_PIN;
	GPIO_Init(&lcd);
	lcd.GPIO_PinConfig.GPIO_PinNumber = LCD_D7_PIN;
	GPIO_Init(&lcd);
}

// functions
void LCD_Init(void) {
	LCD_GPIO_Init();
	LCD_Delay_ms(50);    // wait to power up

	// set RS and EN low initially
	GPIO_WriteToOutputPin(LCD_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_PORT, LCD_EN_PIN, GPIO_PIN_RESET);

	// init 4bit mode
	// the delays are very fucking important
	LCD_Send_Nibble(LCD_INIT_8BIT_MODE);
	LCD_Delay_us(4500);

	LCD_Send_Nibble(LCD_INIT_8BIT_MODE);
	LCD_Delay_us(4500);

	LCD_Send_Nibble(LCD_INIT_8BIT_MODE);
	LCD_Delay_us(150);

	LCD_Send_Nibble(LCD_INIT_4BIT_MODE);

	LCD_Send_Command(
	LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS);

	// Turn display on
	LCD_Send_Command(
	LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);

	// Clear display
	LCD_Send_Command(LCD_CMD_CLEAR_DISPLAY);
	LCD_Delay_us(2000);

	// Entry mode
	LCD_Send_Command(
	LCD_CMD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
}

void LCD_Clear(void) {
	LCD_Send_Command(LCD_CMD_CLEAR_DISPLAY);
	LCD_Delay_us(2000);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
	uint8_t address = (row == 0) ? LCD_ROW_0_ADDR : LCD_ROW_1_ADDR;
	address += col;
	LCD_Send_Command(LCD_CMD_SET_DDRAM_ADDR | address);
}

void LCD_Print(char *str) {
	while (*str)
		LCD_Send_Data(*str++);
}

void LCD_PrintNumber(int32_t num) {
	char buffer[12];
	int i = 0, is_negative = 0;

	if (num == 0) {
		LCD_Send_Data('0');
		return;
	}
	if (num < 0) {
		is_negative = 1;
		num = -num;
	}

	while (num > 0) {
		buffer[i++] = (num % 10) + '0';
		num /= 10;
	}

	if (is_negative)
		LCD_Send_Data('-');

	while (i > 0)
		LCD_Send_Data(buffer[--i]);
}
