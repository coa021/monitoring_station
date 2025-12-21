#include "stm32f411xx.h"
#include "lcd_driver.h"

void delay_ms(uint32_t ms) {
	for (uint32_t i = 0; i < ms * 1000; i++)
		;
}

int main(void) {

	LCD_Init();

	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("LCD Test v1.0");
	LCD_SetCursor(1, 0);
	LCD_Print("Driver Works!");
	delay_ms(2000);

	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("Numbers:");
	LCD_SetCursor(1, 0);
	LCD_PrintNumber(12345);
	delay_ms(2000);

	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("Negative:");
	LCD_SetCursor(1, 0);
	LCD_PrintNumber(-999);
	delay_ms(2000);

	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("Counting:");

	for (int i = 0; i <= 20; i++) {
		LCD_SetCursor(1, 0);
		LCD_Print("Count: ");
		LCD_PrintNumber(i);
		LCD_Print("  ");
		delay_ms(500);
	}

	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("Scrolling...");
	delay_ms(1000);

	char *message = "Hello World! This is a long message that scrolls.";
	for (int pos = 0; pos < 35; pos++) {
		LCD_SetCursor(1, 0);
		// Print 16 characters starting from pos
		for (int i = 0; i < 16; i++) {
			if (message[pos + i] != '\0') {
				char temp[2] = { message[pos + i], '\0' };
				LCD_Print(temp);
			} else {
				LCD_Print(" ");
			}
		}
		delay_ms(300);
	}

	// Test 6: All done
	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("All Tests");
	LCD_SetCursor(1, 0);
	LCD_Print("Passed!");

	while (1) {
	}

	return 0;
}
