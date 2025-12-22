#include "stm32f411xx.h"
#include "lcd_driver.h"
#include "dht11_driver.h"

void Delay_ms(uint32_t ms) {
	for (uint32_t i = 0; i < ms * 1000; i++)
		;
}

int main(void) {
	DHT11_Data_t sensor_data;
	uint8_t read_result;

	LCD_Init();
	DHT11_Init();

	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("DHT11 + LCD");
	LCD_SetCursor(1, 0);
	LCD_Print("Starting...");
	Delay_ms(2000);

	while (1) {
		read_result = DHT11_Read(&sensor_data);

		if (read_result == DHT11_OK) {
			LCD_SetCursor(0, 0);
			LCD_Print("Temp: ");
			LCD_PrintNumber(sensor_data.temperature_int);
			LCD_Print(".");
			LCD_PrintNumber(sensor_data.temperature_dec);
			LCD_Print("C");

			LCD_SetCursor(1, 0);
			LCD_Print("Humid: ");
			LCD_PrintNumber(sensor_data.humidity_int);
			LCD_Print(".");
			LCD_PrintNumber(sensor_data.humidity_dec);
			LCD_Print("%");
		} else {
			LCD_Clear();
			LCD_SetCursor(0, 0);
			LCD_Print("Trouble reading!");
			LCD_SetCursor(1, 0);
			LCD_Print("Debug code pls");
		}

		Delay_ms(2000);	// 2s between reads
	}


	return 0;
}
