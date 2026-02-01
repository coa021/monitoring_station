#include "stm32f411xx.h"
#include "lcd_driver.h"
#include "dht11_driver.h"
#include "adc_driver.h"

void Delay_ms(uint32_t ms) {
	for (uint32_t i = 0; i < ms * 1000; i++)
		;
}

typedef enum {
	MODE_TEMP_HUMIDITY = 0, MODE_LIGHT, MODE_ALL_DATA, MODE_COUNT
} DisplayMode_t;

DisplayMode_t current_mode = MODE_TEMP_HUMIDITY;
DHT11_Data_t sensor_data;
uint16_t light_level;

void Button_Init(void) {
	GPIO_Handle_t button;

	button.pGPIOx = GPIOB;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7; // Pb7
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&button);
}

uint8_t Button_IsPressed(void) {
	static uint8_t button_was_pressed = 0;

	if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7) == GPIO_PIN_RESET) {
		if (button_was_pressed == 0) {
			Delay_ms(50);

			if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7) == GPIO_PIN_RESET) {
				button_was_pressed = 1;
				return 1;  // Button confirmed pressed
			}
		}
	} else {
		button_was_pressed = 0;
	}

	return 0;
}

void Display_Mode_TempHumidity(void) {
	LCD_Clear();

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
}

void Display_Mode_Light(void) {
	LCD_Clear();

	LCD_SetCursor(0, 0);
	LCD_Print("Light Level:");

	LCD_SetCursor(1, 0);
	LCD_PrintNumber(light_level);
	LCD_Print(" / 4095");

	// Show brightness percentage
	uint8_t percent = (light_level * 100) / 4095;
	LCD_SetCursor(1, 12);
	LCD_PrintNumber(percent);
	LCD_Print("%");
}

void Display_Mode_AllData(void) {
	LCD_Clear();

	// Row 0: Temperature and Humidity
	LCD_SetCursor(0, 0);
	LCD_Print("T:");
	LCD_PrintNumber(sensor_data.temperature_int);
	LCD_Print("C H:");
	LCD_PrintNumber(sensor_data.humidity_int);
	LCD_Print("%");

	// Row 1: Light level
	LCD_SetCursor(1, 0);
	LCD_Print("L:");
	LCD_PrintNumber(light_level);
}

void Update_Display(void) {
	switch (current_mode) {
	case MODE_TEMP_HUMIDITY:
		Display_Mode_TempHumidity();
		break;

	case MODE_LIGHT:
		Display_Mode_Light();
		break;

	case MODE_ALL_DATA:
		Display_Mode_AllData();
		break;

	default:
		current_mode = MODE_TEMP_HUMIDITY;
		break;
	}
}

// Display initial layout (called once when mode changes)
void Display_InitialLayout(void) {
	LCD_Clear();

	switch (current_mode) {
	case MODE_TEMP_HUMIDITY:
		LCD_SetCursor(0, 0);
		LCD_Print("Temp:      C");
		LCD_SetCursor(1, 0);
		LCD_Print("Humid:     %");
		break;

	case MODE_LIGHT:
		LCD_SetCursor(0, 0);
		LCD_Print("Light Level:");
		LCD_SetCursor(1, 0);
		LCD_Print("     / 4095   %");
		break;

	case MODE_ALL_DATA:
		LCD_SetCursor(0, 0);
		LCD_Print("T:   C H:   %");
		LCD_SetCursor(1, 0);
		LCD_Print("L:");
		break;
	}
}

// Update only the changing values (NO LCD_Clear!)
void Update_Values(uint8_t dht_changed, uint8_t light_changed) {
	switch (current_mode) {
	case MODE_TEMP_HUMIDITY:
		if (dht_changed) {
			// Update temperature at position (0, 6)
			LCD_SetCursor(0, 6);
			LCD_PrintNumber(sensor_data.temperature_int);
			LCD_Print(".");
			LCD_PrintNumber(sensor_data.temperature_dec);

			// Update humidity at position (1, 7)
			LCD_SetCursor(1, 7);
			LCD_PrintNumber(sensor_data.humidity_int);
			LCD_Print(".");
			LCD_PrintNumber(sensor_data.humidity_dec);
		}
		break;

	case MODE_LIGHT:
		if (light_changed) {
			// Update light value at position (1, 0)
			LCD_SetCursor(1, 0);

			// Print with padding to clear old digits
			if (light_level < 10) {
				LCD_Print("    ");
				LCD_PrintNumber(light_level);
			} else if (light_level < 100) {
				LCD_Print("   ");
				LCD_PrintNumber(light_level);
			} else if (light_level < 1000) {
				LCD_Print("  ");
				LCD_PrintNumber(light_level);
			} else {
				LCD_Print(" ");
				LCD_PrintNumber(light_level);
			}

			// Update percentage at position (1, 13)
			uint8_t percent = (light_level * 100) / 4095;
			LCD_SetCursor(1, 13);

			if (percent < 10) {
				LCD_Print(" ");
				LCD_PrintNumber(percent);
			} else {
				LCD_PrintNumber(percent);
			}
		}
		break;

	case MODE_ALL_DATA:
		if (dht_changed) {
			// Update temperature at (0, 2)
			LCD_SetCursor(0, 2);
			if (sensor_data.temperature_int < 10) {
				LCD_Print(" ");
			}
			LCD_PrintNumber(sensor_data.temperature_int);

			// Update humidity at (0, 9)
			LCD_SetCursor(0, 9);
			if (sensor_data.humidity_int < 10) {
				LCD_Print(" ");
			}
			LCD_PrintNumber(sensor_data.humidity_int);
		}

		if (light_changed) {
			// Update light at (1, 2)
			LCD_SetCursor(1, 2);

			if (light_level < 10) {
				LCD_Print("   ");
				LCD_PrintNumber(light_level);
			} else if (light_level < 100) {
				LCD_Print("  ");
				LCD_PrintNumber(light_level);
			} else if (light_level < 1000) {
				LCD_Print(" ");
				LCD_PrintNumber(light_level);
			} else {
				LCD_PrintNumber(light_level);
			}
		}
		break;
	}
}

int main(void) {
	uint8_t read_result;
	uint32_t dht_counter = 0;
	uint32_t light_counter = 0;
	uint32_t loop_count = 0;

	uint8_t dht_updated = 0;
	uint8_t light_updated = 0;
	uint8_t mode_changed = 0;

	LCD_Init();
	DHT11_Init();
	ADC_Init();
	Button_Init();

	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("Environmental");
	LCD_SetCursor(1, 0);
	LCD_Print("Monitor v1.0");
	Delay_ms(2000);

	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_Print("Press button");
	LCD_SetCursor(1, 0);
	LCD_Print("to change mode");
	Delay_ms(2000);

	// Initial sensor read
	DHT11_Read(&sensor_data);
	light_level = ADC_Read(3);

	// Display initial layout
	Display_InitialLayout();
	Update_Values(1, 1);  // Update with initial values

	while (1) {
		if (Button_IsPressed()) {
			current_mode++;
			if (current_mode >= MODE_COUNT) {
				current_mode = MODE_TEMP_HUMIDITY;
			}

			mode_changed = 1;
			Update_Display();
			Delay_ms(200);
		}

		// If mode changed, redraw entire layout
		if (mode_changed) {
			Display_InitialLayout();
			Update_Values(1, 1);  // Update all values
			mode_changed = 0;
			dht_updated = 0;
			light_updated = 0;
		}

		// Read DHT11 every 2 seconds (20 * 100ms)
		if ((loop_count - dht_counter) >= 20) {
			dht_counter = loop_count;

			read_result = DHT11_Read(&sensor_data);

			if (read_result == DHT11_OK) {
				dht_updated = 1;
			} else {
				// Show error temporarily
				LCD_Clear();
				LCD_SetCursor(0, 0);
				LCD_Print("DHT11 Error!");
				LCD_SetCursor(1, 0);
				LCD_Print("Check wiring");
				Delay_ms(1000);

				// Redraw layout after error
				Display_InitialLayout();
				Update_Values(0, 1);  // Update light only
			}
		}

		// Read light sensor every 200ms (2 * 100ms)
		if ((loop_count - light_counter) >= 2) {
			light_counter = loop_count;

			light_level = ADC_Read(3);
			light_updated = 1;
		}

		// Update only changed values (NO clear!)
		if (dht_updated || light_updated) {
			Update_Values(dht_updated, light_updated);

			dht_updated = 0;
			light_updated = 0;
		}

		loop_count++;
		Delay_ms(100);
	}

	return 0;
}
