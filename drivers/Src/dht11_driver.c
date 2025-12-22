#include "dht11_driver.h"

static GPIO_Handle_t DHT11_Pin;
// TODO: better code commentary
// static helpers
static void DHT11_Delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us; i++)
        ;	//
}
static void DHT11_Delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++)
        DHT11_Delay_us(1000);
}

// set output
static void DHT11_SetOutput(void) {
    DHT11_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIO_Init(&DHT11_Pin);
}

// set as input
static void DHT11_SetInput(void) {
    DHT11_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_Init(&DHT11_Pin);
}

static uint32_t DHT11_ExpectPulse(uint8_t level) {
    uint32_t count = 0;

    while(GPIO_ReadFromInputPin(DHT11_PORT, DHT11_PIN) == level) {
        count++;
        if(count > DHT11_TIMEOUT_MS)
            return 0;
    }
    return count;
}

void DHT11_Init(void){
    // init as output initially, later we change
    DHT11_Pin.pGPIOx = DHT11_PORT;
    DHT11_Pin.GPIO_PinConfig.GPIO_PinNumber = DHT11_PIN;
    DHT11_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    DHT11_Pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    DHT11_Pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    DHT11_Pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&DHT11_Pin);

    // set high initially
    GPIO_WriteToOutputPin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
}

uint8_t DHT11_Read(DHT11_Data_t *data){
    // whole approach copied from arduino library lol
    // it wasnt working the way i tried
    uint32_t cycles[80];

    data->humidity_int =0;
    data->humidity_dec =0;
    data->temperature_int =0;
    data->temperature_dec =0;
    data->checksum = 0;

    DHT11_SetInput();
    DHT11_Delay_ms(1);

    DHT11_SetOutput();
    GPIO_WriteToOutputPin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    DHT11_Delay_ms(20);    // at least 18ms

    DHT11_SetInput();
    DHT11_Delay_us(40);    // wait for sensor to respond

    // wait for sensor response; low then high
    if(DHT11_ExpectPulse(GPIO_PIN_RESET) == 0)
        return DHT11_ERROR;    // low

    if(DHT11_ExpectPulse(GPIO_PIN_SET) == 0)
        return DHT11_ERROR;    // high

    uint8_t raw_data[5] = {0};
    // read 40 bits
    for(int i=0; i<80; i+=2) {
        cycles[i] = DHT11_ExpectPulse(GPIO_PIN_RESET);    // low pulse
        cycles[i+1] = DHT11_ExpectPulse(GPIO_PIN_SET);    // high pulse
    }

    // process 40 bits into 5 bytes
    for(int i=0;i<40;i++) {
        uint32_t low = cycles[2*i];
        uint32_t high = cycles[2*i+1];

        if((low == 0) || (high ==0))
            return DHT11_TIMEOUT_ERROR;

        raw_data[i/8] <<=1;

        // copied from arduino library, compare high to low duration
        // if high > low, its 1, otherwise 0
        if(high > low)
            raw_data[i/8] |= 1;
    }

    data->humidity_int = raw_data[0];
    data->humidity_dec = raw_data[1];
    data->temperature_int = raw_data[2];
    data->temperature_dec = raw_data[3];
    data->checksum = raw_data[4];

    uint8_t checksum_calc = raw_data[0] + raw_data[1] + raw_data[2] + raw_data[3];
    if (checksum_calc != raw_data[4]) {
        return DHT11_ERROR;
    }

    return DHT11_OK;
}
