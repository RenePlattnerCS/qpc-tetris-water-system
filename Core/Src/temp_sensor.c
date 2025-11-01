
#include "temp_sensor.h"
#include "stm32c0xx_hal.h"        // includes core HAL headers
#include "stm32c0xx_hal_gpio.h"   // specific GPIO macros/structs


static void DHT11_SetPinOutput(void);
static void DHT11_SetPinInput(void);


uint8_t DHT11_Read(DHT11_Data_t *data)
{
    uint8_t bits[5] = {0};
    uint8_t i, j;

    // Start signal
    DHT11_SetPinOutput();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(20); // Pull low for ≥18ms
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    Delay_us(30);
    DHT11_SetPinInput();

    // Wait for DHT11 response
    uint32_t timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
        if (++timeout > 100) return 1;
        Delay_us(1);
    }
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) {
        if (++timeout > 100) return 1;
        Delay_us(1);
    }
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
        if (++timeout > 100) return 1;
        Delay_us(1);
    }

    // Read 5 bytes (40 bits)
    for (j = 0; j < 5; j++) {
        for (i = 0; i < 8; i++) {
            // Wait for the start of the bit (low signal)
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET);
            Delay_us(30);
            if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET)
                bits[j] |= (1 << (7 - i));
            // Wait for end of bit
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET);
        }
    }

    // Verify checksum
    if ((uint8_t)(bits[0] + bits[1] + bits[2] + bits[3]) != bits[4])
        return 2; // checksum error

    data->Humidity = bits[0];
    data->Temperature = bits[2];

    return 0; // success
}


static void DHT11_SetPinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

// Set pin as input
static void DHT11_SetPinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}


