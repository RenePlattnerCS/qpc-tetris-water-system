
#include "temp_sensor.h"
//#include "stm32c0xx_hal.h"        // includes core HAL headers
//#include "stm32c0xx_hal_gpio.h"   // specific GPIO macros/structs

#include "bsp.h"
#include "qp_port.h"

//extern TIM_HandleTypeDef  htim14;
//extern TIM_HandleTypeDef  htim17;
extern TIM_HandleTypeDef  htim3;
#include "stm32c0xx_ll_tim.h"

void DHT11_pull_low()
{
	// Start signal
	DHT11_SetPinOutput();
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
	Delay_ms(20); // Pull low for ≥18ms
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
	Delay_us(30);
	DHT11_SetPinInput();
}

uint8_t DHT11_Read(uint8_t *data)
{
    uint8_t bits[5] = {0};
    uint8_t i, j;

    // Start signal
    DHT11_SetPinOutput();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    //BSP_delayMs(20); // Pull low for ≥18ms
    Delay_ms(20);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    Delay_us(30);
    DHT11_SetPinInput();


    // Wait for DHT11 response
    uint32_t timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
        if (++timeout > 100) { return 1;}
        Delay_us(1);
    }
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) {
        if (++timeout > 100) { return 1;}
        Delay_us(1);
    }
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
        if (++timeout > 100) {  return 1;}
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

    *data = bits[2];


    return 0; // success
}


void DHT11_SetPinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

// Set pin as input
void DHT11_SetPinInput(void)
{

	// Configure GPIO for TIM16 input capture
		    GPIO_InitTypeDef GPIO_InitStruct = {0};
		    __HAL_RCC_GPIOD_CLK_ENABLE();

		    GPIO_InitStruct.Pin = DHT11_PIN;
		    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		    GPIO_InitStruct.Pull = GPIO_NOPULL;
		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
		    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

		    // Enable TIM16 interrupt in NVIC
		    NVIC_SetPriority(TIM3_IRQn, QF_AWARE_ISR_CMSIS_PRI);
		    NVIC_EnableIRQ(TIM3_IRQn);
		    Delay_us(10);
		    // Start input capture
		    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
		    //__HAL_TIM_DISABLE_IT(&htim16, TIM_IT_UPDATE);
			//__HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
}


void Delay_us(uint16_t us)
{
	LL_TIM_SetCounter(TIM14, 0);
	while (LL_TIM_GetCounter(TIM14) < us);
	printf("done\n");
}

void Delay_ms(uint16_t ms)
{
	 LL_TIM_SetCounter(TIM17, 0);
	 while (LL_TIM_GetCounter(TIM17) < ms);
}
