#include "plant_sensor.h"
#include "stm32c0xx.h"

//extern ADC_HandleTypeDef hadc1;

uint32_t read_dryness(void)
{
	/*
	 uint32_t adc_value;
	 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	 HAL_ADC_Start(&hadc1);
	 uint32_t value = HAL_ADC_GetValue(&hadc1);
	 HAL_ADC_Stop(&hadc1);
	 return adc_value;
	 */
	LL_ADC_REG_StartConversionSWStart(ADC1);

	// Wait for conversion to complete
	while(!LL_ADC_IsActiveFlag_EOC(ADC1));

	// Read ADC value
	uint32_t adc_value = LL_ADC_REG_ReadConversionData12(ADC1);

	// Clear end-of-conversion flag (optional, sometimes auto-cleared)
	LL_ADC_ClearFlag_EOC(ADC1);

	return adc_value;
}
