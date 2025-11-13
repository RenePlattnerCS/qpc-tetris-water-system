#include "plant_sensor.h"
#include "stm32c0xx.h"
#include "stm32c0xx_ll_adc.h"

uint32_t read_dryness(void)
{

	LL_ADC_REG_StartConversion(ADC1);
	// Wait for conversion to complete
	while(!LL_ADC_IsActiveFlag_EOC(ADC1));

	// Read ADC value
	uint32_t adc_value = LL_ADC_REG_ReadConversionData12(ADC1);

	// Clear end-of-conversion flag (optional, sometimes auto-cleared)
	LL_ADC_ClearFlag_EOC(ADC1);

	return adc_value;
}
