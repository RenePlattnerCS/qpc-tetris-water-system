/*
 * temp_sensor.h
 *
 *  Created on: Nov 1, 2025
 *      Author: NeerW
 */

#ifndef INC_TEMP_SENSOR_H_
#define INC_TEMP_SENSOR_H_

#define DHT11_PORT GPIOC
#define DHT11_PIN  GPIO_PIN_6

#include <stdint.h>
#include "stm32c0xx_hal.h"

void Delay_us(uint16_t us);
uint8_t DHT11_Read(uint8_t *temp_data);
void DHT11_pull_low();
void DHT11_SetPinOutput(void);
void DHT11_SetPinInput(void);


#endif /* INC_TEMP_SENSOR_H_ */
