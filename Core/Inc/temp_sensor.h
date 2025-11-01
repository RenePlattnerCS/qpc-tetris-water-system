/*
 * temp_sensor.h
 *
 *  Created on: Nov 1, 2025
 *      Author: NeerW
 */

#ifndef INC_TEMP_SENSOR_H_
#define INC_TEMP_SENSOR_H_

#define DHT11_PORT GPIOA
#define DHT11_PIN  GPIO_PIN_1

#include <stdint.h>

typedef struct {
    uint8_t Temperature;
    uint8_t Humidity;
} DHT11_Data_t;

uint8_t DHT11_Read(DHT11_Data_t *data);





#endif /* INC_TEMP_SENSOR_H_ */
