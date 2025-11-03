/*
 * app_config.h
 *
 *  Created on: Nov 1, 2025
 *      Author: NeerW
 */

#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

#define LONG_PRESS_TIME_MS  	1000  // 1 second for long press
#define DEBOUNCE_TIME_MS    	50    // 50ms debounce
#define RF_BUTTON_PIN			GPIO_PIN_0
#define RF_BUTTON_PORT		GPIOA

#define MAX_DRY 			 (2500U) //for the dryness plant sensor
#define MAX_WET 			 (1000U)

#define DHT11_MAX_EDGES 		3

#endif /* INC_APP_CONFIG_H_ */
