#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

#define LONG_PRESS_TIME_MS  	5000  // 1 second for long press
#define DEBOUNCE_TIME_MS    	50    // 50ms debounce
#define RF_BUTTON_PIN			GPIO_PIN_0
#define RF_BUTTON_PORT			GPIOA
#define DHT11_RESET_PIN			GPIO_PIN_1
#define DHT11_RESET_PORT		GPIOA

#define MAX_DRY 			 	(2500U) //for the dryness plant sensor
#define MAX_WET 			 	(1000U)
#define PLANT_DRY_THREASHOLD 	(20U) //percent
#define DRY_TIMEOUT			 	(8000U)
#define PUMP_TIMEOUT			(9000U)
#define DHT11_MAX_EDGES 		3
#define DHT11_RESET_TIME 		(30000U)
#define TIMESTAMP_SIZE  		(82U)

#define FPS						20U //20fps tetris
#define MS_PER_SEC		 		1000U


#endif /* INC_APP_CONFIG_H_ */
