/*
 * accelerometer.h
 *
 *  Created on: Nov 4, 2025
 *      Author: NeerW
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_


//setup
#define ADXL_ADDR				 (0x53 << 1)
#define ADXL_REG				 0x00
#define	ADXL_DATA_FORMAT_R		 0x31
#define ADXL_POWER_CTL_R		 0x2D
#define ADXL_MEASURE_MODE     	 0x08

//interrupt, shake detection
#define ADXL_BW_RATE			0x2C
#define ADXL_THRESH_ACT			0x24
#define ADXL_ACT_INACT_CTL		0x27
#define ADXL_DATA_START_ADDR 	0x32
#define ADXL_INT_ENABLE			0x2E
#define ADXL_INT_MAP			0x2F
#define ADXL_INT_SOURCE 		0x30
#define TAP_DUR					0x21
#define INT_TAP_ENABLE			0x40

#define REG_THRESH_SHAKE		 0x1D


void init_accelerometer(void);
void accelerometer_init_polling(void);
void read_accelerometer(int16_t *x, int16_t *y, int16_t *z);
void read_accelerometer_tilt(int * xtilt, int * ytilt);

#endif /* INC_ACCELEROMETER_H_ */
