/*
 * accelerometer.h
 *
 *  Created on: Nov 4, 2025
 *      Author: NeerW
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

#define ADXL_ADDR				 (0x53 << 1)
#define ADXL_REG				 0x00
#define	ADXL_DATA_FORMAT_R		 0x31
#define ADXL_POWER_CTL_R		 0x2D
#define ADXL_MEASURE_MODE     	 0x08
#define ADXL_DATA_START_ADDR 	 0x32

void init_accelerometer(void);

void read_accelerometer(int16_t *x, int16_t *y, int16_t *z);

#endif /* INC_ACCELEROMETER_H_ */
