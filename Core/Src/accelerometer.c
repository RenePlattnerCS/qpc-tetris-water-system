
#include <stdint.h>
#include "accelerometer.h"
#include "stm32c0xx_hal.h"
#include "temp_sensor.h"

extern I2C_HandleTypeDef hi2c1;


void init_accelerometer(void) //tap detection
{
	// NOW clear the interrupt
	    uint8_t int_source;
	    HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, 0x30, 1, &int_source, 1, HAL_MAX_DELAY);

    uint8_t data;

    // Full resolution ±2g
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, 0x31, 1, &data, 1, HAL_MAX_DELAY);

    // Tap threshold (adjust)
    data = 0x20; // 0.5g
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, 0x1D, 1, &data, 1, HAL_MAX_DELAY);

    // Tap duration
    data = 0x10; // ~10ms
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, 0x21, 1, &data, 1, HAL_MAX_DELAY);

    // Enable tap on XYZ
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, 0x2A, 1, &data, 1, HAL_MAX_DELAY);

    // Route single-tap interrupt to INT1
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, 0x2F, 1, &data, 1, HAL_MAX_DELAY);

    // Enable single tap interrupt
    data = 0x40;
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, 0x2E, 1, &data, 1, HAL_MAX_DELAY);

    // Measurement mode
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, 0x2D, 1, &data, 1, HAL_MAX_DELAY);

    // NOW clear the interrupt

	HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, 0x30, 1, &int_source, 1, HAL_MAX_DELAY);
}

void init_accelerometer2(void)
{

    uint8_t data = 0;

    // .---------------------------------------------
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_INT_ENABLE, 1, &data, 1, HAL_MAX_DELAY);
    //--------------  ------------------------

    // 1. Set data format (full resolution, ±2g)
        data = 0x08;  // Full resolution, right justified, ±2g
        HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_DATA_FORMAT_R, 1, &data, 1, HAL_MAX_DELAY);

        // 1. Set data format (full resolution, ±2g)
		data = 0x08;  // Full resolution, right justified, ±2g
		HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_DATA_FORMAT_R, 1, &data, 1, HAL_MAX_DELAY);

		// 2. Configure tap detection
		data = 0x20;  // Threshold ~0.5g (adjust as needed)
		HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_THRESH_ACT, 1, &data, 1, HAL_MAX_DELAY);

		data = 0x10;  // Tap duration = 16 * 625us = 10ms
		HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, TAP_DUR, 1, &data, 1, HAL_MAX_DELAY);

		data = 0x07;  // TAP_AXES = 0x07 -> X, Y, Z enabled
		HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, 0x2A, 1, &data, 1, HAL_MAX_DELAY);

		// 3. Map single tap interrupt to INT1
		data = 0x00;  // 0 = INT1
		HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_INT_MAP, 1, &data, 1, HAL_MAX_DELAY);

		// 4. Enable single tap interrupt
		data = 0x10;  // SINGLE_TAP
		HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, INT_TAP_ENABLE, 1, &data, 1, HAL_MAX_DELAY);


            //---------------------------  -------------------------


    // Start measurement mode
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, 0x2D, 1, &data, 1, HAL_MAX_DELAY);

    // Wait for stabilization
    Delay_ms(50000);

    // NOW clear the interrupt
    uint8_t int_source;
    HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, 0x30, 1, &int_source, 1, HAL_MAX_DELAY);


    // ----------------------------------------------------------
    // Re-enable interrupts
    //data = 0x10; activity interrupt
    data = 0x40; // -> single tap interrupt!
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_INT_ENABLE, 1, &data, 1, HAL_MAX_DELAY);
    //  ----------------------------------------------------------

    // NOW clear the interrupt
        HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, 0x30, 1, &int_source, 1, HAL_MAX_DELAY);





}

/*
void read_accelerometer(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t buffer[6];

	// Read 6 bytes starting at DATAX0
	HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, ADXL_DATA_START_ADDR, 1, buffer, 6, HAL_MAX_DELAY);

	// Combine into 16-bit signed integers (little endian)
	*x = (int16_t)((buffer[1] << 8) | buffer[0]);
	*y = (int16_t)((buffer[3] << 8) | buffer[2]);
	*z = (int16_t)((buffer[5] << 8) | buffer[4]);
}

*/
