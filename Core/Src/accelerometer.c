/*
 * accelerometer.c
 *
 *  Created on: Nov 4, 2025
 *      Author: NeerW
 */

static void ADXL345_ReadXYZ(int16_t *x, int16_t *y, int16_t *z);

void init_accelerometer(void)
{
	HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, ADXL_REG, 1, &data, 1, HAL_MAX_DELAY);
	  if (data != 0xE5) {
			  // Handle error: wrong device or not connected
			  printf("error\n");
		  }

	  data = 0x08;
	        HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_POWER_CTL_R, 1, &data, 1, HAL_MAX_DELAY);

	        // Optional: set data format to full resolution, ±2g
	        data = 0x08;
	        HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_DATA_FORMAT_R, 1, &data, 1, HAL_MAX_DELAY);


}

void read_accelerometer(void)
{
	ADXL345_ReadXYZ(&x,&y,&z);
}

static void ADXL345_ReadXYZ(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[6];

    // Read 6 bytes starting at DATAX0
    HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, ADXL_DATA_START_ADDR, 1, buffer, 6, HAL_MAX_DELAY);

    // Combine into 16-bit signed integers (little endian)
    *x = (int16_t)((buffer[1] << 8) | buffer[0]);
    *y = (int16_t)((buffer[3] << 8) | buffer[2]);
    *z = (int16_t)((buffer[5] << 8) | buffer[4]);
}
