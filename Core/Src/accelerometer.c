
#include <stdint.h>
#include "accelerometer.h"
#include "stm32c0xx_hal.h"
#include "temp_sensor.h"

extern I2C_HandleTypeDef hi2c1;
static int compute_tiltX(int32_t rawX, int32_t rawZ);


void init_accelerometer(void) //tap detection
{
	//clear the interrupt
	uint8_t int_source;
	HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, 0x30, 1, &int_source, 1, HAL_MAX_DELAY);

	uint8_t data;

	// Full resolution ±2g
	data = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, 0x31, 1, &data, 1, HAL_MAX_DELAY);

	//Tap threshold (adjust)
	data = 0xc8;
	HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, REG_THRESH_SHAKE , 1, &data, 1, HAL_MAX_DELAY);

	 //Tap duration
	data = 0x90;
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

	// clear the interrupt

	HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, 0x30, 1, &int_source, 1, HAL_MAX_DELAY);
}
void accelerometer_init_polling(void)
{
    uint8_t data;

    // 1) Disable all interrupts
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_INT_ENABLE, 1, &data, 1, HAL_MAX_DELAY);

    // 2) Set measurement range: ±2g, right justified, full resolution
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_DATA_FORMAT_R, 1, &data, 1, HAL_MAX_DELAY);

    // 3) Put ADXL345 into measurement mode
    data = 0x08;  // Measure bit
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDR, ADXL_POWER_CTL_R, 1, &data, 1, HAL_MAX_DELAY);
}


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

void read_accelerometer_tilt(int * xtilt, int * ytilt)
{
	int16_t rawX, rawY, rawZ;
	read_accelerometer(&rawX, &rawY, &rawZ);


	//g force conversion
	int x_mg = (rawX * 1000) / 256;
	int y_mg = (rawY * 1000) / 256;
	int z_mg = (rawZ * 1000) / 256;


	*xtilt = compute_tiltX(x_mg, z_mg);
	*ytilt = compute_tiltX(y_mg, z_mg);

}

static int compute_tiltX(int32_t x_mg, int32_t z_mg)
{
    if(z_mg == 0) z_mg = 1; // avoid divide by zero

    int tiltX = (x_mg * 100) / z_mg;

    if(tiltX > 100) tiltX = 100;
    if(tiltX < -100) tiltX = -100;

    return tiltX;
}
