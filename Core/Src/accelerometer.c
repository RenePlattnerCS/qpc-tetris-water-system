
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

uint16_t read_accelerometer_tilt()
{
	int16_t rawX, rawY, rawZ;
	ADXL345_ReadXYZ(&rawX, &rawY, &rawZ);


	//g force conversion
	//float x_g = rawX * 0.0039f;
	//float y_g = rawY * 0.0039f;
	//float z_g = rawZ * 0.0039f;
	int32_t x_mg = (int32_t)rawX * 3900 / 1000;  // or simply * 39 if you prefer smaller numbers
	int32_t y_mg = (int32_t)rawY * 3900 / 1000;
	int32_t z_mg = (int32_t)rawZ * 3900 / 1000;

	//float tiltX = atan2f(-x_g, z_g) * 180.0f / 3.14159f; // tilt left/right
	//float tiltY = atan2f(-y_g, z_g) * 180.0f / 3.14159f; // tilt forward/back

	compute_tiltX(x_mg, z_mg);

}

int compute_tiltX(int32_t rawX, int32_t rawZ)
{
    if(rawZ == 0) rawZ = 1;  // avoid division by zero

    int tiltX = (int32_t)rawX * 100 / rawZ;

    // clamp to -100 … +100
    if(tiltX > 100) tiltX = 100;
    if(tiltX < -100) tiltX = -100;

    return tiltX;
}
