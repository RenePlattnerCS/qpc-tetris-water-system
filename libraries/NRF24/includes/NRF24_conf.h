/*
 * 25-JUL-2024
 * STM32 HAL NRF24 LIBRARY
 */

#ifndef _NRF_24_CONF_H_
#define _NRF_24_CONF_H_

#include "stm32c0xx_ll_gpio.h"

#define SPIX SPI1

//LL format:
#define csn_gpio_port GPIOB
#define csn_gpio_pin LL_GPIO_PIN_1

#define ce_gpio_port GPIOB
#define ce_gpio_pin LL_GPIO_PIN_0





#endif

