/*
 * display.h
 *
 *  Created on: Nov 1, 2025
 *      Author: NeerW
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdint.h>

void display_temp(uint16_t temp);

#endif /* INC_DISPLAY_H_ */
