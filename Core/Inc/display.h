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
#include <stdbool.h>
#include "tetris_board.h"
void rotate_tetromino_collision_check(Board * me, Tetromino* t);

typedef struct {
    int x0, y0, x1, y1, dx, dy, sx, sy, err;
    bool done;

} LineState;

void display_score(uint16_t score);
void display_temp(uint16_t temp);
void display_dry(uint8_t dryness_percent);
void display_tetris_gameover(void);
void clear_display(void);
void draw_pixel(uint8_t x, uint8_t y);
void draw_pixel_block(uint8_t x, uint8_t y, uint8_t width, uint8_t height);

void draw_line_step(LineState *ls);
void draw_border(Board * board);

#endif /* INC_DISPLAY_H_ */
