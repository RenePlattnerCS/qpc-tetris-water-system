/*
 * tetris_input_handler.h
 *
 *  Created on: Nov 6, 2025
 *      Author: NeerW
 */

#ifndef INC_TETRIS_INPUT_HANDLER_H_
#define INC_TETRIS_INPUT_HANDLER_H_

#include "tetris_board.h"

bool process_tilt_move(Board *board, Tetromino *t, int16_t xtilt, int16_t ytilt);

#endif /* INC_TETRIS_INPUT_HANDLER_H_ */
