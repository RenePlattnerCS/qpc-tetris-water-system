

#ifndef INC_TETRIS_BOARD_H_
#define INC_TETRIS_BOARD_H_

#include "tetris_board.h"
#include "tetromino.h"
#include <stdbool.h>

typedef struct {
	uint8_t pos_x;
	uint8_t pos_y;
    uint8_t width;
    uint8_t height;
    uint8_t blockSize;
    bool	rotate_90;
    uint8_t *grid;       // grid array ptr

    uint8_t tickCounter;
    uint8_t speed;

} Board;


void Board_ctor(Board *me, uint8_t *grid, uint8_t width, uint8_t height, uint8_t pos_x , uint8_t pos_y, bool rotate_90, uint8_t blockSize);
void Board_placeTetromino(Board *me, const Tetromino *t);
void draw_board(Board *me, Tetromino *active, uint8_t score);
void clear_board();
//void Board_init(Board *me);
bool collision_on_move(Board *me, const Tetromino *t, int dx, int dy);
bool collision_on_spawn(Board *me, Tetromino *t);
bool move_down(Board *me, Tetromino *active);
uint8_t check_and_clear_lines(Board * me);


#endif /* INC_TETRIS_BOARD_H_ */
