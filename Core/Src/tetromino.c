
#include "tetromino.h""
#include "tetris_shapes.h"
#include "tetris_board.h"
#include "app_config.h"
void Tetromino_ctor(Tetromino *me, TetrominoType type)
{
	me->type = type;
    memcpy(me->grid4x4, shapes[type], sizeof(me->grid4x4));
    me->x = 0;
    me->y = 0;
    me->current_rotation = 0;
    me->speed = FPS / 2;
    me->moveCounter = 0;

}



void Tetromino_rotate(Tetromino *me)
{
    uint8_t tmp[4][4];

    // Copy original shape
    for(int r = 0; r < 4; r++){
        for(int c = 0; c < 4; c++){
            tmp[r][c] = me->grid4x4[r][c];
        }
    }

    // Rotate into t->grid4x4
    for(int r = 0; r < 4; r++){
        for(int c = 0; c < 4; c++){
        	me->grid4x4[c][3 - r] = tmp[r][c];
        }
    }

    me->current_rotation = (me->current_rotation +1) %4;
}
