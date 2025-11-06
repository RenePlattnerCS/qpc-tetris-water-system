
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

    me->speed = FPS;

}



