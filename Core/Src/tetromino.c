
#include <tetromino.h>
#include <tetris_shapes.h>

void Tetromino_ctor(Tetromino *me, TetrominoType type)
{
	me->type = type;
    memcpy(me->grid4x4, shapes[type], sizeof(me->grid4x4));
    me->x = 0;
    me->y = 0;
}
