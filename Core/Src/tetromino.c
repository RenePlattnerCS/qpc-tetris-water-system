
#include "tetromino.h""
#include "tetris_shapes.h"
#include "app_config.h"
void Tetromino_ctor(Tetromino *me, TetrominoType type)
{
	me->type = type;
    memcpy(me->grid4x4, shapes[type], sizeof(me->grid4x4));
    me->x = 0;
    me->y = 0;
    me->tickCounter = 0;
    me->speed = FPS;
}


void move_down(Tetromino *me)
{
	me->tickCounter++;
	if(me->tickCounter >= me->speed)
	{
		me->y -= 1;
		me->tickCounter = 0;
	}

}
