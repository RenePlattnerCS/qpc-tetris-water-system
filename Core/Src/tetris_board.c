
#include "display.h"
#include "tetris_board.h"
#include "app_config.h"

static void Board_draw_outline(const Board *me);
static void clear_screen(Board *me);

void Board_ctor(Board *me, uint8_t *grid,
                uint8_t width, uint8_t height, uint8_t pos_x , uint8_t pos_y, bool rotate_90, uint8_t blockSize)
{
	me->pos_x = pos_x;
	me->pos_y = pos_y;
	me->width = width;
	me->height = height;
	me->blockSize = blockSize;
	me->grid = grid;
	me->rotate_90 = rotate_90;
    memset(me->grid, 0, width * height);

    me->tickCounter = 0;



}

void Board_init(Board *me)
{
	ssd1306_Fill(Black);
}

static void Board_draw_outline(const Board *me)
{
	draw_border(me);
}

void Board_placeTetromino(Board *me, const Tetromino *t)
{
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            if (t->grid4x4[row][col] == 0) {
                continue; // skip empty cells
            }

            int boardX, boardY;

                // normal orientation
			boardX = t->x + col;
			boardY = t->y + row;


            // bounds check
            if (boardX >= 0 && boardX < me->width &&
                boardY >= 0 && boardY < me->height)
            {
                me->grid[boardY * me->width + boardX] = t->grid4x4[row][col];
            }
        }
    }
}



void clear_board(Board *me)
{
	clear_display();
	memset(me->grid, 0, me->width * me->height);
	Board_draw_outline(me);
}

static void clear_screen(Board *me)
{
	ssd1306_Fill(Black);
	Board_draw_outline(me);
}

bool move_down(Board *me, Tetromino *active)
{
	me->tickCounter++;
	if(me->tickCounter >= active->speed)
	{
		if(!collision_on_move(me,active, 0, -1)) {
			active->y -= 1;
			me->tickCounter = 0;
			return true;   // moved
		}
		else
		{
			return false;
		}


	}

}


bool collision_on_move(Board *me, const Tetromino *t, int dx, int dy)
{
	for(int row = 0; row < 4; row++) {
	        for(int col = 0; col < 4; col++) {

	            if(t->grid4x4[row][col]) {

	                int boardX, boardY;
					boardX = t->x + col + dx;
					boardY = t->y + row + dy;

	                // Bounds check (if outside → collision)
	                if(boardX < 0 || boardX >= me->width  ||
	                   boardY < 0 || boardY >= me->height)
	                {
	                    return true; // COLLISION
	                }

	                // Check collision with existing blocks
	                if(me->grid[boardY * me->width + boardX] != 0) {
	                    return true; // COLLISION
	                }
	            }
	        }
	    }

	    return false;
}


void lock_tetromino(Board *me, Tetromino *active)
{

}



void draw_board(Board *me, Tetromino *active)
{

	clear_screen(me);
	for (int row = 0; row < me->height; row++) {
	        for (int col = 0; col < me->width; col++) {
	            if (me->grid[row * me->width + col]) {

	            	if (row >= 0 && row < me->height &&
	            	    col >= 0 && col < me->width)
	                {
	            		if(me->rotate_90)
	            		{
	            			draw_pixel_block(me->pos_x + (row* me->blockSize) , me->pos_y + (col* me->blockSize), me->blockSize, me->blockSize);
	            		}
	            		else
	            		{
	            			//TODO
	            			//draw_pixel_block((me->pos_x + col) * me->blockSize  , (me->pos_y + row) * me->blockSize , me->blockSize, me->blockSize);
	            		}

	                }
	            }
	        }
	    }


	//active tetomino
	for(int row = 0; row < 4; row++) {
		for(int col = 0; col < 4; col++) {
			if(active->grid4x4[row][col]) {

				int x = active->x + col;
				int y = active->y + row;

				if(x >= 0 && x < me->width &&
				   x >= 0 && y < me->height)
				{
					if(me->rotate_90)
					{
						draw_pixel_block(me->pos_x + (y* me->blockSize) , me->pos_y + (x* me->blockSize), me->blockSize, me->blockSize);

					}

				}
			}
		}
	}
	ssd1306_UpdateScreen();
}
