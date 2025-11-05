
#include "display.h"
#include "tetris_board.h"

static void Board_draw_outline(const Board *me);

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


}

void Board_init(Board *me)
{
	clear_board(me);
	//Board_draw_outline(me);
}

static void Board_draw_outline(const Board *me)
{
    int w = me->width  * me->blockSize;
    int h = me->height * me->blockSize;

    int x0 = me->pos_x;
    int y0 = me->pos_y;

    // Top border
    draw_line(x0, y0, x0 + w - 1, y0);

    // Bottom border
    draw_line(x0, y0 + h - 1, x0 + w - 1, y0 + h - 1);

    // Left border
    draw_line(x0, y0, x0, y0 + h - 1);

    // Right border
    draw_line(x0 + w - 1, y0, x0 + w - 1, y0 + h - 1);
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

/*
void Board_placeTetromino(Board *me, const Tetromino *t)
{
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            if (t->grid4x4[row][col]) {
                int boardX = t->x + col;
                int boardY = t->y + row;

                if (boardX >= 0 && boardX < me->width &&
                    boardY >= 0 && boardY < me->height)
                {
                	me->grid[boardY * me->width + boardX] = t->grid4x4[row][col];
                }
            }
        }
    }
}
*/

void clear_board(Board *me)
{
	clear_display();
	memset(me->grid, 0, me->width * me->height);
}


void draw_board(Board *me)
{
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
	ssd1306_UpdateScreen();
}
