
#include "display.h"
#include "tetris_board.h"
#include "app_config.h"
#include <string.h>

static void Board_draw_outline(const Board  * const me);
static void clear_screen(Board *me);
static void clear_line(Board * me, int line_to_clear);
static bool is_line_full(Board * me, int row);

static uint8_t dummyValue = 0;


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


static void Board_draw_outline(const Board  * const me)
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
	active->tickCounter++;
	if(active->tickCounter >= active->speed)
	{
		if(!collision_on_move(me,active, 0, -1)) {
			active->y -= 1;
			active->tickCounter = 0;
			return true;   // moved
		}
		else
		{
			return false;
		}


	}
	return true;
}

void rotate_tetromino_collision_check(Board * me, Tetromino* t) {
    // Save current rotation
	uint8_t old_rotation = t->current_rotation;
	uint8_t old_grid4x4[4][4];
	memcpy(old_grid4x4, t->grid4x4, sizeof(old_grid4x4));

    // Try new rotation
    Tetromino_rotate(t);

    // Check collision
    if (collision_on_move(me,t, 0, 0)) {
        // Revert if collision detected
    	t->current_rotation = old_rotation;
    	memcpy(t->grid4x4, old_grid4x4, sizeof(t->grid4x4));
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


bool collision_on_spawn(Board *me, Tetromino *t)
{
    for(int row = 0; row < 4; row++) {
        for(int col = 0; col < 4; col++) {

            if(t->grid4x4[row][col]) {

                int x = t->x + col;
                int y = t->y + row;

                // if ANY part is already filled -> GAME OVER
                if (me->grid[y * me->width + x] != 0) {
                		dummyValue = 1;
                	    return true; // out of bounds = collision
                }
            }
        }
    }
    return false;
}

void draw_board(Board *me, Tetromino *active, uint8_t score)
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

				if (x >= 0 && x < me->width &&
				    y >= 0 && y < me->height)
				{
					if(me->rotate_90)
					{
						draw_pixel_block(me->pos_x + (y* me->blockSize) , me->pos_y + (x* me->blockSize), me->blockSize, me->blockSize);

					}

				}
			}
		}
	}

	display_score(score);
	ssd1306_UpdateScreen();
}


// Check if a row is full (all cells occupied)
static bool is_line_full(Board * me, int row) {
    for (int col = 0; col < me->width; col++) {
        if (me->grid[row * me->width + col] == 0) {
            return false;
        }
    }
    return true;
}

static void clear_line(Board *me, int line_to_clear) {
    // Shift all rows ABOVE the cleared line DOWN by one
    for (int row = line_to_clear; row < me->height - 1; row++) {
        for (int col = 0; col < me->width; col++) {
            me->grid[row * me->width + col] = me->grid[(row + 1) * me->width + col];
        }
    }

    // Clear the TOP row (highest row number)
    for (int col = 0; col < me->width; col++) {
        me->grid[(me->height - 1) * me->width + col] = 0;
    }
}

// Check all lines and clear completed ones
 uint8_t check_and_clear_lines(Board * me) {
	 uint8_t lines_cleared = 0;
	 for (int row = 0; row < me->height; row++) {
	     if (is_line_full(me, row)) {
	         clear_line(me, row);
	         lines_cleared++;
	         row--;
	     }
	 }
	 return lines_cleared;
}

