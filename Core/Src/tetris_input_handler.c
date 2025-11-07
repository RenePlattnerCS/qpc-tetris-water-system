#include "tetris_input_handler.h"
#include <stdint.h>
#include "tetris_board.h"

uint8_t compute_move_delay(int16_t tilt) {
	tilt = abs(tilt);

    if (tilt < 3)  return 255;  // no movement (very flat)
    if (tilt < 10) return 12;   // slow
    if (tilt < 25) return 6;    // medium
    return 3;                   // fast
}

uint8_t compute_fall_delay(int16_t ytilt) {
    ytilt = abs(ytilt);
    if (ytilt < 25)  return 0;
    if (ytilt < 40)  return 75;   // no extra gravity
    if (ytilt < 55)  return 50;   // no extra gravity
    if (ytilt < 70) return 25;    // tilt a bit → falls faster
    if (ytilt < 100) return 8;    // tilt more → much faster
    return 1;                   // strong tilt → "soft drop"
}

bool process_tilt_move(Board *board, Tetromino *t, int16_t xtilt, int16_t ytilt) {
    ytilt *= -1; // because your axis is reversed
    if (!board || !t) return false;

    //
    // --- Horizontal movement ---
    //
    int dir = (xtilt > 0) - (xtilt < 0); // +1 right, -1 left, 0 none
    if (dir != 0) {
        uint8_t delay = compute_move_delay(xtilt);
        t->moveCounter++;
        if (t->moveCounter >= delay) {
            t->moveCounter = 0;
            if (!collision_on_move(board, t, dir, 0)) {
                t->x += dir;
            }
        }
    } else {
        // reset counter so movement is smooth when tilt starts again
        t->moveCounter = 0;
    }

    //
    // --- Vertical movement (fall speed depends on y tilt) ---
    //
    uint8_t fallDelay = compute_fall_delay(ytilt);
    if(fallDelay != 0)
    {
    	t->fallCounter++;
    	t->tickCounter = 0;
		if (t->fallCounter >= fallDelay) {
			t->fallCounter = 0;

			// Try to fall
			if (!collision_on_move(board, t, 0, -1)) {   // -1 = fall down in your coord system
				t->y--; // move down
			} else {
				// Collision: lock and spawn next piece
				return true;
			}
		}
    }


    return false;
}
