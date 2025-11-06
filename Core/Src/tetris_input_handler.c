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

void process_tilt_move(Board *board, Tetromino *t, int16_t tilt) {

    int dir = (tilt > 0) - (tilt < 0); // right=1, left=-1, no tilt=0
    if (dir == 0) {
        t->moveCounter = 0;
        return;
    }

    uint8_t delay = compute_move_delay(tilt);

    t->moveCounter++;
    if (t->moveCounter >= delay) {
        t->moveCounter = 0;

        // Try move
        if (!collision_on_move(board, t, dir, 0)) {
            t->x += dir;
        }
    }
}
