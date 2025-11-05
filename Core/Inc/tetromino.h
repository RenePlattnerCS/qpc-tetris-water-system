#ifndef INC_TETROMINO_H_
#define INC_TETROMINO_H_


#include <stdint.h>

typedef enum {
    TETRO_I,
    TETRO_O,
    TETRO_T,
    TETRO_S,
    TETRO_Z,
    TETRO_J,
    TETRO_L
} TetrominoType;

typedef struct {
    TetrominoType type;
    uint8_t grid4x4[4][4];   // 4x4 shape
    int x;                // position on board
    int y;
} Tetromino;


void Tetromino_ctor(Tetromino *t, TetrominoType type);



#endif

