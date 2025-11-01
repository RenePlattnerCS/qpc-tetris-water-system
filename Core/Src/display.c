#include "display.h"


static void display_text(char * text, uint8_t x, uint8_t y);

void display_temp(uint16_t temp)
{
	char buffer[16];
	sprintf(buffer, "%u", temp);
	display_text(buffer, 0,4 );
	display_text("C", 30,4 );
}



static void display_text(char * text, uint8_t x, uint8_t y) {
    //ssd1306_Fill(Black);

    ssd1306_SetCursor(x, y);

    ssd1306_WriteString(text, Font_16x26, White);

    ssd1306_UpdateScreen();
}
