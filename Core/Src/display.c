#include "display.h"


static void display_text(char * text, uint8_t x, uint8_t y);
static void RectangleFill(uint8_t percent);

void display_temp(uint16_t temp)
{
	ssd1306_Fill(Black);
	char buffer[16];
	sprintf(buffer, "%u", temp);
	display_text(buffer, 0,4 );
	display_text("C", 30,4 );
}

void display_dry(uint8_t dryness_percent)
{
	ssd1306_Fill(Black);
	char buffer[16];
	sprintf(buffer, "%u", dryness_percent);

	//display_text("Plant:", 0,6 );
	RectangleFill(dryness_percent);

}




static void display_text(char * text, uint8_t x, uint8_t y) {
    //ssd1306_Fill(Black);

    ssd1306_SetCursor(x, y);

    ssd1306_WriteString(text, Font_16x26, White);

    ssd1306_UpdateScreen();
}


static void RectangleFill(uint8_t percent) {
	uint8_t x2 = (SSD1306_WIDTH-1) * percent / 100;
	ssd1306_FillRectangle(1, 1, x2, SSD1306_HEIGHT-1, White);
	ssd1306_UpdateScreen();
}


