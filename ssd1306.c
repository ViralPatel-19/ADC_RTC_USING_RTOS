/**
 ********************************************************************************
 * @file    ssd1306.c
 * @brief
 * @author  Lenovo
 * @date    Mar 21, 2023
 * @copyright (c) 2022 Engibrains. All rights reserved.
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
//#include "stm32f756xx.h"
#include "cyhal.h"
#include "cybsp.h"
#include "ssd1306.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>  // For memcpy
#include "ssd1306_port.h"
uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];

// Screen object
static SSD1306_t SSD1306;

//GPIO_TypeDef* SSD1306_CS_Port = GPIOC;
uint32_t SSD1306_CS_Pin = 0;
fpSSD1306WriteCommand cbSSD1306WriteCommand;
fpSSD1306WriteData cbSSD1306WriteData;
fpSSD1306Reset cbSSD1306Reset;

/* Fills the Screenbuffer with values from a given buffer of a fixed length */
SSD1306_Error_t ssd1306_FillBuffer(uint8_t* buf, uint32_t len) {
	SSD1306_Error_t ret = SSD1306_ERR;
	if (len <= SSD1306_BUFFER_SIZE) {
		memcpy(SSD1306_Buffer,buf,len);
		ret = SSD1306_OK;
	}
	return ret;
}

void ssd1306_InitDriver(fpSSD1306WriteCommand ssd1306WriteCommand, fpSSD1306WriteData ssd1306WriteData, fpSSD1306Reset ssd1306Reset)
{
	cbSSD1306WriteCommand = ssd1306WriteCommand;
	cbSSD1306WriteData = ssd1306WriteData;
	cbSSD1306Reset = ssd1306Reset;
}

/* Initialize the oled screen */
void ssd1306_Init(void)
{
	// Initialize ssd1306 driver
	ssd1306_InitDriver(&ssd1306_WriteCommand, &ssd1306_WriteData, &ssd1306_Reset);

	// Reset OLED
	//ssd1306_Reset();
	cbSSD1306Reset();

	// Wait for the screen to boot
	Cy_SysLib_Delay(100);

	// Init OLED
	ssd1306_SetDisplayOn(0); //display off

	cbSSD1306WriteCommand(0x20); //Set Memory Addressing Mode
	cbSSD1306WriteCommand(0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
	// 10b,Page Addressing Mode (RESET); 11b,Invalid

	cbSSD1306WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7

#ifdef SSD1306_MIRROR_VERT
	cbSSD1306WriteCommand(0xC0); // Mirror vertically
#else
	cbSSD1306WriteCommand(0xC8); //Set COM Output Scan Direction
#endif

	cbSSD1306WriteCommand(0x00); //---set low column address
	cbSSD1306WriteCommand(0x10); //---set high column address

	cbSSD1306WriteCommand(0x40); //--set start line address - CHECK

	ssd1306_SetContrast(0xFF);

#ifdef SSD1306_MIRROR_HORIZ
	cbSSD1306WriteCommand(0xA0); // Mirror horizontally
#else
	cbSSD1306WriteCommand(0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef SSD1306_INVERSE_COLOR
	cbSSD1306WriteCommand(0xA7); //--set inverse color
#else
	cbSSD1306WriteCommand(0xA6); //--set normal color
#endif

	// Set multiplex ratio.
#if (SSD1306_HEIGHT == 128)
	// Found in the Luma Python lib for SH1106.
	cbSSD1306WriteCommand(0xFF);
#else
	cbSSD1306WriteCommand(0xA8); //--set multiplex ratio(1 to 64) - CHECK
#endif

#if (SSD1306_HEIGHT == 32)
	cbSSD1306WriteCommand(0x1F); //
#elif (SSD1306_HEIGHT == 64)
	cbSSD1306WriteCommand(0x3F); //
#elif (SSD1306_HEIGHT == 128)
	cbSSD1306WriteCommand(0x3F); // Seems to work for 128px high displays too.
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

	cbSSD1306WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

	cbSSD1306WriteCommand(0xD3); //-set display offset - CHECK
	cbSSD1306WriteCommand(0x00); //-not offset

	cbSSD1306WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
	cbSSD1306WriteCommand(0xF0); //--set divide ratio

	cbSSD1306WriteCommand(0xD9); //--set pre-charge period
	cbSSD1306WriteCommand(0x22); //

	cbSSD1306WriteCommand(0xDA); //--set com pins hardware configuration - CHECK
#if (SSD1306_HEIGHT == 32)
	cbSSD1306WriteCommand(0x02);
#elif (SSD1306_HEIGHT == 64)
	cbSSD1306WriteCommand(0x12);
#elif (SSD1306_HEIGHT == 128)
	cbSSD1306WriteCommand(0x12);
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

	cbSSD1306WriteCommand(0xDB); //--set vcomh
	cbSSD1306WriteCommand(0x20); //0x20,0.77xVcc

	cbSSD1306WriteCommand(0x8D); //--set DC-DC enable
	cbSSD1306WriteCommand(0x14); //

	ssd1306_SetDisplayOn(1); //--turn on SSD1306 panel

	// Clear screen
	ssd1306_Fill(Black);

	// Flush buffer to screen
	ssd1306_UpdateScreen();

	// Set default values for screen object
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;

	SSD1306.Initialized = 1;
	SSD1306.DisplayOn = 1;
}

/* Fill the whole screen with the given color */
void ssd1306_Fill(SSD1306_COLOR color) {
	uint32_t i;

	for(i = 0; i < sizeof(SSD1306_Buffer); i++) {
		SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
	}
}

/* Write the screenbuffer with changed to the screen */
void ssd1306_UpdateScreen(void) {
	// Write data to each page of RAM. Number of pages
	// depends on the screen height:
	//
	//  * 32px   ==  4 pages
	//  * 64px   ==  8 pages
	//  * 128px  ==  16 pages
	    for(uint8_t i = 0; i < SSD1306_HEIGHT/8; i++) {
	        cbSSD1306WriteCommand(0xB0 + i); // Set the current RAM page address.
	        cbSSD1306WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER);
	        cbSSD1306WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER);
	        ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH*i],SSD1306_WIDTH);
	    }
//	cbSSD1306WriteCommand(0xB0);
//	cbSSD1306WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER);
//	cbSSD1306WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER);
//	cbSSD1306WriteData(SSD1306_Buffer,sizeof(SSD1306_Buffer));
}
void ssd1306_UpdateScreenp1(void) {
    uint8_t page = 1; // Specify the page to update (page 1)

    cbSSD1306WriteCommand(0xB0 + page); // Set the current RAM page address.
    cbSSD1306WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER); // Set the lower column address.
    cbSSD1306WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER); // Set the upper column address.

    // Update only page 1 data
    ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH * page], SSD1306_WIDTH);
}
void ssd1306_UpdateScreenp2(void) {
    uint8_t page = 2; // Specify the page to update (page 1)

    cbSSD1306WriteCommand(0xB0 + page); // Set the current RAM page address.
    cbSSD1306WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER); // Set the lower column address.
    cbSSD1306WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER); // Set the upper column address.

    // Update only page 1 data
    ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH * page], SSD1306_WIDTH);
}
void ssd1306_UpdateScreenp3(void) {
    uint8_t page = 3; // Specify the page to update (page 1)

    cbSSD1306WriteCommand(0xB0 + page); // Set the current RAM page address.
    cbSSD1306WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER); // Set the lower column address.
    cbSSD1306WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER); // Set the upper column address.

    // Update only page 1 data
    ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH * page], SSD1306_WIDTH);
}
void ssd1306_UpdateScreenp4(void) {
    uint8_t page = 5; // Specify the page to update (page 1)

    cbSSD1306WriteCommand(0xB0 + page); // Set the current RAM page address.
    cbSSD1306WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER); // Set the lower column address.
    cbSSD1306WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER); // Set the upper column address.

    // Update only page 1 data
    ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH * page], SSD1306_WIDTH);
}

/* Write the screenbuffer with changed to the screen */
void ssd1306_UpdateCurValueLine(void) {
	// Write data to each page of RAM. Number of pages
	// depends on the screen height:
	//
	//  * 32px   ==  4 pages
	//  * 64px   ==  8 pages
	//  * 128px  ==  16 pages
	//    for(uint8_t i = CURRENT_LINE_START_PAGE; i < CURRENT_LINE_PAGES; i++) {
	//        cbSSD1306WriteCommand(0xB0 + i); // Set the current RAM page address.
	//        cbSSD1306WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER);
	//        cbSSD1306WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER);
	//        cbSSD1306WriteData(&SSD1306_Buffer[SSD1306_WIDTH*i],SSD1306_WIDTH);
	//    }
	cbSSD1306WriteCommand(0xB0); // Set the current RAM page address.
	cbSSD1306WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER);
	cbSSD1306WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER);
	cbSSD1306WriteData(&SSD1306_Buffer[SSD1306_WIDTH],SSD1306_WIDTH);
}

/* Write the screenbuffer with changed to the screen */
void ssd1306_UpdateInfoLine(void) {
	// Write data to each page of RAM. Number of pages
	// depends on the screen height:
	//
	//  * 32px   ==  4 pages
	//  * 64px   ==  8 pages
	//  * 128px  ==  16 pages
	//    for(uint8_t i = INFO_LINE_START_PAGE ; i < INFO_LINE_START_PAGE + INFO_LINE_PAGES; i++) {
	//        cbSSD1306WriteCommand(0xB0 + i); // Set the current RAM page address.
	//        cbSSD1306WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER);
	//        cbSSD1306WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER);
	//        cbSSD1306WriteData(&SSD1306_Buffer[SSD1306_WIDTH*i],SSD1306_WIDTH);
	//    }
	cbSSD1306WriteCommand(0xB0); // Set the current RAM page address.
	cbSSD1306WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER);
	cbSSD1306WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER);
	cbSSD1306WriteData(&SSD1306_Buffer[SSD1306_WIDTH],SSD1306_WIDTH);
}

/*
 * Draw one pixel in the screenbuffer
 * X => X Coordinate
 * Y => Y Coordinate
 * color => Pixel color
 */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color) {
	if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
		// Don't write outside the buffer
		return;
	}

	// Draw in the right color
	if(color == White) {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

/*
 * Draw 1 char to the screen buffer
 * ch       => char om weg te schrijven
 * Font     => Font waarmee we gaan schrijven
 * color    => Black or White
 */
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color) {
	uint32_t i, b, j;

	// Check if character is valid
	if (ch < 32 || ch > 126)
		return 0;

	// Check remaining space on current line
	if (SSD1306_WIDTH < (SSD1306.CurrentX + Font.FontWidth) ||
			SSD1306_HEIGHT < (SSD1306.CurrentY + Font.FontHeight))
	{
		// Not enough space on current line
		return 0;
	}

	// Use the font to write
	for(i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for(j = 0; j < Font.FontWidth; j++)
		{
			if(Font.FontWidth >= 20)
			{
				uint32_t index = (ch - 32) * Font.FontHeight + i;
				b = *(uint32_t *)((uint32_t*)Font.data + index);
				if((b << j) & 0x80000)
				{
					ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
				}
				else
				{
					ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
				}
			}
			else
			{
				if((b << j) & 0x8000)
				{
					ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
				}
				else
				{
					ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
				}
			}

		}
	}

	// The current space is now taken
	SSD1306.CurrentX += Font.FontWidth;

	// Return written char for validation
	return ch;
}

/* Write full string to screenbuffer */
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color) {
	while (*str) {
		if (ssd1306_WriteChar(*str, Font, color) != *str) {
			// Char could not be written
			return *str;
		}
		str++;
	}

	// Everything ok
	return *str;
}

/* Position the cursor */
void ssd1306_SetCursor(uint8_t x, uint8_t y) {
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

/* Draw line by Bresenhem's algorithm */
void ssd1306_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color) {
	int32_t deltaX = abs(x2 - x1);
	int32_t deltaY = abs(y2 - y1);
	int32_t signX = ((x1 < x2) ? 1 : -1);
	int32_t signY = ((y1 < y2) ? 1 : -1);
	int32_t error = deltaX - deltaY;
	int32_t error2;

	ssd1306_DrawPixel(x2, y2, color);

	while((x1 != x2) || (y1 != y2)) {
		ssd1306_DrawPixel(x1, y1, color);
		error2 = error * 2;
		if(error2 > -deltaY) {
			error -= deltaY;
			x1 += signX;
		}

		if(error2 < deltaX) {
			error += deltaX;
			y1 += signY;
		}
	}
	return;
}

/* Draw polyline */
void ssd1306_Polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color) {
	uint16_t i;
	if(par_vertex == NULL) {
		return;
	}

	for(i = 1; i < par_size; i++) {
		ssd1306_Line(par_vertex[i - 1].x, par_vertex[i - 1].y, par_vertex[i].x, par_vertex[i].y, color);
	}

	return;
}

/* Convert Degrees to Radians */
static float ssd1306_DegToRad(float par_deg) {
	return par_deg * 3.14 / 180.0;
}

/* Normalize degree to [0;360] */
static uint16_t ssd1306_NormalizeTo0_360(uint16_t par_deg) {
	uint16_t loc_angle;
	if(par_deg <= 360) {
		loc_angle = par_deg;
	} else {
		loc_angle = par_deg % 360;
		loc_angle = ((par_deg != 0)?par_deg:360);
	}
	return loc_angle;
}

/*
 * DrawArc. Draw angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle in degree
 * sweep in degree
 */
void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color) {
	static const uint8_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
	float approx_degree;
	uint32_t approx_segments;
	uint8_t xp1,xp2;
	uint8_t yp1,yp2;
	uint32_t count = 0;
	uint32_t loc_sweep = 0;
	float rad;

	loc_sweep = ssd1306_NormalizeTo0_360(sweep);

	count = (ssd1306_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
	approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
	approx_degree = loc_sweep / (float)approx_segments;
	while(count < approx_segments)
	{
		rad = ssd1306_DegToRad(count*approx_degree);
		xp1 = x + (int8_t)(sin(rad)*radius);
		yp1 = y + (int8_t)(cos(rad)*radius);
		count++;
		if(count != approx_segments) {
			rad = ssd1306_DegToRad(count*approx_degree);
		} else {
			rad = ssd1306_DegToRad(loc_sweep);
		}
		xp2 = x + (int8_t)(sin(rad)*radius);
		yp2 = y + (int8_t)(cos(rad)*radius);
		ssd1306_Line(xp1,yp1,xp2,yp2,color);
	}

	return;
}

/*
 * Draw arc with radius line
 * Angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle: start angle in degree
 * sweep: finish angle in degree
 */
void ssd1306_DrawArcWithRadiusLine(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color) {
	static const uint8_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
	float approx_degree;
	uint32_t approx_segments;
	uint8_t xp1 = 0;
	uint8_t xp2 = 0;
	uint8_t yp1 = 0;
	uint8_t yp2 = 0;
	uint32_t count = 0;
	uint32_t loc_sweep = 0;
	float rad;

	loc_sweep = ssd1306_NormalizeTo0_360(sweep);

	count = (ssd1306_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
	approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
	approx_degree = loc_sweep / (float)approx_segments;

	rad = ssd1306_DegToRad(count*approx_degree);
	uint8_t first_point_x = x + (int8_t)(sin(rad)*radius);
	uint8_t first_point_y = y + (int8_t)(cos(rad)*radius);
	while (count < approx_segments) {
		rad = ssd1306_DegToRad(count*approx_degree);
		xp1 = x + (int8_t)(sin(rad)*radius);
		yp1 = y + (int8_t)(cos(rad)*radius);
		count++;
		if (count != approx_segments) {
			rad = ssd1306_DegToRad(count*approx_degree);
		} else {
			rad = ssd1306_DegToRad(loc_sweep);
		}
		xp2 = x + (int8_t)(sin(rad)*radius);
		yp2 = y + (int8_t)(cos(rad)*radius);
		ssd1306_Line(xp1,yp1,xp2,yp2,color);
	}

	// Radius line
	ssd1306_Line(x,y,first_point_x,first_point_y,color);
	ssd1306_Line(x,y,xp2,yp2,color);
	return;
}

/* Draw circle by Bresenhem's algorithm */
void ssd1306_DrawCircle(uint8_t par_x,uint8_t par_y,uint8_t par_r,SSD1306_COLOR par_color) {
	int32_t x = -par_r;
	int32_t y = 0;
	int32_t err = 2 - 2 * par_r;
	int32_t e2;

	if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT) {
		return;
	}

	do {
		ssd1306_DrawPixel(par_x - x, par_y + y, par_color);
		ssd1306_DrawPixel(par_x + x, par_y + y, par_color);
		ssd1306_DrawPixel(par_x + x, par_y - y, par_color);
		ssd1306_DrawPixel(par_x - x, par_y - y, par_color);
		e2 = err;

		if (e2 <= y) {
			y++;
			err = err + (y * 2 + 1);
			if(-x == y && e2 <= x) {
				e2 = 0;
			}
		}

		if (e2 > x) {
			x++;
			err = err + (x * 2 + 1);
		}
	} while (x <= 0);

	return;
}

/* Draw filled circle. Pixel positions calculated using Bresenham's algorithm */
void ssd1306_FillCircle(uint8_t par_x,uint8_t par_y,uint8_t par_r,SSD1306_COLOR par_color) {
	int32_t x = -par_r;
	int32_t y = 0;
	int32_t err = 2 - 2 * par_r;
	int32_t e2;

	if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT) {
		return;
	}

	do {
		for (uint8_t _y = (par_y + y); _y >= (par_y - y); _y--) {
			for (uint8_t _x = (par_x - x); _x >= (par_x + x); _x--) {
				ssd1306_DrawPixel(_x, _y, par_color);
			}
		}

		e2 = err;
		if (e2 <= y) {
			y++;
			err = err + (y * 2 + 1);
			if (-x == y && e2 <= x) {
				e2 = 0;
			}
		}

		if (e2 > x) {
			x++;
			err = err + (x * 2 + 1);
		}
	} while (x <= 0);

	return;
}

/* Draw a rectangle */
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color) {
	ssd1306_Line(x1,y1,x2,y1,color);
	ssd1306_Line(x2,y1,x2,y2,color);
	ssd1306_Line(x2,y2,x1,y2,color);
	ssd1306_Line(x1,y2,x1,y1,color);

	return;
}

/* Draw a filled rectangle */
void ssd1306_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color) {
	uint8_t x_start = ((x1<=x2) ? x1 : x2);
	uint8_t x_end   = ((x1<=x2) ? x2 : x1);
	uint8_t y_start = ((y1<=y2) ? y1 : y2);
	uint8_t y_end   = ((y1<=y2) ? y2 : y1);

	for (uint8_t y= y_start; (y<= y_end)&&(y<SSD1306_HEIGHT); y++) {
		for (uint8_t x= x_start; (x<= x_end)&&(x<SSD1306_WIDTH); x++) {
			ssd1306_DrawPixel(x, y, color);
		}
	}
	return;
}

/* Draw a bitmap */
void ssd1306_DrawBitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color) {
	int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
	uint8_t byte = 0;

	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
		return;
	}

	for (uint8_t j = 0; j < h; j++, y++) {
		for (uint8_t i = 0; i < w; i++) {
			if (i & 7) {
				byte <<= 1;
			} else {
				byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
			}

			if (byte & 0x80) {
				ssd1306_DrawPixel(x + i, y, color);
			}
		}
	}
	return;
}

void ssd1306_SetContrast(const uint8_t value) {
	const uint8_t kSetContrastControlRegister = 0x81;
	cbSSD1306WriteCommand(kSetContrastControlRegister);
	cbSSD1306WriteCommand(value);
}

void ssd1306_SetDisplayOn(const uint8_t on) {
	uint8_t value;
	if (on) {
		value = 0xAF;   // Display on
		SSD1306.DisplayOn = 1;
	} else {
		value = 0xAE;   // Display off
		SSD1306.DisplayOn = 0;
	}
	cbSSD1306WriteCommand(value);
}

uint8_t ssd1306_GetDisplayOn() {
	return SSD1306.DisplayOn;
}




