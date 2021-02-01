#include "ST7789/st7789.h"

uint8_t  _st7789_colstart, _st7789_rowstart, _st7789_xstart, _st7789_ystart;
uint16_t _st7789_width, _st7789_height;

SPI_HandleTypeDef* SPI_PORT = &hspi1;

void st7789_WriteCmd(uint8_t cmd)
{
	HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(SPI_PORT, &cmd, 1, 10);
}

void st7789_WriteData(uint8_t data)
{
	HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_SET);
	HAL_SPI_Transmit(SPI_PORT, &data, 1, 10);
}

void st7789_WriteDataWord(uint16_t data)
{
	data = (data >> 8) | (data << 8);
	HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_SET);
	HAL_SPI_Transmit(SPI_PORT, (uint8_t*)&data, 2, 10);

	//alternative
	//uint8_t b1 = data >> 8;
	//uint8_t b2 = data & 0xFF;
	//HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_SET);
	//HAL_SPI_Transmit(SPI_PORT, &b1, 1, 10);
	//HAL_SPI_Transmit(SPI_PORT, &b2, 1, 10);
}

void st7789_Reset()
{
	//hardware reset
	HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_SET);
	HAL_Delay(150);

	//software reset
	//st7789_WriteCmd(ST7789_SWRESET);
	//HAL_Delay(150);
}

void st7789_SetRotation(uint8_t rotation)
{
	st7789_WriteCmd(ST7789_MADCTL);

	rotation &= 3;
	  switch (rotation) {
	   case 0:
		   st7789_WriteData(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
		   _st7789_xstart = _st7789_colstart;
		   _st7789_ystart = _st7789_rowstart;
	     break;
	   case 1:
		   st7789_WriteData(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		   _st7789_ystart = _st7789_colstart;
		   _st7789_xstart = _st7789_rowstart;
	     break;
	  case 2:
		  st7789_WriteData(ST7789_MADCTL_RGB);
		  _st7789_xstart = _st7789_colstart;
		  _st7789_ystart = _st7789_rowstart;
	     break;
	   case 3:
		   st7789_WriteData(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		   _st7789_ystart = _st7789_colstart;
		   _st7789_xstart = _st7789_rowstart;
	     break;
	  }
}

void st7789_Init()
{
	//initialize display begin and end dimensions
	//allows to only use inner part of a display ie.: 100x100 in 240x240 starting at (50, 50)
	_st7789_colstart = _st7789_xstart = ST7789_240x240_XSTART;
	_st7789_ystart = _st7789_rowstart = ST7789_240x240_YSTART;
	_st7789_width = ST7789_TFTWIDTH;
	_st7789_height = ST7789_TFTHEIGHT;

	st7789_Reset();

	st7789_WriteCmd(ST7789_MADCTL);
	st7789_WriteData(0x00);

	st7789_WriteCmd(ST7789_COLMOD);
	st7789_WriteData(0x05);


	//st7789_WriteCmd(0xB2);	//ST7735_Cmd_FRMCTR2
	//st7789_WriteData(0x0C);
	//st7789_WriteData(0x0C);
	//st7789_WriteData(0x00);
	//st7789_WriteData(0x33);
	//st7789_WriteData(0x33);

	//st7789_WriteCmd(0xB7); 	//Gate Control
	//st7789_WriteData(0x35);

	//st7789_WriteCmd(0xBB);	//VCOM Settings
	//st7789_WriteData(0x19);

	//st7789_WriteCmd(0xC0);	//ST7735_Cmd_PWCTR1 / LCM Control
	//st7789_WriteData(0x2C);

	//st7789_WriteCmd(0xC2);
	//st7789_WriteData(0x01);

	//st7789_WriteCmd(0xC3);
	//st7789_WriteData(0x12);

	//st7789_WriteCmd(0xC4);
	//st7789_WriteData(0x20);

	//st7789_WriteCmd(0xC6); //frame rate control
	//st7789_WriteData(0x0F);

	//st7789_WriteCmd(0xD0); //power control1
	//st7789_WriteData(0xA4);
	//st7789_WriteData(0xA1);


	st7789_WriteCmd(ST7789_INVON);

	st7789_WriteCmd(ST7789_SLPOUT);

	st7789_WriteCmd(ST7789_NORON);

	st7789_WriteCmd(ST7789_DISPON);


	//display orientation
	st7789_SetRotation(2);
}

void st7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	uint16_t xs = x0 + _st7789_xstart, xe = x1 + _st7789_xstart;
	uint16_t ys = y0 + _st7789_ystart, ye = y1 + _st7789_ystart;

	st7789_WriteCmd(ST7789_CASET);
	st7789_WriteDataWord(xs);
	st7789_WriteDataWord(xe);

	st7789_WriteCmd(ST7789_RASET);
	st7789_WriteDataWord(ys);
	st7789_WriteDataWord(ye);

	st7789_WriteCmd(ST7789_RAMWR);
}

void st7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	//disable check for performance
	//if( x>=_width || y>=_height) return;
	st7789_SetWindow(x, y, x+1, y+1);
	st7789_WriteDataWord(color);
}

void st7789_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  //if(x>=_width || y>=_height) return;
  if( x+w > _st7789_width )  w = _st7789_width - x;
  if( y+h > _st7789_height )  h = _st7789_height - y;
  st7789_SetWindow(x, y, x+w - 1, y+h - 1);

  HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_SET);
  uint16_t data = (color >> 8) | (color << 8);


  for(int i = 0; i < (uint32_t)w*h; i++) {
	  HAL_SPI_Transmit(SPI_PORT, (uint8_t*)&data, 2, 10);
  }
}


void st7789_FillScreen(uint16_t color)
{
	st7789_FillRectangle(_st7789_xstart, _st7789_ystart, _st7789_width, _st7789_height, color);
}


void st7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t* image)
{
	  if( x >= _st7789_width || y >= _st7789_height || w <= 0 || h <= 0) return;
	  st7789_SetWindow(x, y, x+w - 1, y+h - 1);

	  HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_SET);
	  uint8_t* img = (uint8_t*)image;
	  for(int i = 0; i < (uint32_t)w*h*2; i+=2)
	  {
		  HAL_SPI_Transmit(SPI_PORT, &img[i+1], 1, 10);
		  HAL_SPI_Transmit(SPI_PORT, &img[i], 1, 10);
	  }
}

void st7789_DrawChar(uint16_t x, uint16_t y, const char ch, sFONT* Font, uint16_t BG_Color, uint16_t FG_Color)
{
	//if( x >= _st7789_width || y >= _st7789_height ) return;

	uint32_t Char_Offset = (ch - ' ') * Font->Height * (Font->Width / 8 + (Font->Width % 8 ? 1 : 0));
	const unsigned char *ptr = &Font->table[Char_Offset];


	st7789_SetWindow(x, y, x+Font->Width - 1, y+Font->Height - 1);
	HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_SET);

	uint16_t fg = (FG_Color >> 8) | (FG_Color << 8);
	uint16_t bg = (BG_Color >> 8) | (BG_Color << 8);

	for (int Page = 0; Page < Font->Height; Page ++ )
	{
		for (int Column = 0; Column < Font->Width; Column ++ )
		{
			if (*ptr & (0x80 >> (Column % 8)))
			{
				HAL_SPI_Transmit(SPI_PORT, (uint8_t*)&fg, 2, 10);
				//st7789_DrawPixel(x + Column, y + Page, FG_Color);
			}
			else
			{
				HAL_SPI_Transmit(SPI_PORT, (uint8_t*)&bg, 2, 10);
				//st7789_DrawPixel(x + Column, y + Page, BG_Color);
			}

			//One pixel is 8 bits
			if (Column % 8 == 7)
				ptr++;
		}// Write a line
		if (Font->Width % 8 != 0)
			ptr++;
	}// Write all
}

void st7789_DrawString(uint16_t x, uint16_t y, const char * pString, sFONT* Font, uint16_t BG_Color, uint16_t FG_Color )
{
	//if ( x > _st7789_width || y > _st7789_height ) return;

	uint16_t Xpoint = x;
	uint16_t Ypoint = y;

	while (* pString != '\0')
	{
		//if X direction filled , reposition to(Xstart,Ypoint),Ypoint is Y direction plus the Height of the character
		if ((Xpoint + Font->Width ) > _st7789_width ) {
			Xpoint = x;
			Ypoint += Font->Height;
		}

		// If the Y direction is full, reposition to(Xstart, Ystart)
		if ((Ypoint  + Font->Height ) > _st7789_height ) {
			Xpoint = x;
			Ypoint = y;
		}
		st7789_DrawChar(Xpoint, Ypoint, * pString, Font, BG_Color, FG_Color);

		pString ++;
		Xpoint += Font->Width;
	}
}
