#ifndef _st7789_
#define _st7789_

#include "stm32f3xx_hal.h"
#include "spi.h"
#include "fonts.h"

#define RST_PIN     	RES_GPIO_Port, RES_Pin
#define DC_PIN      	DC_GPIO_Port, DC_Pin
//#define SPI_Port		hspi1

#define ST7789_TFTWIDTH 	240
#define ST7789_TFTHEIGHT 	240

#define ST7789_240x240_XSTART 0
#define ST7789_240x240_YSTART 0

#define WHITE				0xFFFF
#define BLACK				0x0000
#define BLUE 				0x001F
#define BRED 				0XF81F
#define GRED 				0XFFE0
#define GBLUE				0X07FF
#define RED  				0xF800
#define MAGENTA				0xF81F
#define GREEN				0x07E0
#define CYAN 				0x7FFF
#define YELLOW				0xFFE0
#define BROWN				0XBC40
#define BRRED				0XFC07
#define GRAY 				0X8430
#define DARKBLUE			0X01CF
#define LIGHTBLUE			0X7D7C
#define GRAYBLUE     		0X5458
#define LIGHTGREEN    		0X841F
#define LGRAY 			  	0XC618
#define LGRAYBLUE     		0XA651
#define LBBLUE        		0X2B12

#define ST_CMD_DELAY   0x80

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01

#define ST7789_SLPIN   0x10  // sleep on
#define ST7789_SLPOUT  0x11  // sleep off
#define ST7789_PTLON   0x12  // partial on
#define ST7789_NORON   0x13  // partial off
#define ST7789_INVOFF  0x20  // invert off
#define ST7789_INVON   0x21  // invert on
#define ST7789_DISPOFF 0x28  // display off
#define ST7789_DISPON  0x29  // display on
#define ST7789_IDMOFF  0x38  // idle off
#define ST7789_IDMON   0x39  // idle on

#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

#define ST7789_PTLAR    0x30   // partial start/end
#define ST7789_VSCRDEF  0x33   // SETSCROLLAREA
#define ST7789_VSCRSADD 0x37

#define ST7789_WRDISBV  0x51
#define ST7789_WRCTRLD  0x53
#define ST7789_WRCACE   0x55
#define ST7789_WRCABCMB 0x5e

#define ST7789_POWSAVE    0xbc
#define ST7789_DLPOFFSAVE 0xbd

// bits in MADCTL
#define ST7789_MADCTL_MY  0x80
#define ST7789_MADCTL_MX  0x40
#define ST7789_MADCTL_MV  0x20
#define ST7789_MADCTL_ML  0x10
#define ST7789_MADCTL_RGB 0x00

void st7789_WriteCmd(uint8_t cmd);
void st7789_WriteData(uint8_t data);
void st7789_WriteDataWord(uint16_t data);
void st7789_Reset();
void st7789_SetRotation(uint8_t mode);
void st7789_Init();
void st7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void st7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void st7789_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void st7789_FillScreen(uint16_t color);
void st7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t* image);
void st7789_DrawChar(uint16_t x, uint16_t y, const char ch, sFONT* Font, uint16_t BG_Color, uint16_t FG_Color);
void st7789_DrawString(uint16_t x, uint16_t y, const char * pString, sFONT* Font, uint16_t BG_Color, uint16_t FG_Color );

#endif
