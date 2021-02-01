#ifndef _Context
#define _Context

#include "ST7789/fonts.h"
#include "ST7789/st7789.h"

#include "MPU9250/MPU9250.h"
#include "MPU9250/Madgwick.h"

#include "SD_SPI/fatfs_sd.h"

#include "spi.h"
#include "tim.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include "stm32f3xx_hal_spi.h"
#include "diskio.h"
#include "ff.h"

#include <math.h>
#include <stdio.h>

#define GForce_BufforSize 100
#define Calibration_Init_SampleSize 1000

#define RadToDeg 57.2957795f

#define GForce_Threashold_1 1.10f
#define GForce_Threashold_2 1.20f

#define Angle_Threashold_1 10.0f
#define Angle_Threashold_2 20.0f

#define Display_Spacing 5
#define DispRaw_Font &Font12
#define DispRaw_FontW 7
#define DispRaw_FontH 12

#define Display_ColumnStart Display_Spacing
#define Display_Acc_RowStart Display_Spacing + (Display_FontSize + Display_Spacing)*Display_Acc_RowNumber	//2 + 18*0
#define Display_Gyr_RowStart Display_Spacing + (Display_FontSize + Display_Spacing)*Display_Gyr_RowNumber	//2 + 18*2
#define Display_Mag_RowStart Display_Spacing + (Display_FontSize + Display_Spacing)*Display_Mag_RowNumber
#define Display_Temp_RowStart Display_Spacing + (Display_FontSize + Display_Spacing)*Display_Temp_RowNumber
#define Display_Reads_RowStart Display_Spacing + (Display_FontSize + Display_Spacing)*Display_Reads_RowNumber
#define Display_Pos_RowStart Display_Spacing + (Display_FontSize + Display_Spacing)*Display_Pos_RowNumber

#define SD_FileName "DataValues.txt"

typedef enum {
	Buzzer_Enabled,
	Buzzer_Disabled
} BuzzerStatus;

typedef enum {
	Display_Default,
	Display_Position,
	Display_Raw
} DisplayMode;

typedef struct {
	DisplayMode mode;
	BuzzerStatus status;

	int shouldChangeMode;
	int shouldCalibrate;
	int shouldUpdateDisplay;
	int shouldSaveToSD;

	FATFS fs;  		// file system
	FIL fil;  		// file

	float GForce_Base;
	float GForce_Mean;
	float GForce_LocalMax;

	float GForce_Buffor[GForce_BufforSize];
	int GForce_Buffor_Index;

	float BaseX, BaseY, BaseZ;	//base Position Vector in space
	float PosX, PosY, PosZ;		//current Position Vector in space

	float BaseVectorLength;
	float Angle;


	int _DisableSD;
	volatile int MeasureCount;

} ContextData;

void ProgramInit(void);
void AwaitForSignal(void);
void ProcessValues(void);

void SetToChangeDisplayMode(void);
void SetToUpdateDisplay(void);
void SetToCalibrate(void);
void SetToSaveData(void);

/*		Private		*/

void _SD_Init(void);
void _SaveToSD(void);

void _CalibrateInit(void);
void _Calibrate(void);

void _PreUpdate(void);
void _UpdateDisplay(void);
void _PostUpdate(void);

void _TestDisplay(void);
void _Prepare_DrawDisplay(DisplayMode);
void _DrawDisplay(DisplayMode);

void _Buzzer_Enable(void);
void _Buzzer_Disable(void);

void _Calculate_GForce(void);
void _Calculate_Position(void);
void _Calculate_Angle(void);
void _PosToRadians(void);


#endif
