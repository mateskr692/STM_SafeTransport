#include "Context.h"

	//volatile ContextData ctx;
	//extern volatile MadgwickData _AHRS;

volatile ContextData ctx;
volatile extern AHRS _ahrs;

void ProgramInit()
{
	ctx._DisableSD = 0;
	ctx.status = Buzzer_Disabled;
	ctx.mode = Display_Default;
	htim2.Instance->CCR1 = 20;		//frequency of the buzzer
	ctx.GForce_Buffor_Index = 0; 	//Start Index for the Gforce cyclic buffor

	//initialize Oled display and flash it white
	st7789_Init();
	st7789_FillScreen(WHITE);

	//set up neccesery flag for the MPU
	st7789_DrawString(10, 10, "Initializing Sensors", &Font16, WHITE, RED);
	MPU_Init();
	MPU_begin();

	//connect to the SD and choose a file
	st7789_DrawString(10, 30, "Initializing Memory", &Font16, WHITE, BLUE);
	_SD_Init();

	//enable reading from the MPU and calculate the base vectors
	st7789_DrawString(10, 50, "Calibrating Sensors", &Font16, WHITE, GREEN);
	AHRS_Init();
	HAL_TIM_Base_Start_IT(&htim4);	//starts the MPU timer
	_CalibrateInit();

	//enable updating the display
	HAL_TIM_Base_Start_IT(&htim3);	//starts the display Update timer
	HAL_TIM_Base_Start_IT(&htim8);	//starts the SD Card Save timer

	_Prepare_DrawDisplay(ctx.mode);
	SetToUpdateDisplay();
}

//check for flags from interrupts to calibrate / update screen
void AwaitForSignal()
{
	if( ctx.shouldCalibrate == 1)
	{
		_Calibrate();
		//HAL_Delay(1);

		ctx.shouldCalibrate = 0;
	}

	if( ctx.shouldUpdateDisplay == 1)
	{
		_PreUpdate();		//calculates indirect values for display based on sesnoor readings
		_UpdateDisplay();	//displays data
		_PostUpdate();		//toogles buzzers, saves data to memory, resets local maximum (GForce)

		ctx.shouldUpdateDisplay = 0;
	}

	if( ctx.shouldSaveToSD == 1)
	{
		_SaveToSD();
		ctx.shouldSaveToSD = 0;
	}
}

//read the MPU data and calculate GForce and Space position
void ProcessValues()
{
	MPU_readSensor();
	_Calculate_GForce();
	_Calculate_Position();

	ctx.MeasureCount++;
}

void SetToChangeDisplayMode() 	{ ctx.shouldChangeMode = 1; }
void SetToUpdateDisplay() 		{ ctx.shouldUpdateDisplay = 1; }
void SetToCalibrate() 			{ ctx.shouldCalibrate = 1; }
void SetToSaveData()			{ ctx.shouldSaveToSD = 1; }


//-------------------------------------------------------------------
/*		Private		*/
//-------------------------------------------------------------------

void _SD_Init()
{
	//decrease the spi frequency
	//uint32_t currentPrescaler = hspi2.Init.BaudRatePrescaler;
	//hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	//HAL_SPI_Init(&hspi2);

	//try to mount the card a few times
	int attempts = 0;
	while(f_mount(&ctx.fs, "auto", 1)) {
		attempts++;
		HAL_Delay(15);
		if(attempts > 10)
		{
			ctx._DisableSD = 1;
			return;
		}
	}

	if(f_open(&ctx.fil, SD_FileName, FA_CREATE_ALWAYS | FA_WRITE | FA_READ)) {
		ctx._DisableSD = 1;
	}

	uint16_t pr = f_puts("XXX\tAccX\t|\tAccY\t|\tAccZ\t|\tGyrX\t|\tGyrY\t|\tGyrZ\n", &ctx.fil);
	f_puts("----------------------------------------------------------------------\n", &ctx.fil);

	if(f_sync(&ctx.fil)) {
		ctx._DisableSD = 1;
	}

	//hspi2.Init.BaudRatePrescaler = currentPrescaler;
	//HAL_SPI_Init(&hspi2);
}

void _SaveToSD()
{
	//ignore if could not initialize card
	if(ctx._DisableSD == 1){
		return;
	}

	char buffer[100];

	float AccX = MPU_getAccelX_mss();
	float AccY = MPU_getAccelY_mss();
	float AccZ = MPU_getAccelZ_mss();

	float GyrX = MPU_getGyroX_rads();
	float GyrY = MPU_getGyroY_rads();
	float GyrZ = MPU_getGyroZ_rads();

	snprintf(buffer, 100, "\t%+.2f\t|\t%+.2f\t|\t%+.2f\t|\t%+.2f\t|\t%+.2f\t|\t%+.2f\n",
			AccX, AccY, AccZ, GyrX, GyrY, GyrZ);



	//__NVIC_DisableIRQ(TIM4_IRQn);
	__disable_irq();

	uint16_t pr = f_puts(buffer, &ctx.fil);
	FRESULT sr = f_sync(&ctx.fil);

	__enable_irq();
	//__NVIC_EnableIRQ(TIM4_IRQn);
}


//enable reading data, await for the data to self calibrate and init the base reference values
void _CalibrateInit()
{
	//while(ctx.MeasureCount < Calibration_Init_SampleSize) {}
	HAL_Delay(1000*20);

	_PosToRadians();
	_Calibrate();
}

//update base GForce and angle values for reference
void _Calibrate()
{
	//__NVIC_DisableIRQ(TIM4_IRQn);
	//__disable_irq();

	//MPU_calibrateAccel();
	//MPU_calibrateGyro();
	//MPU_calibrateMag();
	//use the mean of the last few GForce measurments as the new base
	ctx.GForce_Base = ctx.GForce_Mean;

	//use the current space vector as the base one
	_Calculate_Angle();
	ctx.BaseX = ctx.PosX;
	ctx.BaseY = ctx.PosY;
	ctx.BaseZ = ctx.PosZ;
	//ctx.BaseVectorLength = ctx.BaseX * ctx.BaseX + ctx.BaseY * ctx.BaseY;

	//__NVIC_EnableIRQ(TIM4_IRQn);
	//__enable_irq();
}


//calculatethe neccesary values from the recent readings before updating display
void _PreUpdate()
{
	//__NVIC_DisableIRQ(TIM4_IRQn);
	//__disable_irq();

	_Calculate_Angle();

	//__NVIC_EnableIRQ(TIM4_IRQn);
	//__enable_irq();
}

//update the display in the respective mode
void _UpdateDisplay()
{
	if( ctx.shouldChangeMode == 1)
	{
		//cycle through the display modes
		switch(ctx.mode)
		{
			case Display_Default:
			ctx.mode = Display_Position;
			break;

			case Display_Position:
			ctx.mode = Display_Raw;
			break;

			case Display_Raw:
			ctx.mode = Display_Default;
			break;
		}

		//draw the static values for the chosen mode
		_Prepare_DrawDisplay(ctx.mode);

		ctx.shouldChangeMode = 0;
	}

	_DrawDisplay(ctx.mode);
}

void _PostUpdate()
{
	//Toogle Buzzer based on GForce
	if( (ctx.GForce_LocalMax/ctx.GForce_Base >= GForce_Threashold_1) || (ctx.Angle >= Angle_Threashold_1) )
		_Buzzer_Enable();
	else if( (ctx.GForce_LocalMax/ctx.GForce_Base < GForce_Threashold_1) && (ctx.Angle < Angle_Threashold_1) )
		_Buzzer_Disable();

	//ToogleBuzzer based on Angle
	//if(ctx.Angle >= Angle_Threashold_1)
	//	_Buzzer_Enable();
	//else if(ctx.Angle < Angle_Threashold_1)
	//	_Buzzer_Disable();

	//reset GForce local maximum
	ctx.GForce_LocalMax = ctx.GForce_Base;
	ctx.MeasureCount = 0;
}


void _TestDisplay()
{
	//Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, WHITE);
	//Paint_Clear(WHITE);

	st7789_SetRotation(0);
	st7789_FillScreen(RED);
	st7789_SetRotation(1);
	st7789_FillScreen(GREEN);
	st7789_SetRotation(2);
	st7789_FillScreen(BLUE);
	st7789_SetRotation(3);
	st7789_FillScreen(WHITE);


	st7789_SetRotation(2);
	st7789_DrawString(30, 10, "123", &Font24, YELLOW, RED);
	st7789_DrawString(30, 34, "ABC", &Font24, BLUE, CYAN);

	st7789_FillRectangle(30, 100, 150, 30, BLACK);
	st7789_FillRectangle(30, 130, 150, 30, RED);
	st7789_FillRectangle(30, 160, 150, 30, YELLOW);
}

//Draw static values that neve change in the given mode
void _Prepare_DrawDisplay(DisplayMode mode)
{
	switch(mode)
	{
		case Display_Default:
		{
			st7789_FillScreen(BLACK);

			st7789_DrawString(20, 50, "Force: ", &Font24, BLACK, WHITE);
			st7789_DrawString(20, 80, "Angle: ", &Font24, BLACK, WHITE);
			break;
		}

		case Display_Position:
		{
			st7789_FillScreen(BLACK);

			st7789_DrawString(20, 30, "X: ", &Font24, BLACK, RED);
			st7789_DrawString(20, 60, "Y: ", &Font24, BLACK, GREEN);
			st7789_DrawString(20, 90, "Z: ", &Font24, BLACK, BLUE);
			break;
		}

		case Display_Raw:
		{
			char Buff[50];
			st7789_FillScreen(BLACK);

			snprintf(Buff, 50, "%9s%9s%9s", "AccX", "AccY", "AccZ");
			st7789_DrawString(Display_Spacing, (Display_Spacing + DispRaw_FontH) * 0, Buff, DispRaw_Font, BLACK, RED);

			snprintf(Buff, 50, "%9s%9s%9s", "GyrX", "GyrY", "GyrZ");
			st7789_DrawString(Display_Spacing, (Display_Spacing + DispRaw_FontH) * 3, Buff, DispRaw_Font, BLACK, GREEN);

			snprintf(Buff, 50, "%9s%9s%9s", "MagX", "MagY", "MagZ");
			st7789_DrawString(Display_Spacing, (Display_Spacing + DispRaw_FontH) * 6, Buff, DispRaw_Font, BLACK, BLUE);

			st7789_DrawString(Display_Spacing, (Display_Spacing + DispRaw_FontH) * 9, "Temperature", DispRaw_Font, BLACK, YELLOW);
			break;
		}

		default: break;
	}
}

//update the variable elements of the display
void _DrawDisplay(DisplayMode mode)
{
	switch(mode)
	{
		case Display_Default:
		{
			char Buff[10];
			float GForce = ctx.GForce_LocalMax / ctx.GForce_Base;
			uint16_t color;

			if(GForce < GForce_Threashold_1)
				color = GREEN;
			else if(GForce < GForce_Threashold_2)
				color = YELLOW;
			else
				color = RED;
			snprintf(Buff, 10, "%5.2f", GForce);
			st7789_DrawString(20 + 17*7, 50, Buff, &Font24, BLACK, color);


			if(ctx.Angle < Angle_Threashold_1)
				color = GREEN;
			else if(ctx.Angle < Angle_Threashold_2)
				color = YELLOW;
			else
				color = RED;
			snprintf(Buff, 10, "%5.2f", ctx.Angle);
			st7789_DrawString(20 + 17*7, 80, Buff, &Font24, BLACK, color);
			break;
		}

		case Display_Position:
		{
			char Buff[10];

			snprintf(Buff, 10, "%7.2f", ctx.PosX);
			st7789_DrawString(20 + 17*3, 30, Buff, &Font24, BLACK, RED);

			snprintf(Buff, 10, "%7.2f", ctx.PosY);
			st7789_DrawString(20 + 17*3, 60, Buff, &Font24, BLACK, GREEN);

			snprintf(Buff, 10, "%7.2f", ctx.PosZ);
			st7789_DrawString(20 + 17*3, 90, Buff, &Font24, BLACK, BLUE);
			break;
		}

		case Display_Raw:
		{
			char Buff[50];

			const float AccX = MPU_getAccelX_mss();
			const float AccY = MPU_getAccelY_mss();
			const float AccZ = MPU_getAccelZ_mss();
			snprintf(Buff, 50, "%+9.2f%+9.2f%+9.2f", AccX, AccY, AccZ);
			st7789_DrawString(Display_Spacing, (Display_Spacing + DispRaw_FontH) * 1, Buff, DispRaw_Font, BLACK, RED);

			const float GyrX = MPU_getGyroX_rads();
			const float GyrY = MPU_getGyroY_rads();
			const float GyrZ = MPU_getGyroZ_rads();
			snprintf(Buff, 50, "%+9.2f%+9.2f%+9.2f", GyrX, GyrY, GyrZ);
			st7789_DrawString(Display_Spacing, (Display_Spacing + DispRaw_FontH) * 4, Buff, DispRaw_Font, BLACK, GREEN);

			const float MagX = MPU_getMagX_uT();
			const float MagY = MPU_getMagY_uT();
			const float MagZ = MPU_getMagZ_uT();
			snprintf(Buff, 50, "%+9.2f%+9.2f%+9.2f", MagX, MagY, MagZ);
			st7789_DrawString(Display_Spacing, (Display_Spacing + DispRaw_FontH) * 7, Buff, DispRaw_Font, BLACK, BLUE);

			const float Temp = MPU_getTemperature_C();
			snprintf(Buff, 50, "%11.2f", Temp);
			st7789_DrawString(Display_Spacing, (Display_Spacing + DispRaw_FontH) * 10, Buff, DispRaw_Font, BLACK, YELLOW);

			snprintf(Buff, 50, "%5d", ctx.MeasureCount);
			st7789_DrawString(Display_Spacing, (Display_Spacing + DispRaw_FontH) * 13, Buff, DispRaw_Font, BLACK, WHITE);
			break;
		}


		default: break;
	}
}


void _Buzzer_Enable()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	ctx.status = Buzzer_Enabled;
}

void _Buzzer_Disable()
{
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	ctx.status = Buzzer_Disabled;
}

void _Calculate_GForce()
{
	//get the total GForce_Mean value
	const int32_t ax = MPU_getAccelX_raw();
	const int32_t ay = MPU_getAccelY_raw();
	const int32_t az = MPU_getAccelZ_raw();
	const float g = sqrt( (float)(ax*ax + ay*ay + az*az) );

	//Update the Mean GForce_Mean value based on last few results
	ctx.GForce_Mean -= ctx.GForce_Buffor[ctx.GForce_Buffor_Index];
	ctx.GForce_Buffor[ctx.GForce_Buffor_Index] = g;
	ctx.GForce_Mean += ctx.GForce_Buffor[ctx.GForce_Buffor_Index];

	//update the index of cyclic buffor
	ctx.GForce_Buffor_Index++;
	if(ctx.GForce_Buffor_Index == GForce_BufforSize)
		ctx.GForce_Buffor_Index = 0;

	//update local maximum
	if(ctx.GForce_Mean > ctx.GForce_LocalMax)
		ctx.GForce_LocalMax = ctx.GForce_Mean;
}

void _Calculate_Position()
{
	//get latest read values from sensors
	const float gx = MPU_getGyroX_rads();
	const float gy = MPU_getGyroY_rads();
	const float gz = MPU_getGyroZ_rads();

	const float ax = MPU_getAccelX_mss();
	const float ay = MPU_getAccelY_mss();
	const float az = MPU_getAccelZ_mss();

	const float mx = MPU_getMagX_uT();
	const float my = MPU_getMagY_uT();
	const float mz = MPU_getMagZ_uT();

	//try doing the filter 5-10 times for the same data for best results
	//MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
	MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
	//MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
	//MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
}

void _PosToRadians()
{
	// roll (x-axis rotation)
	const float sinr_cosp = 2 * (_ahrs.q0 * _ahrs.q1 + _ahrs.q2 * _ahrs.q3);
	const float cosr_cosp = 1 - 2 * (_ahrs.q1 * _ahrs.q1 + _ahrs.q2 * _ahrs.q2);
	ctx.PosX = atan2(sinr_cosp, cosr_cosp) * RadToDeg;

	// pitch (y-axis rotation)
	float sinp = 2 * (_ahrs.q0 * _ahrs.q2 - _ahrs.q3 * _ahrs.q1);
	sinp = sinp > 1.0 ? 1.0 : sinp;
	sinp = sinp < -1.0 ? -1.0 : sinp;
	ctx.PosY = asin(sinp) * RadToDeg;

	// yaw (z-axis rotation)
	const float siny_cosp = 2 * (_ahrs.q0 * _ahrs.q3 + _ahrs.q1 * _ahrs.q2);
	const float cosy_cosp = 1 - 2 * (_ahrs.q2 * _ahrs.q2 + _ahrs.q3 * _ahrs.q3);
	ctx.PosZ = atan2(siny_cosp, cosy_cosp) * RadToDeg;
}

//convert Madgwick Quanternians to Eulers x, y, z coordinates and calculate angle between base vector
void _Calculate_Angle()
{
	_PosToRadians();

	//float pX = fabs(ctx.PosX);
	//pX = pX > 90.0f ? 180.0f - pX : pX;
	//float bX = fabs(ctx.BaseX);
	//bX = bX > 90.0f ? 180.0f - bX : bX;
	//float pY = fabs(ctx.PosY);
	//pY = pY > 90.0f ? 180.0f - pY : pY;
	//float bY = fabs(ctx.BaseY);
	//bY = bY > 90.0f ? 180.0f - bY : bY;

	float pX = ctx.PosX;
	float pY = ctx.PosY;
	float bX = ctx.BaseX;
	float bY = ctx.BaseY;

	float pitch = fabs(bX - pX);
	float roll = fabs(bY - pY);

	pitch = pitch > 180.0 ? 360.0 - pitch : pitch;
	roll = roll > 90.0 ? 180.0 - roll : roll;

	ctx.Angle = pitch > roll ? pitch : roll;
}







