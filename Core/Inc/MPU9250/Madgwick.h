#ifndef _MadgwickAHRS
#define _MadgwickAHRS

#include <math.h>

#define BetaFactor 			0.2f
#define BaseSampleRate 		500.0f
#define BasetwoKp			(2.0f * 0.5f)	// 2 * proportional gain
#define BasetwoKi			(2.0f * 0.0f)	// 2 * integral gain

typedef struct
{
	float q0, q1, q2, q3;
	float SampleFrequency;

	float Beta;

	float twoKp;
	float twoKi;
	float integralFBx, integralFBy, integralFBz;	// integral error terms scaled by Ki
} AHRS;

void AHRS_Init(void);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

float invSqrt(float x);

#endif
