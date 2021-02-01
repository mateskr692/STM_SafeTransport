/*
MPU9250.h
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MPU9250_h
#define MPU9250_h

#include "spi.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include <math.h>

#define MPU_SPI &hspi2
#define MPU_CS CS_MPU_GPIO_Port,CS_MPU_Pin

typedef enum
{
  GYRO_RANGE_250DPS,
  GYRO_RANGE_500DPS,
  GYRO_RANGE_1000DPS,
  GYRO_RANGE_2000DPS
} MPU_GyroRange;

typedef enum
{
  ACCEL_RANGE_2G,
  ACCEL_RANGE_4G,
  ACCEL_RANGE_8G,
  ACCEL_RANGE_16G
} MPU_AccelRange;

typedef enum
{
  DLPF_BANDWIDTH_184HZ,
  DLPF_BANDWIDTH_92HZ,
  DLPF_BANDWIDTH_41HZ,
  DLPF_BANDWIDTH_20HZ,
  DLPF_BANDWIDTH_10HZ,
  DLPF_BANDWIDTH_5HZ
} MPU_DlpfBandwidth;

typedef enum
{
  LP_ACCEL_ODR_0_24HZ = 0,
  LP_ACCEL_ODR_0_49HZ = 1,
  LP_ACCEL_ODR_0_98HZ = 2,
  LP_ACCEL_ODR_1_95HZ = 3,
  LP_ACCEL_ODR_3_91HZ = 4,
  LP_ACCEL_ODR_7_81HZ = 5,
  LP_ACCEL_ODR_15_63HZ = 6,
  LP_ACCEL_ODR_31_25HZ = 7,
  LP_ACCEL_ODR_62_50HZ = 8,
  LP_ACCEL_ODR_125HZ = 9,
  LP_ACCEL_ODR_250HZ = 10,
  LP_ACCEL_ODR_500HZ = 11
} MPU_LpAccelOdr;

typedef struct
{
	uint8_t SPI_READ;
	uint32_t SPI_LS_CLOCK;  // 1 MHz
	uint32_t SPI_HS_CLOCK; // 15 MHz


	// track success of interacting with sensor
	int _status;
	// buffer for reading from sensor
	uint8_t _buffer[21];
	// data counts
	int16_t _axcounts,_aycounts,_azcounts;
	int16_t _gxcounts,_gycounts,_gzcounts;
	int16_t _hxcounts,_hycounts,_hzcounts;
	int16_t _tcounts;
	// data buffer
	float _ax, _ay, _az;
	float _gx, _gy, _gz;
	float _hx, _hy, _hz;
	float _t;
	// wake on motion
	uint8_t _womThreshold;
	// scale factors
	float _accelScale;
	float _gyroScale;
	float _magScaleX, _magScaleY, _magScaleZ;
	float _tempScale;
	float _tempOffset;
	// configuration
	MPU_AccelRange _accelRange;
	MPU_GyroRange _gyroRange;
	MPU_DlpfBandwidth _bandwidth;
	uint8_t _srd;
	// gyro bias estimation
	size_t _numSamples;
	double _gxbD, _gybD, _gzbD;
	float _gxb, _gyb, _gzb;
	// accel bias and scale factor estimation
	double _axbD, _aybD, _azbD;
	float _axmax, _aymax, _azmax;
	float _axmin, _aymin, _azmin;
	float _axb, _ayb, _azb;
	float _axs;
	float _ays;
	float _azs;
	// magnetometer bias and scale factor estimation
	uint16_t _maxCounts;
	float _deltaThresh;
	uint8_t _coeff;
	uint16_t _counter;
	float _framedelta, _delta;
	float _hxfilt, _hyfilt, _hzfilt;
	float _hxmax, _hymax, _hzmax;
	float _hxmin, _hymin, _hzmin;
	float _hxb, _hyb, _hzb;
	float _hxs;
	float _hys;
	float _hzs;
	float _avgs;


	// transformation matrix
	/* transform the accel and gyro axes to match the magnetometer axes */
	//int16_t tX[3];
	//int16_t tY[3];
	//int16_t tZ[3];
	// constants
	float G;
	float _d2r;
}MPU_TypeDef;

void MPU_Init();
int MPU_begin();
int MPU_setAccelRange(MPU_AccelRange range);
int MPU_setGyroRange(MPU_GyroRange range);
int MPU_setDlpfBandwidth(MPU_DlpfBandwidth bandwidth);
int MPU_setSrd(uint8_t srd);
int MPU_enableDataReadyInterrupt();
int MPU_disableDataReadyInterrupt();
int MPU_enableWakeOnMotion(float womThresh_mg,MPU_LpAccelOdr odr);
int MPU_readSensor();
float MPU_getAccelX_mss();
float MPU_getAccelY_mss();
float MPU_getAccelZ_mss();
float MPU_getGyroX_rads();
float MPU_getGyroY_rads();
float MPU_getGyroZ_rads();
float MPU_getMagX_uT();
float MPU_getMagY_uT();
float MPU_getMagZ_uT();
float MPU_getTemperature_C();

int MPU_calibrateGyro();
float MPU_getGyroBiasX_rads();
float MPU_getGyroBiasY_rads();
float MPU_getGyroBiasZ_rads();
void MPU_setGyroBiasX_rads(float bias);
void MPU_setGyroBiasY_rads(float bias);
void MPU_setGyroBiasZ_rads(float bias);
int MPU_calibrateAccel();
float MPU_getAccelBiasX_mss();
float MPU_getAccelScaleFactorX();
float MPU_getAccelBiasY_mss();
float MPU_getAccelScaleFactorY();
float MPU_getAccelBiasZ_mss();
float MPU_getAccelScaleFactorZ();
void MPU_setAccelCalX(float bias,float scaleFactor);
void MPU_setAccelCalY(float bias,float scaleFactor);
void MPU_setAccelCalZ(float bias,float scaleFactor);
int MPU_calibrateMag();
float MPU_getMagBiasX_uT();
float MPU_getMagScaleFactorX();
float MPU_getMagBiasY_uT();
float MPU_getMagScaleFactorY();
float MPU_getMagBiasZ_uT();
float MPU_getMagScaleFactorZ();
void MPU_setMagCalX(float bias,float scaleFactor);
void MPU_setMagCalY(float bias,float scaleFactor);
void MPU_setMagCalZ(float bias,float scaleFactor);

int MPU_readSensor_noBias();
int16_t MPU_getAccelX_raw();
int16_t MPU_getAccelY_raw();
int16_t MPU_getAccelZ_raw();
int16_t MPU_getGyroX_raw();
int16_t MPU_getGyroY_raw();
int16_t MPU_getGyroZ_raw();
int16_t MPU_getMagX_raw();
int16_t MPU_getMagY_raw();
int16_t MPU_getMagZ_raw();


// MPU9250 registers
#define ACCEL_OUT 0x3B
#define GYRO_OUT  0x43
#define TEMP_OUT  0x41
#define EXT_SENS_DATA_00  0x49
#define ACCEL_CONFIG  0x1C
#define ACCEL_FS_SEL_2G  0x00
#define ACCEL_FS_SEL_4G  0x08
#define ACCEL_FS_SEL_8G  0x10
#define ACCEL_FS_SEL_16G  0x18
#define GYRO_CONFIG  0x1B
#define GYRO_FS_SEL_250DPS  0x00
#define GYRO_FS_SEL_500DPS  0x08
#define GYRO_FS_SEL_1000DPS  0x10
#define GYRO_FS_SEL_2000DPS  0x18
#define ACCEL_CONFIG2  0x1D
#define ACCEL_DLPF_184  0x01
#define ACCEL_DLPF_92  0x02
#define ACCEL_DLPF_41  0x03
#define ACCEL_DLPF_20  0x04
#define ACCEL_DLPF_10  0x05
#define ACCEL_DLPF_5  0x06
#define CONFIG  0x1A
#define GYRO_DLPF_184  0x01
#define GYRO_DLPF_92  0x02
#define GYRO_DLPF_41  0x03
#define GYRO_DLPF_20  0x04
#define GYRO_DLPF_10  0x05
#define GYRO_DLPF_5  0x06
#define SMPDIV  0x19
#define INT_PIN_CFG  0x37
#define INT_ENABLE  0x38
#define INT_DISABLE  0x00
#define INT_PULSE_50US  0x00
#define INT_WOM_EN  0x40
#define INT_RAW_RDY_EN  0x01
#define PWR_MGMNT_1  0x6B
#define PWR_CYCLE  0x20
#define PWR_RESET  0x80
#define CLOCK_SEL_PLL  0x01
#define PWR_MGMNT_2  0x6C
#define SEN_ENABLE  0x00
#define DIS_GYRO  0x07
#define USER_CTRL  0x6A
#define I2C_MST_EN  0x20
#define I2C_MST_CLK  0x0D
#define I2C_MST_CTRL  0x24
#define I2C_SLV0_ADDR  0x25
#define I2C_SLV0_REG  0x26
#define I2C_SLV0_DO  0x63
#define I2C_SLV0_CTRL  0x27
#define I2C_SLV0_EN  0x80
#define I2C_READ_FLAG  0x80
#define MOT_DETECT_CTRL  0x69
#define ACCEL_INTEL_EN  0x80
#define ACCEL_INTEL_MODE  0x40
#define LP_ACCEL_ODR  0x1E
#define WOM_THR  0x1F
#define WHO_AM_I  0x75
#define FIFO_EN  0x23
#define FIFO_TEMP  0x80
#define FIFO_GYRO  0x70
#define FIFO_ACCEL  0x08
#define FIFO_MAG  0x01
#define FIFO_COUNT  0x72
#define FIFO_READ  0x74
// AK8963 registers
#define AK8963_I2C_ADDR  0x0C
#define AK8963_HXL  0x03
#define AK8963_CNTL1  0x0A
#define AK8963_PWR_DOWN  0x00
#define AK8963_CNT_MEAS1  0x12
#define AK8963_CNT_MEAS2  0x16
#define AK8963_FUSE_ROM  0x0F
#define AK8963_CNTL2  0x0B
#define AK8963_RESET  0x01
#define AK8963_ASA  0x10
#define AK8963_WHO_AM_I  0x00

// private functions
int _MPU_writeRegister(uint8_t subAddress, uint8_t data);
int _MPU_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
int _MPU_writeAK8963Register(uint8_t subAddress, uint8_t data);
int _MPU_readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
int _MPU_whoAmI();
int _MPU_whoAmIAK8963();

#endif
