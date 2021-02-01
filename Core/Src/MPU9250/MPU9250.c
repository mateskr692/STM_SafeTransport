#include "MPU9250/MPU9250.h"

	//volatile MPU_TypeDef MPUType;
volatile MPU_TypeDef MPUType;

void MPU_Init()
{
	MPUType.SPI_READ = 0x80;
	MPUType.SPI_LS_CLOCK = 1000000;
	MPUType.SPI_HS_CLOCK = 1500000;

	MPUType._tempScale = 333.87f;
	MPUType._tempOffset = 21.0f;

	MPUType._numSamples = 100;

	MPUType._axs = 1.0f;
	MPUType._ays = 1.0f;
	MPUType._azs = 1.0f;

	MPUType._maxCounts = 1000;
	MPUType._deltaThresh = 0.3f;
	MPUType._coeff = 8;

	MPUType._hxs = 1.0f;
	MPUType._hys = 1.0f;
	MPUType._hzs = 1.0f;

	//MPUType.tX[0] = 0; MPUType.tX[1] = 1; MPUType.tX[2] = 0;
	//MPUType.tY[0] = 1; MPUType.tY[1] = 0; MPUType.tY[2] = 0;
	//MPUType.tZ[0] = 0; MPUType.tZ[1] = 0; MPUType.tZ[2] = -1;

	MPUType.G = 9.807f;
	MPUType._d2r = 3.14159265359f/180.0f;
}


/* starts communication with the MPU-9250 */
int MPU_begin()
{

  // select clock source to gyro
  if(_MPU_writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -1;
  }
  // enable I2C master mode
  if(_MPU_writeRegister(USER_CTRL,I2C_MST_EN) < 0){
    return -2;
  }
  // set the I2C bus speed to 400 kHz
  if(_MPU_writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
    return -3;
  }
  // set AK8963 to Power Down
  _MPU_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
  // reset the MPU9250
  _MPU_writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  HAL_Delay(1);
  // reset the AK8963
  _MPU_writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
  // select clock source to gyro
  if(_MPU_writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -4;
  }
  // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
  if((_MPU_whoAmI() != 113)&&(_MPU_whoAmI() != 115)){
    return -5;
  }
  // enable accelerometer and gyro
  if(_MPU_writeRegister(PWR_MGMNT_2,SEN_ENABLE) < 0){
    return -6;
  }
  // setting accel range to 16G as default
  if(_MPU_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
    return -7;
  }
  MPUType._accelScale = MPUType.G * 16.0f/32767.5f; // setting the accel scale to 16G
  MPUType._accelRange = ACCEL_RANGE_16G;
  // setting the gyro range to 2000DPS as default
  if(_MPU_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
    return -8;
  }
  MPUType._gyroScale = 2000.0f/32767.5f * MPUType._d2r; // setting the gyro scale to 2000DPS
  MPUType._gyroRange = GYRO_RANGE_2000DPS;
  // setting bandwidth to 184Hz as default
  if(_MPU_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){
    return -9;
  }
  if(_MPU_writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
    return -10;
  }
  MPUType._bandwidth = DLPF_BANDWIDTH_184HZ;
  // setting the sample rate divider to 0 as default
  if(_MPU_writeRegister(SMPDIV,0x00) < 0){
    return -11;
  }
  MPUType._srd = 0;
  // enable I2C master mode
  if(_MPU_writeRegister(USER_CTRL,I2C_MST_EN) < 0){
  	return -12;
  }
	// set the I2C bus speed to 400 kHz
	if( _MPU_writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
		return -13;
	}
	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( _MPU_whoAmIAK8963() != 72 ){
    return -14;
	}
  /* get the magnetometer calibration */
  // set AK8963 to Power Down
  if(_MPU_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -15;
  }
  	  	  //HAL_Delay(100); // long wait between AK8963 mode changes
  // set AK8963 to FUSE ROM access
  if(_MPU_writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) < 0){
    return -16;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // read the AK8963 ASA registers and compute magnetometer scale factors
  _MPU_readAK8963Registers(AK8963_ASA, 3, MPUType._buffer);
  MPUType._magScaleX = ((((float)MPUType._buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  MPUType._magScaleY = ((((float)MPUType._buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  MPUType._magScaleZ = ((((float)MPUType._buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  // set AK8963 to Power Down
  if(_MPU_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -17;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // set AK8963 to 16 bit resolution, 100 Hz update rate
  if(_MPU_writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
    return -18;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // select clock source to gyro
  if(_MPU_writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -19;
  }
  // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  _MPU_readAK8963Registers(AK8963_HXL,7,MPUType._buffer);
  // estimate gyro bias
  if (MPU_calibrateGyro() < 0) {
    return -20;
  }
  // successful init, return 1
  return 1;
}

/* sets the accelerometer full scale range to values other than default */
int MPU_tAccelRange(MPU_AccelRange range) {
  // use low speed SPI for register setting
  switch(range) {
    case ACCEL_RANGE_2G: {
      // setting the accel range to 2G
      if(_MPU_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) < 0){
        return -1;
      }
      MPUType._accelScale = MPUType.G * 2.0f/32767.5f; // setting the accel scale to 2G
      break;
    }
    case ACCEL_RANGE_4G: {
      // setting the accel range to 4G
      if(_MPU_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) < 0){
        return -1;
      }
      MPUType._accelScale = MPUType.G * 4.0f/32767.5f; // setting the accel scale to 4G
      break;
    }
    case ACCEL_RANGE_8G: {
      // setting the accel range to 8G
      if(_MPU_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) < 0){
        return -1;
      }
      MPUType._accelScale = MPUType.G * 8.0f/32767.5f; // setting the accel scale to 8G
      break;
    }
    case ACCEL_RANGE_16G: {
      // setting the accel range to 16G
      if(_MPU_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
        return -1;
      }
      MPUType._accelScale = MPUType.G * 16.0f/32767.5f; // setting the accel scale to 16G
      break;
    }
  }
  MPUType._accelRange = range;
  return 1;
}

/* sets the gyro full scale range to values other than default */
int MPU_setGyroRange(MPU_GyroRange range) {
  // use low speed SPI for register setting
  //_useSPIHS = false;
  switch(range) {
    case GYRO_RANGE_250DPS: {
      // setting the gyro range to 250DPS
      if(_MPU_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) < 0){
        return -1;
      }
      MPUType._gyroScale = 250.0f/32767.5f * MPUType._d2r; // setting the gyro scale to 250DPS
      break;
    }
    case GYRO_RANGE_500DPS: {
      // setting the gyro range to 500DPS
      if(_MPU_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) < 0){
        return -1;
      }
      MPUType._gyroScale = 500.0f/32767.5f * MPUType._d2r; // setting the gyro scale to 500DPS
      break;
    }
    case GYRO_RANGE_1000DPS: {
      // setting the gyro range to 1000DPS
      if(_MPU_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) < 0){
        return -1;
      }
      MPUType._gyroScale = 1000.0f/32767.5f * MPUType._d2r; // setting the gyro scale to 1000DPS
      break;
    }
    case GYRO_RANGE_2000DPS: {
      // setting the gyro range to 2000DPS
      if(_MPU_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
        return -1;
      }
      MPUType._gyroScale = 2000.0f/32767.5f * MPUType._d2r; // setting the gyro scale to 2000DPS
      break;
    }
  }
  MPUType._gyroRange = range;
  return 1;
}

/* sets the DLPF bandwidth to values other than default */
int MPU_setDlpfBandwidth(MPU_DlpfBandwidth bandwidth) {
  // use low speed SPI for register setting
  //_useSPIHS = false;
  switch(bandwidth) {
    case DLPF_BANDWIDTH_184HZ: {
      if(_MPU_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ // setting accel bandwidth to 184Hz
        return -1;
      }
      if(_MPU_writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_92HZ: {
      if(_MPU_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_92) < 0){ // setting accel bandwidth to 92Hz
        return -1;
      }
      if(_MPU_writeRegister(CONFIG,GYRO_DLPF_92) < 0){ // setting gyro bandwidth to 92Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_41HZ: {
      if(_MPU_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_41) < 0){ // setting accel bandwidth to 41Hz
        return -1;
      }
      if(_MPU_writeRegister(CONFIG,GYRO_DLPF_41) < 0){ // setting gyro bandwidth to 41Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_20HZ: {
      if(_MPU_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_20) < 0){ // setting accel bandwidth to 20Hz
        return -1;
      }
      if(_MPU_writeRegister(CONFIG,GYRO_DLPF_20) < 0){ // setting gyro bandwidth to 20Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_10HZ: {
      if(_MPU_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10) < 0){ // setting accel bandwidth to 10Hz
        return -1;
      }
      if(_MPU_writeRegister(CONFIG,GYRO_DLPF_10) < 0){ // setting gyro bandwidth to 10Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_5HZ: {
      if(_MPU_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5) < 0){ // setting accel bandwidth to 5Hz
        return -1;
      }
      if(_MPU_writeRegister(CONFIG,GYRO_DLPF_5) < 0){ // setting gyro bandwidth to 5Hz
        return -2;
      }
      break;
    }
  }
  MPUType._bandwidth = bandwidth;
  return 1;
}

/* sets the sample rate divider to values other than default */
int MPU_setSrd(uint8_t srd) {
  // use low speed SPI for register setting
  //_useSPIHS = false;
  /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
  if(_MPU_writeRegister(SMPDIV,19) < 0){ // setting the sample rate divider
    return -1;
  }
  if(srd > 9){
    // set AK8963 to Power Down
    if(_MPU_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
      return -2;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // set AK8963 to 16 bit resolution, 8 Hz update rate
    if(_MPU_writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1) < 0){
      return -3;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    _MPU_readAK8963Registers(AK8963_HXL,7,MPUType._buffer);
  } else {
    // set AK8963 to Power Down
    if(_MPU_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
      return -2;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // set AK8963 to 16 bit resolution, 100 Hz update rate
    if(_MPU_writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
      return -3;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    _MPU_readAK8963Registers(AK8963_HXL,7,MPUType._buffer);
  }
  /* setting the sample rate divider */
  if(_MPU_writeRegister(SMPDIV,srd) < 0){ // setting the sample rate divider
    return -4;
  }
  MPUType._srd = srd;
  return 1;
}

/* enables the data ready interrupt */
int MPU_enableDataReadyInterrupt() {
  // use low speed SPI for register setting
  //_useSPIHS = false;
  /* setting the interrupt */
  if (_MPU_writeRegister(INT_PIN_CFG,INT_PULSE_50US) < 0){ // setup interrupt, 50 us pulse
    return -1;
  }
  if (_MPU_writeRegister(INT_ENABLE,INT_RAW_RDY_EN) < 0){ // set to data ready
    return -2;
  }
  return 1;
}

/* disables the data ready interrupt */
int MPU_disableDataReadyInterrupt() {
  // use low speed SPI for register setting
  //_useSPIHS = false;
  if(_MPU_writeRegister(INT_ENABLE,INT_DISABLE) < 0){ // disable interrupt
    return -1;
  }
  return 1;
}

/* configures and enables wake on motion, low power mode */
int MPU_enableWakeOnMotion(float womThresh_mg,MPU_LpAccelOdr odr) {
  // use low speed SPI for register setting
  //_useSPIHS = false;
  // set AK8963 to Power Down
	_MPU_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
  // reset the MPU9250
	_MPU_writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  HAL_Delay(1);
  if(_MPU_writeRegister(PWR_MGMNT_1,0x00) < 0){ // cycle 0, sleep 0, standby 0
    return -1;
  }
  if(_MPU_writeRegister(PWR_MGMNT_2,DIS_GYRO) < 0){ // disable gyro measurements
    return -2;
  }
  if(_MPU_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ // setting accel bandwidth to 184Hz
    return -3;
  }
  if(_MPU_writeRegister(INT_ENABLE,INT_WOM_EN) < 0){ // enabling interrupt to wake on motion
    return -4;
  }
  if(_MPU_writeRegister(MOT_DETECT_CTRL,(ACCEL_INTEL_EN | ACCEL_INTEL_MODE)) < 0){ // enabling accel hardware intelligence
    return -5;
  }
  MPUType._womThreshold = (womThresh_mg / 1020) * 255;
  if(_MPU_writeRegister(WOM_THR,MPUType._womThreshold) < 0){ // setting wake on motion threshold
    return -6;
  }
  if(_MPU_writeRegister(LP_ACCEL_ODR,(uint8_t)odr) < 0){ // set frequency of wakeup
    return -7;
  }
  if(_MPU_writeRegister(PWR_MGMNT_1,PWR_CYCLE) < 0){ // switch to accel low power mode
    return -8;
  }
  return 1;
}

/* reads the most current data from MPU9250 and stores in buffer */
int MPU_readSensor() {
  //_useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the MPU9250
  if (_MPU_readRegisters(ACCEL_OUT, 21, MPUType._buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  MPUType._axcounts = (((int16_t)MPUType._buffer[0]) << 8) | MPUType._buffer[1];
  MPUType._aycounts = (((int16_t)MPUType._buffer[2]) << 8) | MPUType._buffer[3];
  MPUType._azcounts = (((int16_t)MPUType._buffer[4]) << 8) | MPUType._buffer[5];
  MPUType._tcounts =  (((int16_t)MPUType._buffer[6]) << 8) | MPUType._buffer[7];
  MPUType._gxcounts = (((int16_t)MPUType._buffer[8]) << 8) | MPUType._buffer[9];
  MPUType._gycounts = (((int16_t)MPUType._buffer[10]) << 8) | MPUType._buffer[11];
  MPUType._gzcounts = (((int16_t)MPUType._buffer[12]) << 8) | MPUType._buffer[13];
  MPUType._hxcounts = (((int16_t)MPUType._buffer[15]) << 8) | MPUType._buffer[14];
  MPUType._hycounts = (((int16_t)MPUType._buffer[17]) << 8) | MPUType._buffer[16];
  MPUType._hzcounts = (((int16_t)MPUType._buffer[19]) << 8) | MPUType._buffer[18];

  // transform and convert to float values
  MPUType._ax = (( (float)(MPUType._aycounts) ) * MPUType._accelScale - MPUType._axb) * MPUType._axs;
  MPUType._ay = (( (float)(MPUType._axcounts) ) * MPUType._accelScale - MPUType._ayb) * MPUType._ays;
  MPUType._az = (( (float)(-MPUType._azcounts)) * MPUType._accelScale - MPUType._azb) * MPUType._azs;

  MPUType._gx = ( (float)(MPUType._gycounts) * MPUType._gyroScale) - MPUType._gxb;
  MPUType._gy = ( (float)(MPUType._gxcounts) * MPUType._gyroScale) - MPUType._gyb;
  MPUType._gz = ( (float)(-MPUType._gzcounts) * MPUType._gyroScale) - MPUType._gzb;

  MPUType._hx = (( (float)(MPUType._hxcounts) * MPUType._magScaleX) - MPUType._hxb) * MPUType._hxs;
  MPUType._hy = (( (float)(MPUType._hycounts) * MPUType._magScaleY) - MPUType._hyb) * MPUType._hys;
  MPUType._hz = (( (float)(MPUType._hzcounts) * MPUType._magScaleZ) - MPUType._hzb) * MPUType._hzs;

  MPUType._t = ((((float) MPUType._tcounts) - MPUType._tempOffset)/MPUType._tempScale) + MPUType._tempOffset;
  return 1;
}

int MPU_readSensor_noBias() {
  //_useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the MPU9250
  if (_MPU_readRegisters(ACCEL_OUT, 21, MPUType._buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  MPUType._axcounts = (((int16_t)MPUType._buffer[0]) << 8) | MPUType._buffer[1];
  MPUType._aycounts = (((int16_t)MPUType._buffer[2]) << 8) | MPUType._buffer[3];
  MPUType._azcounts = (((int16_t)MPUType._buffer[4]) << 8) | MPUType._buffer[5];
  MPUType._tcounts =  (((int16_t)MPUType._buffer[6]) << 8) | MPUType._buffer[7];
  MPUType._gxcounts = (((int16_t)MPUType._buffer[8]) << 8) | MPUType._buffer[9];
  MPUType._gycounts = (((int16_t)MPUType._buffer[10]) << 8) | MPUType._buffer[11];
  MPUType._gzcounts = (((int16_t)MPUType._buffer[12]) << 8) | MPUType._buffer[13];
  MPUType._hxcounts = (((int16_t)MPUType._buffer[15]) << 8) | MPUType._buffer[14];
  MPUType._hycounts = (((int16_t)MPUType._buffer[17]) << 8) | MPUType._buffer[16];
  MPUType._hzcounts = (((int16_t)MPUType._buffer[19]) << 8) | MPUType._buffer[18];

  // transform and convert to float values
  MPUType._ax = (float)(MPUType._aycounts)  * MPUType._accelScale;
  MPUType._ay = (float)(MPUType._axcounts)  * MPUType._accelScale;
  MPUType._az = (float)(-MPUType._azcounts) * MPUType._accelScale;

  MPUType._gx = (float)(MPUType._gycounts)  * MPUType._gyroScale;
  MPUType._gy = (float)(MPUType._gxcounts)  * MPUType._gyroScale;
  MPUType._gz = (float)(-MPUType._gzcounts) * MPUType._gyroScale;

  MPUType._hx = (float)(MPUType._hxcounts) * MPUType._magScaleX;
  MPUType._hy = (float)(MPUType._hycounts) * MPUType._magScaleY;
  MPUType._hz = (float)(MPUType._hzcounts) * MPUType._magScaleZ;

  MPUType._t = ((((float) MPUType._tcounts) - MPUType._tempOffset)/MPUType._tempScale) + MPUType._tempOffset;
  return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
float MPU_getAccelX_mss() {
  return MPUType._ax;
}

/* returns the accelerometer measurement in the y direction, m/s/s */
float MPU_getAccelY_mss() {
  return MPUType._ay;
}

/* returns the accelerometer measurement in the z direction, m/s/s */
float MPU_getAccelZ_mss() {
  return MPUType._az;
}

/* returns the gyroscope measurement in the x direction, rad/s */
float MPU_getGyroX_rads() {
  return MPUType._gx;
}

/* returns the gyroscope measurement in the y direction, rad/s */
float MPU_getGyroY_rads() {
  return MPUType._gy;
}

/* returns the gyroscope measurement in the z direction, rad/s */
float MPU_getGyroZ_rads() {
  return MPUType._gz;
}

/* returns the magnetometer measurement in the x direction, uT */
float MPU_getMagX_uT() {
  return MPUType._hx;
}

/* returns the magnetometer measurement in the y direction, uT */
float MPU_getMagY_uT() {
  return MPUType._hy;
}

/* returns the magnetometer measurement in the z direction, uT */
float MPU_getMagZ_uT() {
  return MPUType._hz;
}

/* returns the die temperature, C */
float MPU_getTemperature_C() {
  return MPUType._t;
}


int16_t MPU_getAccelX_raw() {
  return MPUType._axcounts;
}

/* returns the accelerometer measurement in the y direction, m/s/s */
int16_t MPU_getAccelY_raw() {
  return MPUType._aycounts;
}

/* returns the accelerometer measurement in the z direction, m/s/s */
int16_t MPU_getAccelZ_raw() {
  return MPUType._azcounts;
}

/* returns the gyroscope measurement in the x direction, rad/s */
int16_t MPU_getGyroX_raw() {
  return MPUType._gxcounts;
}

/* returns the gyroscope measurement in the y direction, rad/s */
int16_t MPU_getGyroY_raw() {
  return MPUType._gycounts;
}

/* returns the gyroscope measurement in the z direction, rad/s */
int16_t MPU_getGyroZ_raw() {
  return MPUType._gzcounts;
}

/* returns the magnetometer measurement in the x direction, uT */
int16_t MPU_getMagX_raw() {
  return MPUType._hxcounts;
}

/* returns the magnetometer measurement in the y direction, uT */
int16_t MPU_getMagY_raw() {
  return MPUType._hycounts;
}

/* returns the magnetometer measurement in the z direction, uT */
int16_t MPU_getMagZ_raw() {
  return MPUType._hzcounts;
}


/* estimates the gyro biases */
int MPU_calibrateGyro() {
  // set the range, bandwidth, and srd
  if (MPU_setGyroRange(GYRO_RANGE_250DPS) < 0) {
    return -1;
  }
  if (MPU_setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
    return -2;
  }
  if (MPU_setSrd(19) < 0) {
    return -3;
  }

  // take samples and find bias
  MPUType._gxbD = 0;
  MPUType._gybD = 0;
  MPUType._gzbD = 0;
  for (size_t i=0; i < MPUType._numSamples; i++) {
    MPU_readSensor();
    MPUType._gxbD += (MPU_getGyroX_rads() + MPUType._gxb)/((double)MPUType._numSamples);
    MPUType._gybD += (MPU_getGyroY_rads() + MPUType._gyb)/((double)MPUType._numSamples);
    MPUType._gzbD += (MPU_getGyroZ_rads() + MPUType._gzb)/((double)MPUType._numSamples);
    HAL_Delay(2);
  }
  MPUType._gxb = (float)MPUType._gxbD;
  MPUType._gyb = (float)MPUType._gybD;
  MPUType._gzb = (float)MPUType._gzbD;

  // set the range, bandwidth, and srd back to what they were
  if (MPU_setGyroRange(MPUType._gyroRange) < 0) {
    return -4;
  }
  if (MPU_setDlpfBandwidth(MPUType._bandwidth) < 0) {
    return -5;
  }
  if (MPU_setSrd(MPUType._srd) < 0) {
    return -6;
  }
  return 1;
}

/* returns the gyro bias in the X direction, rad/s */
float MPU_getGyroBiasX_rads() {
  return MPUType._gxb;
}

/* returns the gyro bias in the Y direction, rad/s */
float MPU_getGyroBiasY_rads() {
  return MPUType._gyb;
}

/* returns the gyro bias in the Z direction, rad/s */
float MPU_getGyroBiasZ_rads() {
  return MPUType._gzb;
}

/* sets the gyro bias in the X direction to bias, rad/s */
void MPU_setGyroBiasX_rads(float bias) {
	MPUType._gxb = bias;
}

/* sets the gyro bias in the Y direction to bias, rad/s */
void MPU_setGyroBiasY_rads(float bias) {
	MPUType._gyb = bias;
}

/* sets the gyro bias in the Z direction to bias, rad/s */
void MPU_setGyroBiasZ_rads(float bias) {
	MPUType._gzb = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int MPU_calibrateAccel() {
  // set the range, bandwidth, and srd
  if (MPU_setAccelRange(ACCEL_RANGE_2G) < 0) {
    return -1;
  }
  if (MPU_setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
    return -2;
  }
  if (MPU_setSrd(19) < 0) {
    return -3;
  }

  // take samples and find min / max
  MPUType._axbD = 0;
  MPUType._aybD = 0;
  MPUType._azbD = 0;
  for (size_t i=0; i < MPUType._numSamples; i++) {
    MPU_readSensor();
    MPUType._axbD += (MPU_getAccelX_mss()/MPUType._axs + MPUType._axb)/((double)MPUType._numSamples);
    MPUType._aybD += (MPU_getAccelY_mss()/MPUType._ays + MPUType._ayb)/((double)MPUType._numSamples);
    MPUType._azbD += (MPU_getAccelZ_mss()/MPUType._azs + MPUType._azb)/((double)MPUType._numSamples);
    HAL_Delay(2);
  }
  if (MPUType._axbD > 9.0f) {
	  MPUType._axmax = (float)MPUType._axbD;
  }
  if (MPUType._aybD > 9.0f) {
	  MPUType._aymax = (float)MPUType._aybD;
  }
  if (MPUType._azbD > 9.0f) {
	  MPUType. _azmax = (float)MPUType._azbD;
  }
  if (MPUType._axbD < -9.0f) {
	  MPUType._axmin = (float)MPUType._axbD;
  }
  if (MPUType._aybD < -9.0f) {
	  MPUType._aymin = (float)MPUType._aybD;
  }
  if (MPUType._azbD < -9.0f) {
	  MPUType._azmin = (float)MPUType._azbD;
  }

  // find bias and scale factor
  if ((fabs(MPUType._axmin) > 9.0f) && (fabs(MPUType._axmax) > 9.0f)) {
	  MPUType._axb = (MPUType._axmin + MPUType._axmax) / 2.0f;
	  MPUType._axs = MPUType.G/((fabs(MPUType._axmin) + fabs(MPUType._axmax)) / 2.0f);
  }
  if ((fabs(MPUType._aymin) > 9.0f) && (fabs(MPUType._aymax) > 9.0f)) {
	  MPUType._ayb = (MPUType._aymin + MPUType._aymax) / 2.0f;
	  MPUType._ays = MPUType.G/((fabs(MPUType._aymin) + fabs(MPUType._aymax)) / 2.0f);
  }
  if ((fabs(MPUType._azmin) > 9.0f) && (fabs(MPUType._azmax) > 9.0f)) {
	  MPUType._azb = (MPUType._azmin + MPUType._azmax) / 2.0f;
	  MPUType._azs = MPUType.G/((fabs(MPUType._azmin) + fabs(MPUType._azmax)) / 2.0f);
  }

  // set the range, bandwidth, and srd back to what they were
  if (MPU_setAccelRange(MPUType._accelRange) < 0) {
    return -4;
  }
  if (MPU_setDlpfBandwidth(MPUType._bandwidth) < 0) {
    return -5;
  }
  if (MPU_setSrd(MPUType._srd) < 0) {
    return -6;
  }
  return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float MPU_getAccelBiasX_mss() {
  return MPUType._axb;
}

/* returns the accelerometer scale factor in the X direction */
float MPU_getAccelScaleFactorX() {
  return MPUType._axs;
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float MPU_getAccelBiasY_mss() {
  return MPUType._ayb;
}

/* returns the accelerometer scale factor in the Y direction */
float MPU_getAccelScaleFactorY() {
  return MPUType._ays;
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float MPU_getAccelBiasZ_mss() {
  return MPUType._azb;
}

/* returns the accelerometer scale factor in the Z direction */
float MPU_getAccelScaleFactorZ() {
  return MPUType._azs;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void MPU_setAccelCalX(float bias,float scaleFactor) {
	MPUType._axb = bias;
	MPUType._axs = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void MPU_setAccelCalY(float bias,float scaleFactor) {
	MPUType._ayb = bias;
	MPUType._ays = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void MPU_setAccelCalZ(float bias,float scaleFactor) {
	MPUType._azb = bias;
	MPUType._azs = scaleFactor;
}

/* finds bias and scale factor calibration for the magnetometer,
the sensor should be rotated in a figure 8 motion until complete */
int MPU_calibrateMag() {
  // set the srd
  if (MPU_setSrd(19) < 0) {
    return -1;
  }

  // get a starting set of data
  MPU_readSensor();
  MPUType._hxmax = MPU_getMagX_uT();
  MPUType._hxmin = MPU_getMagX_uT();
  MPUType._hymax = MPU_getMagY_uT();
  MPUType._hymin = MPU_getMagY_uT();
  MPUType._hzmax = MPU_getMagZ_uT();
  MPUType._hzmin = MPU_getMagZ_uT();

  // collect data to find max / min in each channel
  MPUType._counter = 0;
  while (MPUType._counter < MPUType._maxCounts) {
	  MPUType._delta = 0.0f;
	  MPUType._framedelta = 0.0f;
	  MPU_readSensor();
	  MPUType._hxfilt = (MPUType._hxfilt*((float)MPUType._coeff-1)+(MPU_getMagX_uT()/MPUType._hxs+MPUType._hxb))/((float)MPUType._coeff);
	  MPUType._hyfilt = (MPUType._hyfilt*((float)MPUType._coeff-1)+(MPU_getMagY_uT()/MPUType._hys+MPUType._hyb))/((float)MPUType._coeff);
	  MPUType._hzfilt = (MPUType._hzfilt*((float)MPUType._coeff-1)+(MPU_getMagZ_uT()/MPUType._hzs+MPUType._hzb))/((float)MPUType._coeff);
    if (MPUType._hxfilt > MPUType._hxmax) {
    	MPUType._delta = MPUType._hxfilt - MPUType._hxmax;
    	MPUType._hxmax = MPUType._hxfilt;
    }
    if (MPUType._delta > MPUType._framedelta) {
    	MPUType._framedelta = MPUType._delta;
    }
    if (MPUType._hyfilt > MPUType._hymax) {
    	MPUType._delta = MPUType._hyfilt - MPUType._hymax;
    	MPUType._hymax = MPUType._hyfilt;
    }
    if (MPUType._delta > MPUType._framedelta) {
    	MPUType._framedelta = MPUType._delta;
    }
    if (MPUType._hzfilt > MPUType._hzmax) {
    	MPUType._delta = MPUType._hzfilt - MPUType._hzmax;
    	MPUType._hzmax = MPUType._hzfilt;
    }
    if (MPUType._delta > MPUType._framedelta) {
    	MPUType._framedelta = MPUType._delta;
    }
    if (MPUType._hxfilt < MPUType._hxmin) {
    	MPUType._delta = fabs(MPUType._hxfilt - MPUType._hxmin);
    	MPUType._hxmin = MPUType._hxfilt;
    }
    if (MPUType._delta > MPUType._framedelta) {
    	MPUType._framedelta = MPUType._delta;
    }
    if (MPUType._hyfilt < MPUType._hymin) {
    	MPUType._delta = fabs(MPUType._hyfilt - MPUType._hymin);
    	MPUType._hymin = MPUType._hyfilt;
    }
    if (MPUType._delta > MPUType._framedelta) {
    	MPUType._framedelta = MPUType._delta;
    }
    if (MPUType._hzfilt < MPUType._hzmin) {
    	MPUType._delta = fabs(MPUType._hzfilt - MPUType._hzmin);
    	MPUType._hzmin = MPUType._hzfilt;
    }
    if (MPUType._delta > MPUType._framedelta) {
    	MPUType._framedelta = MPUType._delta;
    }
    if (MPUType._framedelta > MPUType._deltaThresh) {
    	MPUType._counter = 0;
    } else {
    	MPUType._counter++;
    }
    HAL_Delay(2);
  }

  // find the magnetometer bias
  MPUType._hxb = (MPUType._hxmax + MPUType._hxmin) / 2.0f;
  MPUType._hyb = (MPUType._hymax + MPUType._hymin) / 2.0f;
  MPUType._hzb = (MPUType._hzmax + MPUType._hzmin) / 2.0f;

  // find the magnetometer scale factor
  MPUType._hxs = (MPUType._hxmax - MPUType._hxmin) / 2.0f;
  MPUType._hys = (MPUType._hymax - MPUType._hymin) / 2.0f;
  MPUType._hzs = (MPUType._hzmax - MPUType._hzmin) / 2.0f;
  MPUType._avgs = (MPUType._hxs + MPUType._hys + MPUType._hzs) / 3.0f;
  MPUType._hxs = MPUType._avgs/MPUType._hxs;
  MPUType._hys = MPUType._avgs/MPUType._hys;
  MPUType._hzs = MPUType._avgs/MPUType._hzs;

  // set the srd back to what it was
  if (MPU_setSrd(MPUType._srd) < 0) {
    return -2;
  }
  return 1;
}

/* returns the magnetometer bias in the X direction, uT */
float MPU_getMagBiasX_uT() {
  return MPUType._hxb;
}

/* returns the magnetometer scale factor in the X direction */
float MPU_getMagScaleFactorX() {
  return MPUType._hxs;
}

/* returns the magnetometer bias in the Y direction, uT */
float MPU_getMagBiasY_uT() {
  return MPUType._hyb;
}

/* returns the magnetometer scale factor in the Y direction */
float MPU_getMagScaleFactorY() {
  return MPUType._hys;
}

/* returns the magnetometer bias in the Z direction, uT */
float MPU_getMagBiasZ_uT() {
  return MPUType._hzb;
}

/* returns the magnetometer scale factor in the Z direction */
float MPU_getMagScaleFactorZ() {
  return MPUType._hzs;
}

/* sets the magnetometer bias (uT) and scale factor in the X direction */
void MPU_setMagCalX(float bias,float scaleFactor) {
	MPUType._hxb = bias;
	MPUType._hxs = scaleFactor;
}

/* sets the magnetometer bias (uT) and scale factor in the Y direction */
void MPU_setMagCalY(float bias,float scaleFactor) {
	MPUType._hyb = bias;
	MPUType._hys = scaleFactor;
}

/* sets the magnetometer bias (uT) and scale factor in the Z direction */
void MPU_setMagCalZ(float bias,float scaleFactor) {
	MPUType._hzb = bias;
	MPUType._hzs = scaleFactor;
}





//Private
/* writes a byte to MPU9250 register given a register address and data */
int _MPU_writeRegister(uint8_t subAddress, uint8_t data)
{
	/* write data to device */
    //_spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
	HAL_GPIO_WritePin(MPU_CS, GPIO_PIN_RESET); 	// select the MPU9250 chip
    HAL_SPI_Transmit(MPU_SPI, &subAddress, 1, 10);
    HAL_SPI_Transmit(MPU_SPI, &data, 1, 10);
    HAL_GPIO_WritePin(MPU_CS, GPIO_PIN_SET);  		// deselect the MPU9250 chip
    //_spi->endTransaction(); // end the transaction

    HAL_Delay(10);

    /* read back the register */
    //_MPU_readRegisters(subAddress, 1, MPUType._buffer);

    /* check the read back register against the written register */
	//if(MPUType._buffer[0] == data) {
	//	return 1;
	//}
	//else{
	//	return -1;
	//}
    return 1;
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
int _MPU_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){

    // begin the transaction
	HAL_GPIO_WritePin(MPU_CS, GPIO_PIN_RESET); 	// select the MPU9250 chip
	uint8_t data = subAddress | MPUType.SPI_READ;
	HAL_SPI_Transmit(MPU_SPI, &data, 1, 10);
    HAL_SPI_Receive(MPU_SPI, dest, count, 10); // read the data
    HAL_GPIO_WritePin(MPU_CS, GPIO_PIN_SET);
    //_spi->endTransaction(); // end the transaction
    return 1;

}


/* writes a register to the AK8963 given a register address and data */
int _MPU_writeAK8963Register(uint8_t subAddress, uint8_t data){
  // set slave 0 to the AK8963 and set for write
	if (_MPU_writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address
	if (_MPU_writeRegister(I2C_SLV0_REG,subAddress) < 0) {
    return -2;
  }
  // store the data for write
	if (_MPU_writeRegister(I2C_SLV0_DO,data) < 0) {
    return -3;
  }
  // enable I2C and send 1 byte
	if (_MPU_writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1) < 0) {
    return -4;
  }
	// read the register and confirm
	if (_MPU_readAK8963Registers(subAddress,1,MPUType._buffer) < 0) {
    return -5;
  }
	if(MPUType._buffer[0] == data) {
  	return 1;
  } else{
  	return -6;
  }
}

/* reads registers from the AK8963 */
int _MPU_readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
  // set slave 0 to the AK8963 and set for read
	if (_MPU_writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address
	if (_MPU_writeRegister(I2C_SLV0_REG,subAddress) < 0) {
    return -2;
  }
  // enable I2C and request the bytes
	if (_MPU_writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count) < 0) {
    return -3;
  }
	HAL_Delay(1); // takes some time for these registers to fill
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
	MPUType._status = _MPU_readRegisters(EXT_SENS_DATA_00,count,dest);
  return MPUType._status;
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
int _MPU_whoAmI(){
  // read the WHO AM I register
  if (_MPU_readRegisters(WHO_AM_I,1,MPUType._buffer) < 0) {
    return -1;
  }
  // return the register value
  return MPUType._buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
int _MPU_whoAmIAK8963(){
  // read the WHO AM I register
  if (_MPU_readAK8963Registers(AK8963_WHO_AM_I,1,MPUType._buffer) < 0) {
    return -1;
  }
  // return the register value
  return MPUType._buffer[0];
}
