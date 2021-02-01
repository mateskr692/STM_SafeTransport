#include "MPU9250/Madgwick.h"

//volatile MadgwickData _AHRS;
volatile AHRS _ahrs;

void AHRS_Init()
{
	_ahrs.q0 = 1.0f;
	_ahrs.q1 = 0.0f;
	_ahrs.q2 = 0.0f;
	_ahrs.q3 = 0.0f;
	_ahrs.SampleFrequency = BaseSampleRate;

	_ahrs.Beta = BetaFactor;
	_ahrs.twoKp = BasetwoKp;
	_ahrs.twoKi = BasetwoKi;

	_ahrs.integralFBx = 0.0f;
	_ahrs.integralFBy = 0.0f;
	_ahrs.integralFBz = 0.0f;
}

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = _ahrs.q0 * _ahrs.q0;
        q0q1 = _ahrs.q0 * _ahrs.q1;
        q0q2 = _ahrs.q0 * _ahrs.q2;
        q0q3 = _ahrs.q0 * _ahrs.q3;
        q1q1 = _ahrs.q1 * _ahrs.q1;
        q1q2 = _ahrs.q1 * _ahrs.q2;
        q1q3 = _ahrs.q1 * _ahrs.q3;
        q2q2 = _ahrs.q2 * _ahrs.q2;
        q2q3 = _ahrs.q2 * _ahrs.q3;
        q3q3 = _ahrs.q3 * _ahrs.q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(_ahrs.twoKi > 0.0f) {
			_ahrs.integralFBx += _ahrs.twoKi * halfex * (1.0f / _ahrs.SampleFrequency);	// integral error scaled by Ki
			_ahrs.integralFBy += _ahrs.twoKi * halfey * (1.0f / _ahrs.SampleFrequency);
			_ahrs.integralFBz += _ahrs.twoKi * halfez * (1.0f / _ahrs.SampleFrequency);
			gx += _ahrs.integralFBx;	// apply integral feedback
			gy += _ahrs.integralFBy;
			gz += _ahrs.integralFBz;
		}
		else {
			_ahrs.integralFBx = 0.0f;	// prevent integral windup
			_ahrs.integralFBy = 0.0f;
			_ahrs.integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += _ahrs.twoKp * halfex;
		gy += _ahrs.twoKp * halfey;
		gz += _ahrs.twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / _ahrs.SampleFrequency));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / _ahrs.SampleFrequency));
	gz *= (0.5f * (1.0f / _ahrs.SampleFrequency));
	qa = _ahrs.q0;
	qb = _ahrs.q1;
	qc = _ahrs.q2;
	_ahrs.q0 += (-qb * gx - qc * gy - _ahrs.q3 * gz);
	_ahrs.q1 += (qa * gx + qc * gz - _ahrs.q3 * gy);
	_ahrs.q2 += (qa * gy - qb * gz + _ahrs.q3 * gx);
	_ahrs.q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(_ahrs.q0 * _ahrs.q0 + _ahrs.q1 * _ahrs.q1 + _ahrs.q2 * _ahrs.q2 + _ahrs.q3 *_ahrs. q3);
	_ahrs.q0 *= recipNorm;
	_ahrs.q1 *= recipNorm;
	_ahrs.q2 *= recipNorm;
	_ahrs.q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = _ahrs.q1 * _ahrs.q3 - _ahrs.q0 * _ahrs.q2;
		halfvy = _ahrs.q0 * _ahrs.q1 + _ahrs.q2 * _ahrs.q3;
		halfvz = _ahrs.q0 * _ahrs.q0 - 0.5f + _ahrs.q3 * _ahrs.q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(_ahrs.twoKi > 0.0f) {
			_ahrs.integralFBx += _ahrs.twoKi * halfex * (1.0f / _ahrs.SampleFrequency);	// integral error scaled by Ki
			_ahrs.integralFBy += _ahrs.twoKi * halfey * (1.0f / _ahrs.SampleFrequency);
			_ahrs.integralFBz += _ahrs.twoKi * halfez * (1.0f / _ahrs.SampleFrequency);
			gx += _ahrs.integralFBx;	// apply integral feedback
			gy += _ahrs.integralFBy;
			gz += _ahrs.integralFBz;
		}
		else {
			_ahrs.integralFBx = 0.0f;	// prevent integral windup
			_ahrs.integralFBy = 0.0f;
			_ahrs.integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += _ahrs.twoKp * halfex;
		gy += _ahrs.twoKp * halfey;
		gz += _ahrs.twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / _ahrs.SampleFrequency));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / _ahrs.SampleFrequency));
	gz *= (0.5f * (1.0f / _ahrs.SampleFrequency));
	qa = _ahrs.q0;
	qb = _ahrs.q1;
	qc = _ahrs.q2;
	_ahrs.q0 += (-qb * gx - qc * gy - _ahrs.q3 * gz);
	_ahrs.q1 += (qa * gx + qc * gz - _ahrs.q3 * gy);
	_ahrs.q2 += (qa * gy - qb * gz + _ahrs.q3 * gx);
	_ahrs.q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(_ahrs.q0 * _ahrs.q0 + _ahrs.q1 * _ahrs.q1 + _ahrs.q2 * _ahrs.q2 + _ahrs.q3 * _ahrs.q3);
	_ahrs.q0 *= recipNorm;
	_ahrs.q1 *= recipNorm;
	_ahrs.q2 *= recipNorm;
	_ahrs.q3 *= recipNorm;
}

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-_ahrs.q1 * gx - _ahrs.q2 * gy - _ahrs.q3 * gz);
	qDot2 = 0.5f * (_ahrs.q0 * gx + _ahrs.q2 * gz - _ahrs.q3 * gy);
	qDot3 = 0.5f * (_ahrs.q0 * gy - _ahrs.q1 * gz + _ahrs.q3 * gx);
	qDot4 = 0.5f * (_ahrs.q0 * gz + _ahrs.q1 * gy - _ahrs.q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * _ahrs.q0 * mx;
		_2q0my = 2.0f * _ahrs.q0 * my;
		_2q0mz = 2.0f * _ahrs.q0 * mz;
		_2q1mx = 2.0f * _ahrs.q1 * mx;
		_2q0 = 2.0f * _ahrs.q0;
		_2q1 = 2.0f * _ahrs.q1;
		_2q2 = 2.0f * _ahrs.q2;
		_2q3 = 2.0f * _ahrs.q3;
		_2q0q2 = 2.0f * _ahrs.q0 * _ahrs.q2;
		_2q2q3 = 2.0f * _ahrs.q2 * _ahrs.q3;
		q0q0 = _ahrs.q0 * _ahrs.q0;
		q0q1 = _ahrs.q0 * _ahrs.q1;
		q0q2 = _ahrs.q0 * _ahrs.q2;
		q0q3 = _ahrs.q0 * _ahrs.q3;
		q1q1 = _ahrs.q1 * _ahrs.q1;
		q1q2 = _ahrs.q1 * _ahrs.q2;
		q1q3 = _ahrs.q1 * _ahrs.q3;
		q2q2 = _ahrs.q2 * _ahrs.q2;
		q2q3 = _ahrs.q2 * _ahrs.q3;
		q3q3 = _ahrs.q3 * _ahrs.q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * _ahrs.q3 + _2q0mz * _ahrs.q2 + mx * q1q1 + _2q1 * my * _ahrs.q2 + _2q1 * mz * _ahrs.q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * _ahrs.q3 + my * q0q0 - _2q0mz * _ahrs.q1 + _2q1mx * _ahrs.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * _ahrs.q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * _ahrs.q2 + _2q0my * _ahrs.q1 + mz * q0q0 + _2q1mx * _ahrs.q3 - mz * q1q1 + _2q2 * my * _ahrs.q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * _ahrs.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * _ahrs.q3 + _2bz * _ahrs.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * _ahrs.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * _ahrs.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * _ahrs.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * _ahrs.q2 + _2bz * _ahrs.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * _ahrs.q3 - _4bz * _ahrs.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * _ahrs.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * _ahrs.q2 - _2bz * _ahrs.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * _ahrs.q1 + _2bz * _ahrs.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * _ahrs.q0 - _4bz * _ahrs.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * _ahrs.q3 + _2bz * _ahrs.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * _ahrs.q0 + _2bz * _ahrs.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * _ahrs.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= _ahrs.Beta * s0;
		qDot2 -= _ahrs.Beta * s1;
		qDot3 -= _ahrs.Beta * s2;
		qDot4 -= _ahrs.Beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	_ahrs.q0 += qDot1 * (1.0f / _ahrs.SampleFrequency);
	_ahrs.q1 += qDot2 * (1.0f / _ahrs.SampleFrequency);
	_ahrs.q2 += qDot3 * (1.0f / _ahrs.SampleFrequency);
	_ahrs.q3 += qDot4 * (1.0f / _ahrs.SampleFrequency);

	// Normalise quaternion
	recipNorm = invSqrt(_ahrs.q0 * _ahrs.q0 + _ahrs.q1 * _ahrs.q1 + _ahrs.q2 * _ahrs.q2 + _ahrs.q3 * _ahrs.q3);
	_ahrs.q0 *= recipNorm;
	_ahrs.q1 *= recipNorm;
	_ahrs.q2 *= recipNorm;
	_ahrs.q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-_ahrs.q1 * gx - _ahrs.q2 * gy - _ahrs.q3 * gz);
	qDot2 = 0.5f * (_ahrs.q0 * gx + _ahrs.q2 * gz - _ahrs.q3 * gy);
	qDot3 = 0.5f * (_ahrs.q0 * gy - _ahrs.q1 * gz + _ahrs.q3 * gx);
	qDot4 = 0.5f * (_ahrs.q0 * gz + _ahrs.q1 * gy - _ahrs.q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * _ahrs.q0;
		_2q1 = 2.0f * _ahrs.q1;
		_2q2 = 2.0f * _ahrs.q2;
		_2q3 = 2.0f * _ahrs.q3;
		_4q0 = 4.0f * _ahrs.q0;
		_4q1 = 4.0f * _ahrs.q1;
		_4q2 = 4.0f * _ahrs.q2;
		_8q1 = 8.0f * _ahrs.q1;
		_8q2 = 8.0f * _ahrs.q2;
		q0q0 = _ahrs.q0 * _ahrs.q0;
		q1q1 = _ahrs.q1 * _ahrs.q1;
		q2q2 = _ahrs.q2 * _ahrs.q2;
		q3q3 = _ahrs.q3 * _ahrs.q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * _ahrs.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * _ahrs.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * _ahrs.q3 - _2q1 * ax + 4.0f * q2q2 * _ahrs.q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= _ahrs.Beta * s0;
		qDot2 -= _ahrs.Beta * s1;
		qDot3 -= _ahrs.Beta * s2;
		qDot4 -= _ahrs.Beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	_ahrs.q0 += qDot1 * (1.0f / _ahrs.SampleFrequency);
	_ahrs.q1 += qDot2 * (1.0f / _ahrs.SampleFrequency);
	_ahrs.q2 += qDot3 * (1.0f / _ahrs.SampleFrequency);
	_ahrs.q3 += qDot4 * (1.0f / _ahrs.SampleFrequency);

	// Normalise quaternion
	recipNorm = invSqrt(_ahrs.q0 * _ahrs.q0 + _ahrs.q1 * _ahrs.q1 + _ahrs.q2 * _ahrs.q2 + _ahrs.q3 * _ahrs.q3);
	_ahrs.q0 *= recipNorm;
	_ahrs.q1 *= recipNorm;
	_ahrs.q2 *= recipNorm;
	_ahrs.q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
