/*
 * MPU_int.h
 *
 *  Created on: Dec 1, 2022
 *      Author: hp
 */

#ifndef MPU_MPU_INT_H_
#define MPU_MPU_INT_H_

#define AHRS true         // set to false for basic data read
//#define SerialDebug true   // set to true to get Serial output for debugging

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError  M_PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
#define GyroMeasDrift  M_PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define Kp 10.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

// Specify sensor full scale
extern u8 Gscale;	// = GFS_250DPS;
extern u8 Ascale;	// = AFS_2G;
extern u8 Mscale;  // = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
extern u8 Mmode;	     // = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
f32 aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

f32   temperature;    // Stores the real internal chip temperature in degrees Celsius
f32   SelfTest[6];    // holds results of gyro and accelerometer self test

extern f32 magBias[3];
extern f32 magScale[3];

extern f32 magCalibration[3];		// = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
extern f32 gyroBias[3];		// = {0, 0, 0};
extern f32 accelBias[3];		// = {0, 0, 0};      // Bias corrections for gyro and accelerometer
u16 tempCount;      // temperature raw count output

extern f32 deltat;	// = 0;
extern f32 sum; 	//= 0;        // integration interval for both filter schemes

extern f32 beta;		//= 0.8660254 * GyroMeasError; //sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
extern f32 zeta;		// = 0.8660254 * GyroMeasDrift; //sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

void mpu9250_setup(void);

/*---------------------------------- Lokale Funktionen -----------------------------------------*/
void writeByte(u8 address, u8 subAddress, u8 data);
u8 readByte(u8 address, u8 subAddress);
void readBytes(u8 address, u8 subAddress, u8 count, u8 * dest);

u8 readByte_Debug(u8 address, u8 subAddress);
void readBytes_Debug(u8 address, u8 subAddress, u8 count, u8 * dest);

void magcalMPU9250(f32 * dest1, f32 * dest2);
void getMres(void);
void getGres(void);
void getAres(void);
void readAccelData(u16 * destination);
void getMres(void);
void readMagData(u16 * destination);
u16 readTempData(void);
void initAK8963(f32 * destination);
void readGyroData(u16 * destination);
void initMPU9250(void);
void MPU9250SelfTest(f32 * destination);
void calibrateMPU9250(f32 * dest1, f32 * dest2);
void MadgwickQuaternionUpdate(f32 ax, f32 ay, f32 az, f32 gx, f32 gy, f32 gz, f32 mx, f32 my, f32 mz, f32 * quaternionBuffer);

#endif /* MPU_MPU_INT_H_ */
