/*
 * MPU_pri.h
 *
 *  Created on: Dec 1, 2022
 *      Author: hp
 */

#ifndef MPU_MPU_PRI_H_
#define MPU_MPU_PRI_H_

// Set initial input parameters
enum Ascale_values {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale_values {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale_values {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

#define ADO 1
#if ADO
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif

#endif /* MPU_MPU_PRI_H_ */
