/*
 * TDS_SENSOR_int.h
 *
 *  Created on: Oct 31, 2022
 *      Author: hp
 */

#ifndef TDS_SENSOR_TDS_SENSOR_INT_H_
#define TDS_SENSOR_TDS_SENSOR_INT_H_

#define SCOUNT  30            // sum of sample point

/*Initialization of TDS Sensor*/
void TDS_SENSOR_vInit(void);

/*Get Value of TDS Senso */
u32 TDS_SENSOR_u16GetReading(void);

/*Get Median of Samples*/
u32 getMedianNum(u32 bArray[], u32 iFilterLen);
#endif /* TDS_SENSOR_TDS_SENSOR_INT_H_ */
