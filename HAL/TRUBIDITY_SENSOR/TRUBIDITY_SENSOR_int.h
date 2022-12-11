/*
 * TRUBIDITY_SENSOR_int.h
 *
 *  Created on: Oct 20, 2022
 *      Author: hp
 */

#ifndef TRUBIDITY_SENSOR_TRUBIDITY_SENSOR_INT_H_
#define TRUBIDITY_SENSOR_TRUBIDITY_SENSOR_INT_H_

/*Initialization of Turbidity Sensor*/
void TRUBIDITY_SENSOR_vInit(void);

/*Get Value of Turbidity Sensor */
u32 TRUBIDITY_SENSOR_u32ReadPhValue(void);

#endif /* TRUBIDITY_SENSOR_TRUBIDITY_SENSOR_INT_H_ */
