/*
 * TRUBIDITY_SENSOR_prg.c
 *
 *  Created on: Oct 20, 2022
 *      Author: hp
 */

#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"

#include "../../MCAL/ADC/ADC_int.h"

#include "TRUBIDITY_SENSOR_pri.h"
#include "TRUBIDITY_SENSOR_cfg.h"
#include "TRUBIDITY_SENSOR_int.h"

#include <util/delay.h>

/*************************************************************************************************/
/********************************Function: Initialization of Turbidity Sensor*********************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void TRUBIDITY_SENSOR_vInit(void)
{
	/*Initialization of ADC*/
	ADC_vInt();
}

/*************************************************************************************************/
/********************************Function: Get Value of Turbidity*********************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
u32 TRUBIDITY_SENSOR_u32ReadPhValue(void)
{
	u16 L_u16PhAnalogVal=ADC_u16GetDigValSync(Turbidity_SENSOR_ADC_CHANNEL);
	f32 L_f32VoltValue=L_u16PhAnalogVal*(5/1024.0);
	u32 L_u32NTU;
	if(L_f32VoltValue<2.5)
		L_u32NTU=3000;
	else
		L_u32NTU=(float)(-1120.4* L_f32VoltValue*L_f32VoltValue)+(float)(5742.3*L_f32VoltValue-4353.8);
	_delay_ms(500);
	return L_u32NTU;

}
