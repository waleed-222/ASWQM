/*
 * PH_SENSOR_prg.c
 *
 *  Created on: Oct 9, 2022
 *      Author: hp
 */
#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"

#include "../../MCAL/ADC/ADC_int.h"

#include "PH_SENSOR_pri.h"
#include "PH_SENSOR_cfg.h"
#include "PH_SENSOR_int.h"

#include <util/delay.h>

/*************************************************************************************************/
/********************************Function: Initialization of PH Sensor****************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void PH_SENSOR_vInit(void)
{
	/*Initialization of ADC*/
	ADC_vInt();
}

/*************************************************************************************************/
/********************************Function: Get Value of PH****************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/

u8 PH_SENSOR_u8ReadPhValue(void)
{
	u16 L_u16Buffer[10];
	for(int i=0;i<10;i++)
	 {
		L_u16Buffer[i]=ADC_u16GetDigValSync(PH_SENSOR_ADC_CHANNEL);
	 _delay_ms(30);
	 }
	 for(u8 i=0;i<9;i++)
	 {
	 for(u8 j=i+1;j<10;j++)
	 {
	 if(L_u16Buffer[i]>L_u16Buffer[j])
	 {
	 u16 L_u16Temp=L_u16Buffer[i];
	 L_u16Buffer[i]= L_u16Buffer[j];
	 L_u16Buffer[j]= L_u16Temp;
	 }
	 }
	 }
	 u16 L_u16Avg=0;
	 for(int i=2;i<8;i++)
		 L_u16Avg+= L_u16Buffer[i];
	 f32 L_f32Volt=(float)L_u16Avg*5.0/1024/6;
	 u8 L_u8Ph_act = (float)-5.70 *  L_f32Volt + (float)(21.34-0.7);
	 L_u8Ph_act-=245;
	return L_u8Ph_act;

}


