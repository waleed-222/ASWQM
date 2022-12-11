/*
 * WATER_FlOW_prg.c
 *
 *  Created on: Oct 20, 2022
 *      Author: hp
 */

#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"

#include "../../MCAL/DIO/DIO_int.h"
#include "../../MCAL/ADC/ADC_int.h"
#include "../../MCAL/TIMER1/TIMER1_int.h"
#include "../../MCAL/GIE/GIE_int.h"

#include "WATER_FlOW_pri.h"
#include "WATER_FlOW_cfg.h"
#include "WATER_FlOW_int.h"

u32 G_u32Counts=0;
/*************************************************************************************************/
/********************************Function: Initialization of Water Flow Sensor********************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void WATER_FLOW_vInit(void)
{
	/*Initialization of ICU*/
	Icu_Init();

	/*Pass Address of ISR of ICU*/
	TIMER1_vCallBack_CE(ISR_ICU);

	/*Initialization of ADC*/
	ADC_vInt();

	/*Enable GIE*/
	GIE_vEnableGlobalInterrupt();

	/*Select ICU Edge*/
	Icu_SetEdge(TIMER1_ICU_RISING_EDGE);





}

void ISR_ICU(void)
{
	G_u32Counts++;
}
