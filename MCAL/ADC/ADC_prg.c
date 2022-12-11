/*
 * ADC_prg.c
 *
 *  Created on: Aug 31, 2022
 *      Author: hp
 */
#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"
#include "ADC_reg.h"
#include "ADC_pri.h"
#include "ADC_cfg.h"
#include "ADC_int.h"

/*************************************************************************************************/
/********************************Function: Initialization of ADC**********************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void ADC_vInt(void)
{
	SET_BIT(ADMUX,REFS0);
	CLR_BIT(ADMUX,REFS1); /*Voltage reference AVCC*/
	CLR_BIT(ADMUX,ADLAR);/*Right adjust*/
	CLR_BIT(ADCSRA,ADATE); /*Disable auto trigger*/
	ADCSRA=(ADCSRA&PRESCALER_MASK_BITS)|PRESCALER_MODE; /*Choose Prescaler*/
	SET_BIT(ADCSRA,ADEN);/*Enable ADC*/
}

/*************************************************************************************************/
/********************************Function: Read Value by ADC in Synchronous way*******************/
/********************************Inputs:Channel Number********************************************/
/*************************************************************************************************/
u16 ADC_u16GetDigValSync(u8 A_u8ChannelNum)
{
	ADMUX=(ADMUX&0xE0)|A_u8ChannelNum; /*Choose ADC channel*/
	SET_BIT(ADCSRA,ADSC);               /*Start Conversion*/
	while((GET_BIT(ADCSRA,ADSC))==1);   /*wait until conversion finishes*/

	return ADCW; /*return read data*/
}

/*************************************************************************************************/
/********************************Function: Read Value by ADC in ASynchronous way******************/
/********************************Inputs:Address of Application function,Channel Number************/
/*************************************************************************************************/
void ADC_u16GetDigValAsync(ptr_fun_IvOu16_t ptr_func,u8 A_u8ChannelNum)
{
	G_ISR_ADC=ptr_func; /*save address of app func*/
	ADMUX=(ADMUX&0xE0)|A_u8ChannelNum;/*Choose ADC channel*/
	SET_BIT(ADCSRA,ADIE); /*Enable Interrupt of ADC*/
	SET_BIT(ADCSRA,ADSC); /*Start Conversion*/
	while((GET_BIT(ADCSRA,ADSC))==1); /*wait until conversion finishes*/
}

/*************************************************************************************************/
/********************************Function: ADC ISR Function***************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void __vector_16 (void)
{
	if (G_ISR_ADC!=ADDRESS_NULL)
	{
		G_ISR_ADC(ADCW);
		CLR_BIT(ADCSRA,ADIE);
	}
}

