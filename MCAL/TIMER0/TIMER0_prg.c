/*
 * TIMER_prg.c
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */
#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"
#include "../DIO/DIO_int.h"
#include "TIMER0_reg.h"
#include "TIMER0_pri.h"
#include "TIMER0_cfg.h"
#include "TIMER0_int.h"



/*************************************************************************************************/
/********************************Function: Initialization of Timer0*******************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void TIMER0_vInit(void)
{

	/*Timer modes*/
#if TIMER0_MODE==TIMER0_NORMAL_MODE

	TCCR0&=TIMER0_MODE_MASK;
	TCCR0|=TIMER0_NORMAL_MODE;/*Normal mode*/

	CLR_BIT(TCCR0,COM00);
	CLR_BIT(TCCR0,COM01);/*Disconnected OC0*/

	/*Set the interrupt*/
#if TIMER0_INTERRUPT_STATE_OVF==TIMER0_INT_ENABLE
	TIMSK=(TIMSK&254)|1;/*enable OVF interrupt*/
#endif

#elif TIMER0_MODE==TIMER0_CTC_MODE
	TCCR0=(TCCR0&TIMER0_MODE_MASK)|TIMER0_CTC_MODE; /*CTC mode*/
#if TIMER0_INTERRUPT_STATE_CTC==TIMER0_INT_ENABLE
	SET_BIT(TIMSK, OCIE0);/*enable OC interrupt*/
#endif
#if TIMER0_OCP_PIN_MODE==TIMER0_OC0_DISCONNECTED
	CLR_BIT(TCCR0,COM00);
	CLR_BIT(TCCR0,COM01);/*Disconnected OC0*/

#elif  TIMER0_OCP_PIN_MODE==TIMER0_OC0_TOGGLE
	/*Must set the OC0 pin output if in CTC, fast PWM or phase correct PWM modes*/
	DIO_vPinDir(OC0_PORT, OC0_PIN,DIR_OUTPUT);
	CLR_BIT(TCCR0,COM01);
	SET_BIT(TCCR0,COM00);/*Toggle OC0*/

#elif	  TIMER0_OCP_PIN_MODE==TIMER0_OC0_CLR
	DIO_vPinDir(OC0_PORT, OC0_PIN,DIR_OUTPUT);
	CLR_BIT(TCCR0,COM00);
	SET_BIT(TCCR0,COM01);/*CLR OC0*/

#elif	 TIMER0_OCP_PIN_MODE== TIMER0_OC0_SET
	DIO_vPinDir(OC0_PORT, OC0_PIN,DIR_OUTPUT);
	SET_BIT(TCCR0,COM00);
	SET_BIT(TCCR0,COM01);/*SET OC0*/

#else
#warning "Invalid OCP Pin Mode Option ...."

#endif

#elif   TIMER0_MODE==TIMER0_FAST_PWM_MODE

	TCCR0=(TCCR0&TIMER0_MODE_MASK)|TIMER0_FAST_PWM_MODE; /*FAST_PWM mode*/

#if TIMER0_INTERRUPT_STATE_OVF==TIMER0_INT_ENABLE
	TIMSK=(TIMSK&254)|1;/*enable OVF interrupt*/
#endif
#if TIMER0_INTERRUPT_STATE_CTC==TIMER0_INT_ENABLE
	TIMSK=(TIMSK&253)|2;/*enable OC interrupt*/
#endif

	DIO_vPinDir(OC0_PORT, OC0_PIN,DIR_OUTPUT);

#if TIMER0_PWM_MODE==TIMER0_PWM_NON_INVERTING
	CLR_BIT(TCCR0,COM00);
	SET_BIT(TCCR0,COM01);/*CLR OC0*/

#elif TIMER0_PWM_MODE==TIMER0_PWM_INVERTING
	SET_BIT(TCCR0,COM00);
	SET_BIT(TCCR0,COM01);/*SET OC0*/

#else
#warning "Invalid PWM Mode Option ...."

#endif

#elif    TIMER0_MODE==TIMER0_PWM_PHASE_CORRECT_MODE
	CCR0=(TCCR0&TIMER0_MODE_MASK)|TIMER0_PWM_PHASE_CORRECT_MODE; /*PWM phase correct mode*/
#if TIMER0_INTERRUPT_STATE_OVF==TIMER0_INT_ENABLE
	TIMSK=(TIMSK&254)|1;/*enable OVF interrupt*/
#endif
#if TIMER0_INTERRUPT_STATE_CTC==TIMER0_INT_ENABLE
	TIMSK=(TIMSK&253)|2;/*enable OC interrupt*/
#endif
	DIO_vPinDir(OC0_PORT, OC0_PIN,DIR_OUTPUT);

#if TIMER0_PWM_MODE==TIMER0_PWM_NON_INVERTING
	CLR_BIT(TCCR0,COM00);
	SET_BIT(TCCR0,COM01);/*CLR OC0*/

#elif TIMER0_PWM_MODE==TIMER0_PWM_INVERTING
	SET_BIT(TCCR0,COM00);
	SET_BIT(TCCR0,COM01);/*SET OC0*/

#else
#warning "Invalid PWM Mode Option ...."

#endif

#else
#warning "Invalid Timer Mode Option ...."



#endif
	/*Disable force output compare by default*/
	CLR_BIT(TCCR0, FOC0);


}

/*************************************************************************************************/
/********************************Function: Turn On Timer0*****************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void TIMER0_vTurnOn(void)
{
	TCCR0&=TIMER0_CLK_SELECT_MASK;
	TCCR0|=TIMER0_CLOCK_SELECT; /*Timer0 on and prescaler 1024  */

}

/*************************************************************************************************/
/********************************Function: Turn Off Timer0****************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void TIMER0_vTurnOff(void)
{
	TCCR0&=TIMER0_CLK_SELECT_MASK;

}

/*************************************************************************************************/
/********************************Function: Set Preload of Timer0**********************************/
/********************************Inputs:Preload Value*********************************************/
/*************************************************************************************************/
void TIMER0_vSetPreload(u8 A_u8PreloadVal)
{
	TCNT0=A_u8PreloadVal;
}

/*************************************************************************************************/
/********************************Function: Set OCRA of Timer0*************************************/
/********************************Inputs:OCRA Value************************************************/
/*************************************************************************************************/
void TIMER0_vSetOcrVal(u8 A_u8OcrVal)
{
	OCR0=A_u8OcrVal;
}


/*set callback to execute ISR related with OVR Event*/
void TIMER0_vCallBack_OVF(ptr_fun_IvOv_t ptr)
{
	G_PTRF_TIMER0_OVF=ptr;
}

/*set callback to execute ISR related with CTC Event*/
void TIMER0_vCallBack_CTC(ptr_fun_IvOv_t ptr)
{
	G_PTRF_TIMER0_CTC=ptr;
}

u8 TIMER0_u8DutyCycle()
{
	u8 L_u8DutyCycle;
	/*find mode of timer*/
	switch(TIMER0_MODE)
	{
	case TIMER0_CTC_MODE:
#if TIMER0_OCP_PIN_MODE==TIMER0_OC0_DISCONNECTED
		L_u8DutyCycle=0;/*Disconnected OC0*/

#elif  TIMER0_OCP_PIN_MODE==TIMER0_OC0_TOGGLE
		/*Must set the OC0 pin output if in CTC, fast PWM or phase correct PWM modes*/
		L_u8DutyCycle=50;/*Toggle OC0*/

#elif	  TIMER0_OCP_PIN_MODE==TIMER0_OC0_CLR
		L_u8DutyCycle=0;/*CLR OC0*/

#elif	 TIMER0_OCP_PIN_MODE== TIMER0_OC0_SET
		L_u8DutyCycle=100;/*Set OC0*/
#else
#warning "Invalid DutyCycle of CTC...."
#endif
		break;
	case TIMER0_FAST_PWM_MODE:
#if TIMER0_PWM_MODE==TIMER0_PWM_NON_INVERTING
		L_u8DutyCycle=(OCR0*100ul)/TIMER0_MAX_COUNT;/*CLR OC0*/

#elif TIMER0_PWM_MODE==TIMER0_PWM_INVERTING
		L_u8DutyCycle=1-((OCR0*100)/TIMER0_MAX_COUNT);/*SET OC0*/

#else
#warning "Invalid PWM Mode Option ...."

#endif
		break;
	}
	return L_u8DutyCycle;
}

/*Get frequency of wave on OC0 */
u32 TIMER0_u32FrequencyOfOC0Pin()
{
	u32 L_u32FrequencyOC0;
	/*find mode of timer*/
	switch(TIMER0_MODE)
	{
	case TIMER0_CTC_MODE:
#if  TIMER0_OCP_PIN_MODE==TIMER0_OC0_TOGGLE
		/*Toggle OC0*/
		switch(TIMER0_CLOCK_SELECT)
		{
		case TIMER0_NO_PRESCALER:
			L_u32FrequencyOC0=TIMER0_InputFreq/(2*(1+OCR0));
			break;
		case TIMER0_8_PRESCALER:
			L_u32FrequencyOC0=TIMER0_InputFreq/(2*8*(1+OCR0));
			break;
		case TIMER0_64_PRESCALER:
			L_u32FrequencyOC0=TIMER0_InputFreq/(2*64*(1+OCR0));
			break;
		case TIMER0_256_PRESCALER:
			L_u32FrequencyOC0=TIMER0_InputFreq/(2*256*(1+OCR0));
			break;
		case TIMER0_1024_PRESCALER:
			L_u32FrequencyOC0=TIMER0_InputFreq/(2*1024*(1+OCR0));
			break;

		}
#else
#warning "Invalid frequency of CTC...."
#endif
		break;
		case TIMER0_FAST_PWM_MODE:
			switch(TIMER0_CLOCK_SELECT)
			{
			case TIMER0_NO_PRESCALER:
				L_u32FrequencyOC0=TIMER0_InputFreq/(TIMER0_MAX_COUNT);
				break;
			case TIMER0_8_PRESCALER:
				L_u32FrequencyOC0=TIMER0_InputFreq/(8 *(TIMER0_MAX_COUNT));
				break;
			case TIMER0_64_PRESCALER:
				L_u32FrequencyOC0=TIMER0_InputFreq/(64 *(TIMER0_MAX_COUNT));
				break;
			case TIMER0_256_PRESCALER:
				L_u32FrequencyOC0=TIMER0_InputFreq/((u32)256* TIMER0_MAX_COUNT );
				break;
			case TIMER0_1024_PRESCALER:
				L_u32FrequencyOC0=TIMER0_InputFreq/((u32)1024*(TIMER0_MAX_COUNT));
				break;

			}
			break;
			case TIMER0_PWM_PHASE_CORRECT_MODE:
				switch(TIMER0_CLOCK_SELECT)
				{
				case TIMER0_NO_PRESCALER:
					L_u32FrequencyOC0=TIMER0_InputFreq/((TIMER0_MAX_COUNT)-2);
					break;
				case TIMER0_8_PRESCALER:
					L_u32FrequencyOC0=TIMER0_InputFreq/(8 *((TIMER0_MAX_COUNT)-2));
					break;
				case TIMER0_64_PRESCALER:
					L_u32FrequencyOC0=TIMER0_InputFreq/(64 *((TIMER0_MAX_COUNT)-2));
					break;
				case TIMER0_256_PRESCALER:
					L_u32FrequencyOC0=TIMER0_InputFreq/((u32)256 *((TIMER0_MAX_COUNT)-2));
					break;
				case TIMER0_1024_PRESCALER:
					L_u32FrequencyOC0=TIMER0_InputFreq/((u32)1024*((TIMER0_MAX_COUNT)-2));
					break;

				}
				break;
	}
	return L_u32FrequencyOC0;
}

/*TIMER0 COMP*/
void __vector_10(void){
	if(G_PTRF_TIMER0_CTC != ADDRESS_NULL){
		G_PTRF_TIMER0_CTC();
	} else {
		/*Handle callback error*/
	}
}

/*TIMER0 OVF*/
void __vector_11(void){
	G_PTRF_TIMER0_OVF();
}
