/*
 * TIMER1_prg.c
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"
#include "../DIO/DIO_int.h"
#include "TIMER1_reg.h"
#include "TIMER1_pri.h"
#include "TIMER1_cfg.h"
#include "TIMER1_int.h"


void TIMER1_vInit(void)
{
	/*Timer modes*/
#if TIMER1_MODE==TIMER1_NORMAL_MODE

	CLR_BIT(TCCR1A,WGM10);
	CLR_BIT(TCCR1A,WGM11);
	CLR_BIT(TCCR1B,WGM12);
	CLR_BIT(TCCR1B,WGM13);/*Normal mode*/

	CLR_BIT(TCCR1A,COM1A0);
	CLR_BIT(TCCR1A,COM1A1);/*Disconnected OC1A*/

	CLR_BIT(TCCR1A,COM1B0);
	CLR_BIT(TCCR1A,COM1B1);/*Disconnected OC1B*/

#if TIMER1_ICU_STATE==TIMER1_ICU_ENABLE
	SET_BIT(TCCR1B,ICES1);
	SET_BIT(TIMSK,TICIE1); //Enable interrupt ICU
#endif


	/*Set the interrupt*/
#if TIMER1_INTERRUPT_STATE_OVF==TIMER1_INT_ENABLE
	SET_BIT(TIMSK,TOIE1) ;/*enable OVF interrupt*/
#endif

#elif TIMER1_MODE==TIMER1_CTC_OCR1A
	CLR_BIT(TCCR1A,WGM10);
	CLR_BIT(TCCR1A,WGM11);
	SET_BIT(TCCR1B,WGM12);
	CLR_BIT(TCCR1B,WGM13); /*CTC_OCR1A mode*/

#if TIMER1_INTERRUPT_STATE_CTCA==TIMER1_INT_ENABLE
	SET_BIT(TIMSK, OCIE1A);/*enable OC1A interrupt*/
#endif

	CLR_BIT(TCCR1A,COM1B0);
	CLR_BIT(TCCR1A,COM1B1);/*Disconnected OC1B*/
#if TIMER1_OCP_PIN_MODE==TIMER1_OC1_DISCONNECTED
	CLR_BIT(TCCR1A,COM1A0);
	CLR_BIT(TCCR1A,COM1A1);/*Disconnected OC1A*/

#elif  TIMER1_OCP_PIN_MODE==TIMER1_OC1_TOGGLE
	/*Must set the OCA1 pin output if in CTC, fast PWM or phase correct PWM modes*/
	DIO_vPinDir(OC1A_PORT, OC1A_PIN,DIR_OUTPUT);
	CLR_BIT(TCCR1A,COM1A1);
	SET_BIT(TCCR1A,COM1A0);/*Toggle OC1A*/

#elif	  TIMER1_OCP_PIN_MODE==TIMER1_OC1_CLR
	DIO_vPinDir(OC1A_PORT, OC1A_PIN,DIR_OUTPUT);
	CLR_BIT(TCCR1A,COM1A0);
	SET_BIT(TCCR1A,COM1A1);/*CLR OC1A*/

#elif	 TIMER1_OCP_PIN_MODE== TIMER1_OC1_SET
	DIO_vPinDir(OC1A_PORT, OC1A_PIN,DIR_OUTPUT);
	SET_BIT(TCCR1A,COM1A0);
	SET_BIT(TCCR1A,COM1A1);/*SET OC1A*/

#else
#warning "Invalid OCP Pin Mode Option ...."

#endif

#elif   TIMER1_MODE==TIMER1_FAST_PWM_ICR1

	CLR_BIT(TCCR1A,WGM10);
	SET_BIT(TCCR1A,WGM11);
	SET_BIT(TCCR1B,WGM12);
	SET_BIT(TCCR1B,WGM13); /*Fast PWM ICR1 mode*/

#if TIMER1_INTERRUPT_STATE_OVF==TIMER1_INT_ENABLE
	SET_BIT(TIMSK,TOIE1) ;/*enable OVF interrupt*/
#endif

#if TIMER1_INTERRUPT_STATE_CTCA==TIMER1_INT_ENABLE
	SET_BIT(TIMSK, OCIE1A);/*enable OC1A interrupt*/
#endif

#if TIMER1_INTERRUPT_STATE_CTCB==TIMER1_INT_ENABLE
	SET_BIT(TIMSK, OCIE1B);/*enable OC1B interrupt*/
#endif

	DIO_vPinDir(OC1A_PORT, OC1A_PIN,DIR_OUTPUT);
	DIO_vPinDir(OC1B_PORT, OC1B_PIN,DIR_OUTPUT);


#if TIMER1_PWM_MODE_OC1A==TIMER1_PWM_NON_INVERTING
	CLR_BIT(TCCR1A,COM1A0);
	SET_BIT(TCCR1A,COM1A1);/*CLR OC1A*/

#elif TIMER1_PWM_MODE_OC1A==TIMER1_PWM_INVERTING
	SET_BIT(TCCR1A,COM1A0);
	SET_BIT(TCCR1A,COM1A1);/*SET OC1A*/

#else
#warning "Invalid PWM Mode Option ...."

#endif



#else
#warning "Invalid Timer Mode Option ...."



#endif


}

/*set preload to start count from the preload value*/
void TIMER1_vSetPreload(u8 A_u8PreloadVal)
{

}

/*set OCR1A to be the top counts to generate CTC flag*/
void TIMER1_vSetOcrA1Val(u16 A_u16Ocr1aVal)
{
	OCR1A=A_u16Ocr1aVal;
}

/*set OCR1B to be the top counts to generate CTC flag*/
void TIMER1_vSetOcrB1Val(u16 A_u16Ocr1bVal)
{
	OCR1B=A_u16Ocr1bVal;
}

/*Set ICR1 to be the top counts*/
void TIMER1_vSetICR1(u16 A_u16ICR)
{
	ICR1=A_u16ICR;
}

/*set callback to execute ISR related with OVR Event*/
void TIMER1_vCallBack_OVF(void(*Fptr)(void))
{
	G_PTRF_TIMER1_OVF=Fptr;
}

/*set callback to execute ISR related with CTCA Event*/
void TIMER1_vCallBack_CTCA(void(*Fptr)(void))
{
	G_PTRF_TIMER1_CTCA=Fptr;
}

/*set callback to execute ISR related with CTCB Event*/
void TIMER1_vCallBack_CTCB(void(*Fptr)(void))
{
	G_PTRF_TIMER1_CTCB=Fptr;
}

/*set callback to execute ISR related with Capture Event*/
void TIMER1_vCallBack_CE(void(*Fptr)(void))
{
	G_PTRF_TIMER1_CE=Fptr;
}

/*Turn On Timer1*/
void TIMER1_vTurnOn(void)
{
	TCCR1B&=TIMER1_CLK_SELECT_MASK;
	TCCR1B|=TIMER1_CLOCK_SELECT;
}

/*Turn Off Timer1*/
void TIMER1_vTurnOff(void)
{
	TCCR1B&=TIMER1_CLK_SELECT_MASK;
}

/*make Delay*/
void TIMER1_vDelay(u8 A_u8TimeBySecond)
{

}

/*Get Duty cycle of wave on OC1A */
u8 TIMER1_u8DutyCycleOc1A()
{
	return 0;
}

/*Get Duty cycle of wave on OC1B */
u8 TIMER1_u8DutyCycleOc1B()
{
	return 0;
}

/*Get frequency of wave on OC1A */
u32 TIMER1_u32FrequencyOfOC1APin()
{
	return 0;
}

/*Get frequency of wave on OC1B */
u32 TIMER1_u32FrequencyOfOC1BPin()
{
	return 0;
}

/*Read ICR1*/
u16 TIMER1_u16ReadICR1()
{
	return ICR1;
}

/*choose trigger*/
void TIMER1_vSetTrigger(u8 A_u8Trigger)
{
	switch(A_u8Trigger)
	{
	case TIMER1_ICU_FALLING_EDGE: CLR_BIT(TCCR1B,ICES1);break;
	case TIMER1_ICU_RISING_EDGE:  SET_BIT(TCCR1B,ICES1);break;
	}
}

/*Disable Inerrupt of ICU*/
void TIMER1_vDisableIcuInt()
{
	CLR_BIT(TIMSK,TICIE1);
}

/* Capture Event Timer1 ISR*/
void __vector_6 (void)
{
	G_PTRF_TIMER1_CE();
}

/* Compare Match A Timer1 ISR*/
void __vector_7 (void)
{
	G_PTRF_TIMER1_CTCA();
}

/* Compare Match B Timer1 ISR*/
void __vector_8 (void)
{
	G_PTRF_TIMER1_CTCB();
}

/* OVF TIMER1 ISR*/
void __vector_9 (void)
{
	G_PTRF_TIMER1_OVF();
}

void Icu_Init()
{
	/* Input Capture Noise Canceler */
	SET_BIT(TCCR1B, ICNC1);
	/* Set ICP Pin Input Floating */
	DIO_vPinDir(PORTD_ID, PIN6_ID, DIR_INPUT);
	DIO_vSetPinVal(PORTD_ID, PIN6_ID, VAL_HIGH);

#if TIMER1_INTERRUPT_STATE_CE==TIMER1_INT_ENABLE
	SET_BIT(TIMSK,TICIE1); //Enable interrupt ICU
#else
	CLR_BIT(TIMSK,TICIE1); //Disable interrupt ICU
#endif
}

void Icu_SetEdge(u8 A_u8Edge)
{
	switch (A_u8Edge)
	{
	case TIMER1_ICU_FALLING_EDGE:
		CLR_BIT(TCCR1B, ICES1);
		break;
	case TIMER1_ICU_RISING_EDGE:
		SET_BIT(TCCR1B, ICES1);
		break;
	default:
		break;
	}
}

