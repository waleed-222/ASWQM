/*
 * TIMER2_prg.c
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"
#include "../DIO/DIO_int.h"
#include "TIMER2_reg.h"
#include "TIMER2_pri.h"
#include "TIMER2_cfg.h"
#include "TIMER2_int.h"




void TIMER2_vInit(void)
{

	/*Timer modes*/
#if TIMER2_MODE==TIMER2_NORMAL_MODE

    TCCR2&=TIMER2_MODE_MASK;
    TCCR2|=TIMER2_NORMAL_MODE;/*Normal mode*/

    CLR_BIT(TCCR2,COM20);
    CLR_BIT(TCCR2,COM21);/*Disconnected OC2*/

/*Set the interrupt*/
		#if TIMER2_INTERRUPT_STATE_OVF==TIMER2_INT_ENABLE
    SET_BIT(TIMSK,TOIE2);/*enable OVF interrupt*/
        #endif

#elif TIMER2_MODE==TIMER2_CTC_MODE
TCCR2=(TCCR2&TIMER2_MODE_MASK)|TIMER2_CTC_MODE; /*CTC mode*/
#if TIMER2_INTERRUPT_STATE_CTC==TIMER2_INT_ENABLE
SET_BIT(TIMSK, OCIE2);/*enable OC interrupt*/
#endif
#if TIMER2_OCP_PIN_MODE==TIMER2_OC2_DISCONNECTED
CLR_BIT(TCCR2,COM20);
CLR_BIT(TCCR2,COM21);/*Disconnected OC2*/

#elif  TIMER2_OCP_PIN_MODE==TIMER2_OC2_TOGGLE
/*Must set the OC2 pin output if in CTC, fast PWM or phase correct PWM modes*/
		DIO_vPinDir(OC2_PORT, OC2_PIN,DIR_OUTPUT);
CLR_BIT(TCCR2,COM21);
SET_BIT(TCCR2,COM20);/*Toggle OC2*/

#elif	  TIMER2_OCP_PIN_MODE==TIMER2_OC2_CLR
DIO_vPinDir(OC2_PORT, OC2_PIN,DIR_OUTPUT);
CLR_BIT(TCCR2,COM20);
SET_BIT(TCCR2,COM21);/*CLR OC2*/

#elif	 TIMER2_OCP_PIN_MODE== TIMER2_OC2_SET
DIO_vPinDir(OC2_PORT, OC2_PIN,DIR_OUTPUT);
SET_BIT(TCCR2,COM20);
SET_BIT(TCCR2,COM21);/*SET OC2*/

#else
#warning "Invalid OCP Pin Mode Option ...."

#endif

#elif   TIMER2_MODE==TIMER2_FAST_PWM_MODE

  TCCR2=(TCCR2&TIMER2_MODE_MASK)|TIMER2_FAST_PWM_MODE; /*FAST_PWM mode*/

       #if TIMER2_INTERRUPT_STATE_OVF==TIMER2_INT_ENABLE
  SET_BIT(TIMSK,TOIE2);/*enable OVF interrupt*/
       #endif
#if TIMER2_INTERRUPT_STATE_CTC==TIMER2_INT_ENABLE
  SET_BIT(TIMSK,OCIE2);/*enable OC interrupt*/
#endif

DIO_vPinDir(OC2_PORT, OC2_PIN,DIR_OUTPUT);

#if TIMER2_PWM_MODE==TIMER2_PWM_NON_INVERTING
CLR_BIT(TCCR2,COM20);
SET_BIT(TCCR2,COM21);/*CLR OC0*/

#elif TIMER2_PWM_MODE==TIMER2_PWM_INVERTING
SET_BIT(TCCR2,COM20);
SET_BIT(TCCR2,COM21);/*SET OC0*/

#else
#warning "Invalid PWM Mode Option ...."

#endif

#elif    TIMER2_MODE==TIMER2_PWM_PHASE_CORRECT_MODE
CCR0=(TCCR0&TIMER2_MODE_MASK)|TIMER2_PWM_PHASE_CORRECT_MODE; /*PWM phase correct mode*/
#if TIMER2_INTERRUPT_STATE_OVF==TIMER2_INT_ENABLE
          SET_BIT(TIMSK,TOIE2);/*enable OVF interrupt*/
       #endif
#if TIMER2_INTERRUPT_STATE_CTC==TIMER2_INT_ENABLE
          SET_BIT(TIMSK ,OCIE2);/*enable OC interrupt*/
#endif
DIO_vPinDir(OC2_PORT, OC2_PIN,DIR_OUTPUT);

#if TIMER2_PWM_MODE==TIMER2_PWM_NON_INVERTING
CLR_BIT(TCCR2,COM20);
SET_BIT(TCCR2,COM21);/*CLR OC2*/

#elif TIMER2_PWM_MODE==TIMER2_PWM_INVERTING
SET_BIT(TCCR2,COM20);
SET_BIT(TCCR2,COM21);/*SET OC2*/

#else
#warning "Invalid PWM Mode Option ...."

#endif

#else
#warning "Invalid Timer Mode Option ...."



#endif
/*Disable force output compare by default*/
	CLR_BIT(TCCR2, FOC2);


}

/*Turn ON Timer*/
void TIMER2_vTurnOn(void)
{
	TCCR2&=TIMER2_CLK_SELECT_MASK;
	TCCR2|=TIMER2_CLOCK_SELECT; /*Timer0 on and prescaler 1024  */

}


/*Turn Off Timer0*/
void TIMER2_vTurnOff(void)
{
	TCCR2&=TIMER2_CLK_SELECT_MASK;

}

/*set preload to start count from the preload value*/
void TIMER2_vSetPreload(u8 A_u8PreloadVal)
{
	TCNT2=A_u8PreloadVal;
}

/*set ocra to be the top counts to generate CTC flag*/
void TIMER2_vSetOcrVal(u8 A_u8OcrVal)
{
	OCR2=A_u8OcrVal;
}

void TIMER2_vDelay(u8 A_u8TimeBySecond)
{
	TCCR2=(TCCR2&TIMER2_MODE_MASK)|TIMER2_CTC_MODE; /*CTC mode*/
	TIMER2_vSetOcrVal(250);
	u8 L_u8Num_OCM=A_u8TimeBySecond/OCR2;
#if TIMER2_INTERRUPT_STATE_CTC==TIMER2_INT_ENABLE
SET_BIT(TIMSK, OCIE2);/*enable OC interrupt*/
#endif
}
/*set callback to execute ISR related with OVR Event*/
void TIMER2_vCallBack_OVF(ptr_fun_IvOv_t ptr)
{
	G_PTRF_TIMER2_OVF=ptr;
}

/*set callback to execute ISR related with CTC Event*/
void TIMER2_vCallBack_CTC(ptr_fun_IvOv_t ptr)
{
	G_PTRF_TIMER2_CTC=ptr;
}

u8 TIMER2_u8DutyCycle()
{
	u8 L_u8DutyCycle;
	/*find mode of timer*/
	switch(TIMER2_MODE)
	{
	case TIMER2_CTC_MODE:
#if TIMER2_OCP_PIN_MODE==TIMER0_OC2_DISCONNECTED
L_u8DutyCycle=0;/*Disconnected OC2*/

#elif  TIMER2_OCP_PIN_MODE==TIMER2_OC2_TOGGLE
/*Must set the OC2 pin output if in CTC, fast PWM or phase correct PWM modes*/
L_u8DutyCycle=50;/*Toggle OC2*/

#elif	  TIMER2_OCP_PIN_MODE==TIMER2_OC2_CLR
L_u8DutyCycle=0;/*CLR OC2*/

#elif	 TIMER2_OCP_PIN_MODE== TIMER2_OC2_SET
L_u8DutyCycle=100;/*Set OC2*/
#else
#warning "Invalid DutyCycle of CTC...."
#endif
		break;
	case TIMER2_FAST_PWM_MODE:
#if TIMER2_PWM_MODE==TIMER2_PWM_NON_INVERTING
		L_u8DutyCycle=(OCR2*100ul)/TIMER2_MAX_COUNT;/*CLR OC2*/

#elif TIMER2_PWM_MODE==TIMER2_PWM_INVERTING
		L_u8DutyCycle=1-((OCR2*100)/TIMER0_MAX_COUNT);/*SET OC2*/

#else
#warning "Invalid PWM Mode Option ...."

#endif
		break;
	}
	return L_u8DutyCycle;
}

/*Get frequency of wave on OC2 */
u32 TIMER2_u32FrequencyOfOC2Pin()
{
	u32 L_u32FrequencyOC2;
		/*find mode of timer*/
		switch(TIMER2_MODE)
		{
		case TIMER2_CTC_MODE:
#if  TIMER2_OCP_PIN_MODE==TIMER2_OC2_TOGGLE
	/*Toggle OC2*/
			switch(TIMER2_CLOCK_SELECT)
			{
			case TIMER2_NO_PRESCALER:
				L_u32FrequencyOC2=TIMER0_InputFreq/(2*(1+OCR2));
				break;
			case TIMER2_8_PRESCALER:
				L_u32FrequencyOC2=TIMER2_InputFreq/(2*8*(1+OCR2));
				break;
			case TIMER2_32_PRESCALER:
				L_u32FrequencyOC2=TIMER2_InputFreq/(2*32*(1+OCR2));
				break;
			case TIMER0_64_PRESCALER:
				L_u32FrequencyOC2=TIMER2_InputFreq/(2*64*(1+OCR2));
				break;
			case TIMER0_128_PRESCALER:
				L_u32FrequencyOC2=TIMER2_InputFreq/(2*128*(1+OCR2));
				break;
			case TIMER0_256_PRESCALER:
				L_u32FrequencyOC2=TIMER2_InputFreq/(2*256*(1+OCR2));
				break;
			case TIMER0_1024_PRESCALER:
				L_u32FrequencyOC2=TIMER2_InputFreq/(2*1024*(1+OCR2));
				break;

			}
	#else
	#warning "Invalid frequency of CTC...."
	#endif
			break;
		case TIMER2_FAST_PWM_MODE:
			switch(TIMER2_CLOCK_SELECT)
									{
									case TIMER2_NO_PRESCALER:
										L_u32FrequencyOC2=TIMER2_InputFreq/(TIMER2_MAX_COUNT);
										break;
									case TIMER2_8_PRESCALER:
										L_u32FrequencyOC2=TIMER2_InputFreq/(8 *(TIMER2_MAX_COUNT));
										break;
									case TIMER2_32_PRESCALER:
										L_u32FrequencyOC2=TIMER2_InputFreq/(32 *(TIMER2_MAX_COUNT));
										break;
									case TIMER2_64_PRESCALER:
										L_u32FrequencyOC2=TIMER2_InputFreq/(64 *(TIMER2_MAX_COUNT));
										break;
									case TIMER2_128_PRESCALER:
										L_u32FrequencyOC2=TIMER2_InputFreq/(128 *(TIMER2_MAX_COUNT));
										break;
									case TIMER2_256_PRESCALER:
										L_u32FrequencyOC2=TIMER2_InputFreq/(256* TIMER2_MAX_COUNT );
										break;
									case TIMER2_1024_PRESCALER:
										L_u32FrequencyOC2=TIMER2_InputFreq/(1024*(TIMER2_MAX_COUNT));
										break;

									}
			break;
			case TIMER2_PWM_PHASE_CORRECT_MODE:
				switch(TIMER2_CLOCK_SELECT)
													{
													case TIMER2_NO_PRESCALER:
														L_u32FrequencyOC2=TIMER2_InputFreq/((TIMER2_MAX_COUNT)-2);
														break;
													case TIMER2_8_PRESCALER:
														L_u32FrequencyOC2=TIMER2_InputFreq/(8 *((TIMER2_MAX_COUNT)-2));
														break;
													case TIMER2_32_PRESCALER:
														L_u32FrequencyOC2=TIMER2_InputFreq/(32 *((TIMER2_MAX_COUNT)-2));
														break;
													case TIMER2_64_PRESCALER:
														L_u32FrequencyOC2=TIMER2_InputFreq/(64 *((TIMER2_MAX_COUNT)-2));
														break;
													case TIMER2_128_PRESCALER:
														L_u32FrequencyOC2=TIMER2_InputFreq/(128 *((TIMER2_MAX_COUNT)-2));
													    break;
													case TIMER2_256_PRESCALER:
														L_u32FrequencyOC2=TIMER2_InputFreq/(256 *((TIMER2_MAX_COUNT)-2));
														break;
													case TIMER2_1024_PRESCALER:
														L_u32FrequencyOC2=TIMER2_InputFreq/(1024*((TIMER2_MAX_COUNT)-2));
														break;

													}
							break;
		}
		return L_u32FrequencyOC2;
}

/*TIMER0 COMP*/
void __vector_4(void){
	if(G_PTRF_TIMER2_CTC != ADDRESS_NULL){
		G_PTRF_TIMER2_CTC();
	} else {
		/*Handle callback error*/
	}
}

/*TIMER0 OVF*/
void __vector_5(void){
	G_PTRF_TIMER2_OVF();
}

