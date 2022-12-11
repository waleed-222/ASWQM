/*
 * TIMER1_pri.h
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_TIMER1_TIMER1_PRI_H_
#define MCAL_TIMER1_TIMER1_PRI_H_

/*Timer1 Modes*/
#define TIMER1_NORMAL_MODE     0
#define TIMER1_CTC_OCR1A       4
#define TIMER1_FAST_PWM_ICR1   14


/*Timer1 OCP1 Mode*/
#define TIMER1_OC1_DISCONNECTED 0
#define TIMER1_OC1_TOGGLE       1
#define TIMER1_OC1_CLR          2
#define TIMER1_OC1_SET          3

/*Timer1 Clock Select Mask*/
#define TIMER1_CLK_SELECT_MASK 248

/*TIMER0 Clock Select*/
#define TIMER1_NO_CLK                 0  /*Timer/counter stop*/
#define TIMER1_NO_PRESCALER           1  /*No prescaler*/
#define TIMER1_8_PRESCALER            2
#define TIMER1_64_PRESCALER           3
#define TIMER1_256_PRESCALER          4
#define TIMER1_1024_PRESCALER         5
#define TIMER1_EXT_CLK_FALLING_EDGE   6
#define TIMER1_EXT_CLK_RISINGING_EDGE 7

/*PWM MODE Options*/
#define TIMER1_PWM_NON_INVERTING 0
#define TIMER1_PWM_INVERTING     1

/*Max Counts of Timer0*/
#define TIMER1_MAX_COUNT 65536ul

/*Timer1 INT State*/
#define TIMER1_INT_DISABLE 0
#define TIMER1_INT_ENABLE  1

/*Timer1 Input Capture Unit*/
#define  TIMER1_ICU_DISABLE 0
#define TIMER1_ICU_ENABLE   1


/*ISR TIMER0*/
void __vector_6 (void) __attribute__((signal)); /* Capture Event Timer1*/
void __vector_7 (void) __attribute__((signal)); /* Compare Match A Timer1*/
void __vector_8 (void) __attribute__((signal)); /* Compare Match B Timer1*/
void __vector_9 (void) __attribute__((signal)); /* OVF TIMER1*/

/*Pointer to function*/
void (*G_PTRF_TIMER1_CE)(void) =ADDRESS_NULL;
 void (*G_PTRF_TIMER1_OVF)(void) = ADDRESS_NULL;
 void (*G_PTRF_TIMER1_CTCA)(void) =ADDRESS_NULL;
 void (*G_PTRF_TIMER1_CTCB)(void) =ADDRESS_NULL;



#endif /* MCAL_TIMER1_TIMER1_PRI_H_ */
