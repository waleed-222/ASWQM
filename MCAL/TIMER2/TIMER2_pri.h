/*
 * TIMER2_pri.h
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_TIMER2_TIMER2_PRI_H_
#define MCAL_TIMER2_TIMER2_PRI_H_


/*Timer2 Modes*/
#define TIMER2_NORMAL_MODE 0
#define TIMER2_PWM_PHASE_CORRECT_MODE 64
#define TIMER2_CTC_MODE 8
#define TIMER2_FAST_PWM_MODE 72

/*Timer2 Modes Mask*/
#define TIMER2_MODE_MASK 183


/*Timer2 OCP2 Mode*/
#define TIMER2_OC2_DISCONNECTED 0
#define TIMER2_OC2_TOGGLE       1
#define TIMER2_OC2_CLR          2
#define TIMER2_OC2_SET          3



/*TIMER2 Clock Select*/
#define TIMER2_NO_CLK                 0  /*Timer/counter stop*/
#define TIMER2_NO_PRESCALER           1  /*No prescaler*/
#define TIMER2_8_PRESCALER            2
#define TIMER2_32_PRESCALER           3
#define TIMER2_64_PRESCALER           4
#define TIMER2_128_PRESCALER          5
#define TIMER2_256_PRESCALER          6
#define TIMER2_1024_PRESCALER         7

/*Timer2 Clock Select Mask*/
#define TIMER2_CLK_SELECT_MASK 248

/*PWM MODE Options*/
#define TIMER2_PWM_NON_INVERTING 0
#define TIMER2_PWM_INVERTING     1

/*Max Counts of Timer2*/
#define TIMER2_MAX_COUNT 255

/*Timer0 INT State*/
#define TIMER2_INT_DISABLE 0
#define TIMER2_INT_ENABLE  1

/*ISR TIMER0*/
void __vector_4 (void) __attribute__((signal)); /* CTC TIMER2*/
void __vector_5 (void) __attribute__((signal)); /* OVF TIMER2*/

/*Pointer to function*/
 void (*G_PTRF_TIMER2_OVF)(void) = ADDRESS_NULL;
 void (*G_PTRF_TIMER2_CTC)(void) =ADDRESS_NULL;


#endif /* MCAL_TIMER2_TIMER2_PRI_H_ */
