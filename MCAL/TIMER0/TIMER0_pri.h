/*
 * Timer_pri.h
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_TIMER0_TIMER0_PRI_H_
#define MCAL_TIMER0_TIMER0_PRI_H_

/*Timer0 Modes*/
#define TIMER0_NORMAL_MODE 0
#define TIMER0_PWM_PHASE_CORRECT_MODE 64
#define TIMER0_CTC_MODE 8
#define TIMER0_FAST_PWM_MODE 72

/*Timer0 Modes Mask*/
#define TIMER0_MODE_MASK 183


/*Timer0 OCP0 Mode*/
#define TIMER0_OC0_DISCONNECTED 0
#define TIMER0_OC0_TOGGLE       1
#define TIMER0_OC0_CLR          2
#define TIMER0_OC0_SET          3



/*TIMER0 Clock Select*/
#define TIMER0_NO_CLK                 0  /*Timer/counter stop*/
#define TIMER0_NO_PRESCALER           1  /*No prescaler*/
#define TIMER0_8_PRESCALER            2
#define TIMER0_64_PRESCALER           3
#define TIMER0_256_PRESCALER          4
#define TIMER0_1024_PRESCALER         5
#define TIMER0_EXT_CLK_FALLING_EDGE   6
#define TIMER0_EXT_CLK_RISINGING_EDGE 7

/*Timer0 Clock Select Mask*/
#define TIMER0_CLK_SELECT_MASK 248

/*PWM MODE Options*/
#define TIMER0_PWM_NON_INVERTING 0
#define TIMER0_PWM_INVERTING     1

/*Max Counts of Timer0*/
#define TIMER0_MAX_COUNT 255

/*Timer0 INT State*/
#define TIMER0_INT_DISABLE 0
#define TIMER0_INT_ENABLE  1

/*ISR TIMER0*/
void __vector_10 (void) __attribute__((signal)); /* CTC TIMER0*/
void __vector_11 (void) __attribute__((signal)); /* OVF TIMER0*/

/*Pointer to function*/
 void (*G_PTRF_TIMER0_OVF)(void) = ADDRESS_NULL;
 void (*G_PTRF_TIMER0_CTC)(void) =ADDRESS_NULL;


#endif /* MCAL_TIMER0_TIMER0_PRI_H_ */
