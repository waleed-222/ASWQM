/*
 * TIMER_reg.h
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_TIMER0_TIMER0_REG_H_
#define MCAL_TIMER0_TIMER0_REG_H_

#define TCCR0 *((u8*)0x53)
#define TCNT0 *((volatile u8*)0x52)
#define OCR0  *((volatile u8*)0x5C)
#define TIMSK *((u8*)0x59)
#define TIFR *((volatile u8*)0x58)


/*TCCR0*/
#define CS00   0
#define CS01   1
#define CS02   2
#define WGM01  3
#define COM00  4
#define COM01  5
#define WGM00  6
#define FOC0   7

/*TIMSK*/
#define TOIE0  0
#define OCIE0  1


/*TIFR*/
#define TOV0 0
#define OCF0 1


#endif

