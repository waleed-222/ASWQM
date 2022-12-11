/*
 * TIMER2_reg.h
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_TIMER2_TIMER2_REG_H_
#define MCAL_TIMER2_TIMER2_REG_H_

#define TCCR2 *((volatile u8*)0x45)
#define TCNT2  *((volatile u8*)0x44)
#define OCR2  *((volatile u8*)0x43)
#define ASSR  *((volatile u8*)0x42)
#define TIMSK *((u8*)0x59)
#define TIFR *((volatile u8*)0x58)
#define SFIOR *((volatile u8*)0x50)

/*TCCR2*/
#define CS20       0
#define CS21       1
#define CS22       2
#define WGM21      3
#define COM20      4
#define COM21      5
#define WGM20      6
#define FOC2       7

/*ASSR*/
#define TCR2UB     0
#define OCR2UB     1
#define TCN2UB     2
#define AS2        3

/*TIMSK*/
#define TOIE2  6
#define OCIE2  7

/*TIFR*/
#define TOV2 6
#define OCF2 7

/*SFIOR*/
#define PSR2 1
#endif /* MCAL_TIMER2_TIMER2_REG_H_ */
