/*
 * TIMER1_reg.h
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_TIMER1_TIMER1_REG_H_
#define MCAL_TIMER1_TIMER1_REG_H_

#define TCCR1A *((u8*)0x4F)
#define TCCR1B *((u8*)0x4E)
#define TCNT1 *((volatile u16*)0x4C)
#define OCR1A  *((volatile u16*)0x4A)
#define OCR1B  *((volatile u16*)0x48)
#define ICR1   *((volatile u16*)0x46)
#define TIMSK   *((u8*)0x59)
#define TIFR   *((volatile u8*)0x58)

/*TCCR1A*/
#define WGM10    0
#define WGM11    1
#define FOC1B    2
#define FOC1A    3
#define COM1B0   4
#define COM1B1   5
#define COM1A0   6
#define COM1A1   7

/*TCCR1B*/
#define CS10     0
#define CS11     1
#define CS12     2
#define WGM12    3
#define WGM13    4
#define ICES1    6
#define ICNC1    7
/*TIMSK*/
#define TOIE1  2
#define OCIE1B 3
#define OCIE1A 4
#define TICIE1 5

/*TIFR*/
#define TOV1 2
#define OCF1B 3
#define OCF1A 4
#define ICF1 5


#endif /* MCAL_TIMER1_TIMER1_REG_H_ */
