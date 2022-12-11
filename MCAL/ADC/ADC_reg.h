/*
 * ADC_reg.h
 *
 *  Created on: Aug 31, 2022
 *      Author: hp
 */

#ifndef MCAL_ADC_ADC_REG_H_
#define MCAL_ADC_ADC_REG_H_

#define ADCSRA *((volatile u8*)0x26)
#define ADMUX  *((u8*)0x27)
#define ADCW  *((volatile u16*)0x24)
/*#define ADCL *((volatile u8*)0x24)
#define ADCH *((volatile u8*)0x25)*/
#define SFIOR	*((u8*)0x50)

/*ADCSRA*/
#define ADPS0   0
#define ADPS1   1
#define ADPS2   2
#define ADIE    3
#define ADIF    4
#define ADATE   5
#define ADSC    6
#define ADEN    7

/*ADMUX*/
#define MUX0  0
#define MUX1  1
#define MUX2  2
#define MUX3  3
#define MUX4  4
#define ADLAR 5
#define REFS0 6
#define REFS1 7



/*SFIOR*/

#define ADTS0 5
#define ADTS1 6
#define ADTS2 7

#endif /* MCAL_ADC_ADC_REG_H_ */
