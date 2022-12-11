/*
 * ADC_pri.h
 *
 *  Created on: Aug 31, 2022
 *      Author: hp
 */

#ifndef MCAL_ADC_ADC_PRI_H_
#define MCAL_ADC_ADC_PRI_H_

/*Prescaler Options*/
#define PRESCALER2   0
#define PRESCALER4   2
#define PRESCALER8   3
#define PRESCALER16  4
#define PRESCALER32  5
#define PRESCALER64  6
#define PRESCALER128 7

/*Prescaler Mask*/
#define PRESCALER_MASK_BITS 0xf8

/*vector number of ADC interrupt*/
void __vector_16 (void) __attribute__((signal));

/*Pointer to function*/
ptr_fun_IvOu16_t G_ISR_ADC=ADDRESS_NULL;



#endif /* MCAL_ADC_ADC_PRI_H_ */
