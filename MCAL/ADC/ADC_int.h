/*
 * ADDC_int.h
 *
 *  Created on: Aug 31, 2022
 *      Author: hp
 */

#ifndef MCAL_ADC_ADC_INT_H_
#define MCAL_ADC_ADC_INT_H_

/*ADC Channels*/
#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3
#define ADC4 4
#define ADC5 5
#define ADC6 6
#define ADC7 7

/*Initialization of ADC*/
void ADC_vInt(void);

/*Read Value by ADC in Synchronous way*/
u16 ADC_u16GetDigValSync(u8 A_u8ChannelNum);

/*Read Value by ADC in Asynchronous way*/
void ADC_u16GetDigValAsync(ptr_fun_IvOu16_t ptr_func,u8 A_u8ChannelNum);







#endif /* MCAL_ADC_ADC_INT_H_ */
