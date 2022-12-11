/*
 * ADC_cfg.h
 *
 *  Created on: Aug 31, 2022
 *      Author: hp
 */

#ifndef MCAL_ADC_ADC_CFG_H_
#define MCAL_ADC_ADC_CFG_H_

/*options PRESCALER_MODE:
 PRESCALER2
 PRESCALER4
 PRESCALER8
 PRESCALER16
 PRESCALER32
 PRESCALER64
 PRESCALER128
 * */

#define PRESCALER_MODE PRESCALER128

/*ADC Channels Port*/
#define ADC_PORT PORTA_ID

/*ADC Channels Pin*/
#define ADC0_PIN PIN0_ID
#define ADC1_PIN PIN1_ID
#define ADC2_PIN PIN2_ID
#define ADC3_PIN PIN3_ID
#define ADC4_PIN PIN4_ID
#define ADC5_PIN PIN5_ID
#define ADC6_PIN PIN6_ID
#define ADC7_PIN PIN7_ID

#endif /* MCAL_ADC_ADC_CFG_H_ */
