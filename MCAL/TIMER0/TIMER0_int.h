/*
 * TIMER_int.h
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_TIMER0_TIMER0_INT_H_
#define MCAL_TIMER0_TIMER0_INT_H_

/*Initilization timer0:
 * 1.Enable Timer
 * 2.Set prescaler
 * 3.Timer Mode
 * 4.Set OC0 pin state*/
void TIMER0_vInit(void);

/*set preload to start count from the preload value*/
void TIMER0_vSetPreload(u8 A_u8PreloadVal);

/*set ocra to be the top counts to generate CTC flag*/
void TIMER0_vSetOcrVal(u8 A_u8OcrVal);

/*set callback to execute ISR related with OVR Event*/
void TIMER0_vCallBack_OVF(void(*Fptr)(void));

/*set callback to execute ISR related with CTC Event*/
void TIMER0_vCallBack_CTC(void(*Fptr)(void));

/*Turn On Timer0*/
void TIMER0_vTurnOn(void);

/*Turn Off Timer0*/
void TIMER0_vTurnOff(void);

/*make Delay*/
void TIMER0_vDelay(u8 A_u8TimeBySecond);

/*Get Duty cycle of wave on OC0 */
u8 TIMER0_u8DutyCycle();

/*Get frequency of wave on OC0 */
u32 TIMER0_u32FrequencyOfOC0Pin();



#endif /* MCAL_TIMER0_TIMER0_INT_H_ */
