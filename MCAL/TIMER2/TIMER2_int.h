/*
 * TIMER2_int.h
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_TIMER2_TIMER2_INT_H_
#define MCAL_TIMER2_TIMER2_INT_H_

/*Initilization timer2:
 * 1.Enable Timer
 * 2.Set prescaler
 * 3.Timer Mode
 * 4.Set OC2 pin state*/
void TIMER2_vInit(void);

/*set preload to start count from the preload value*/
void TIMER2_vSetPreload(u8 A_u8PreloadVal);

/*set ocra to be the top counts to generate CTC flag*/
void TIMER2_vSetOcrVal(u8 A_u8OcrVal);

/*set callback to execute ISR related with OVR Event*/
void TIMER2_vCallBack_OVF(void(*Fptr)(void));

/*set callback to execute ISR related with CTC Event*/
void TIMER2_vCallBack_CTC(void(*Fptr)(void));

/*Turn On Timer0*/
void TIMER2_vTurnOn(void);

/*Turn Off Timer0*/
void TIMER2_vTurnOff(void);

/*make Delay*/
void TIMER2_vDelay(u8 A_u8TimeBySecond);

/*Get Duty cycle of wave on OC0 */
u8 TIMER2_u8DutyCycle();

/*Get frequency of wave on OC0 */
u32 TIMER2_u32FrequencyOfOC0Pin();

#endif /* MCAL_TIMER2_TIMER2_INT_H_ */
