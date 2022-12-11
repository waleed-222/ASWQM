/*
 * TIMER1_int.h
 *
 *  Created on: Sep 13, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_TIMER1_TIMER1_INT_H_
#define MCAL_TIMER1_TIMER1_INT_H_

#define TIMER1_ICU_FALLING_EDGE 0
#define TIMER1_ICU_RISING_EDGE  1

void TIMER1_vInit(void);

/*set preload to start count from the preload value*/
void TIMER1_vSetPreload(u8 A_u8PreloadVal);

/*Set ICR1 to be the top counts*/
void TIMER1_vSetICR1(u16 A_u16ICR);

/*set OCR1A to be the top counts to generate CTC flag*/
void TIMER1_vSetOcrA1Val(u16 A_u16Ocr1aVal);

/*set OCR1B to be the top counts to generate CTC flag*/
void TIMER1_vSetOcrB1Val(u16 A_u16Ocr1bVal);

/*set callback to execute ISR related with OVR Event*/
void TIMER1_vCallBack_OVF(void(*Fptr)(void));

/*set callback to execute ISR related with CTCA Event*/
void TIMER1_vCallBack_CTCA(void(*Fptr)(void));

/*set callback to execute ISR related with CTCB Event*/
void TIMER1_vCallBack_CTCB(void(*Fptr)(void));

/*set callback to execute ISR related with Capture Event*/
void TIMER1_vCallBack_CE(void(*Fptr)(void));

/*Turn On Timer1*/
void TIMER1_vTurnOn(void);

/*Turn Off Timer1*/
void TIMER1_vTurnOff(void);

/*make Delay*/
void TIMER1_vDelay(u8 A_u8TimeBySecond);

/*Get Duty cycle of wave on OC1A */
u8 TIMER1_u8DutyCycleOc1A();

/*Get Duty cycle of wave on OC1B */
u8 TIMER1_u8DutyCycleOc1B();

/*Get frequency of wave on OC1A */
u32 TIMER1_u32FrequencyOfOC1APin();

/*Get frequency of wave on OC1B */
u32 TIMER1_u32FrequencyOfOC1BPin();

/*Read ICR1*/
u16 TIMER1_u16ReadICR1();

/*choose trigger*/
void TIMER1_vSetTrigger(u8 A_u8Trigger);

/*Disable Inerrupt of ICU*/
void TIMER1_vDisableIcuInt();

/*Initialization of Input capture unit*/
void Icu_Init();
void Icu_SetEdge(u8 A_u8Edge);



#endif /* MCAL_TIMER1_TIMER1_INT_H_ */
