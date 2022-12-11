/*
 * SPI_pri.h
 *
 *  Created on: Sep 24, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_SPI_SPI_PRI_H_
#define MCAL_SPI_SPI_PRI_H_

/*SPI INT Options*/
#define SPI_INT_DISABLE 0
#define SPI_INT_ENABLE 1

/*SPI_MODE*/
#define SPI_SLAVE 0
#define SPI_MASTER 1

/*SPI Data Order*/
#define SPI_DORD_MSB 0
#define SPI_DORD_LSB 1

/*SPI Clock Polarity*/
#define SPI_CLK_POLARITY_LEADING_RISING_TRAILING_FALLING 0
#define SPI_CLK_POLARITY_LEADING_FALLING_TRAILING_RISING 1

/*SPI Clock Phase*/
#define SPI_CLK_PHASE_LEADING_SAMPLE_TRAILING_SETUP 0
#define SPI_CLK_PHASE_LEADING_SETUP_TRAILING_SAMPLE 1

/*SPI Prescaler*/
#define SPI_4_PRESCALER          0
#define SPI_16_PRESCALER         1
#define SPI_64_PRESCALER         2
#define SPI_128_PRESCALER        3

/*SPI Mask of Prescaler*/
#define SPI_PRESCALER_MASK   252

/*SPI Double Speed Bit*/
#define SPI2X_DISABLE 0
#define SPI2X_ENABLE  1


/*ISR UART*/
void __vector_12 (void) __attribute__((signal)); /*STC SPI*/

/*Pointer to function*/
void (*G_PTRF_SPI_STC)(void) =ADDRESS_NULL;
#endif /* MCAL_SPI_SPI_PRI_H_ */
