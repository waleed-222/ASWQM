/*
 * SPI_int.h
 *
 *  Created on: Sep 24, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_SPI_SPI_INT_H_
#define MCAL_SPI_SPI_INT_H_

/*Initialization of SPI*/
void SPI_vInit(void);

/*Send and Receive Char*/
u8 SPI_u8Send_Receive_Char(u8 A_u8Char);

/*Send Char*/
void SPI_u8Send_Char(u8 A_u8Char);

/*Receive Char*/
u8 SPI_Receive_Char(void);

/*Send String*/
void SPI_u8Send_String(const u8 *Str);

/*Receive Data*/
void SPI_Receive_String(s8 *Str);

/*set callback to execute ISR related with STC Event*/
void SPI_vCallBack_STC(void(*Fptr)(void));

#endif /* MCAL_SPI_SPI_INT_H_ */
