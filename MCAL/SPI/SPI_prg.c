/*
 * SPI_prg.c
 *
 *  Created on: Sep 24, 2022
 *      Author: mostafa_ebrahim
 */

#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"

#include "../DIO/DIO_int.h"

#include "SPI_reg.h"
#include "SPI_pri.h"
#include "SPI_cfg.h"
#include "SPI_int.h"

/*************************************************************************************************/
/********************************Function: Initialization of SPI**********************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void SPI_vInit(void)
{
#if SPI_MODE==SPI_MASTER
	/*Configuration of Master Pins*/
	DIO_vPinDir(SPI_SCK_PORT,SPI_SCK_PIN,DIR_OUTPUT);
	DIO_vPinDir(SPI_MOSI_PORT,SPI_MOSI_PIN,DIR_OUTPUT);
	DIO_vPinDir(SPI_MISO_PORT,SPI_MISO_PIN,DIR_INPUT);
	DIO_vPinDir(SPI_SS_PORT,SPI_SS_PIN,DIR_OUTPUT);

	/*Configure SPI as Master*/
	SET_BIT(SPCR,MSTR);
#elif SPI_MODE==SPI_SLAVE
	/*Configuration of Slave Pins*/
	DIO_vPinDir(SPI_SCK_PORT,SPI_SCK_PIN,DIR_INPUT);
	DIO_vPinDir(SPI_MOSI_PORT,SPI_MOSI_PIN,DIR_INPUT);
	DIO_vPinDir(SPI_MISO_PORT,SPI_MISO_PIN,DIR_OUTPUT);
	DIO_vPinDir(SPI_SS_PORT,SPI_SS_PIN,DIR_INPUT);

	/*Configure SPI as Slave*/
	CLR_BIT(SPCR,MSTR);
#else
#warning "Invalid SPI Mode...."
#endif

#if SPI_INT_STATE==SPI_INT_ENABLE
	/*Enable SPI Interrupt*/
	SET_BIT(SPCR,SPIE);
#elif SPI_INT_STATE==SPI_INT_DISABLE
	/*Disable SPI Interrupt*/
	CLR_BIT(SPCR,SPIE);
#else
#warning "Invalid SPI Interrupt......"
#endif

#if SPI_DORD_SELECT==SPI_DORD_LSB
	/*Data Order from LSB*/
	SET_BIT(SPCR,DORD);
#elif SPI_DORD_SELECT==SPI_DORD_MSB
	/*Data Order from MSB*/
	CLR_BIT(SPCR,DORD);
#else
#warning "Invalid SPI DORD..."
#endif

#if SPI_CLK_POLARITY_SELECT==SPI_CLK_POLARITY_LEADING_RISING_TRAILING_FALLING
	/*SPI Clock Polarity Leading on Raising and Trailing on Falling*/
	CLR_BIT(SPCR,CPOL);
#elif	SPI_CLK_POLARITY_SELECT==SPI_CLK_POLARITY_LEADING_FALLING_TRAILING_RISING
	/*SPI Clock Polarity Trailing on Raising and Leading on Falling*/
	SET_BIT(SPCR,CPOL);
#else
#warning "Invalid CLK Polarity......."
#endif

#if SPI_CLK_PHASE_SELECT==SPI_CLK_PHASE_LEADING_SETUP_TRAILING_SAMPLE
	/*SPI Clock Phase Setup on Leading and Sample on Trailing*/
	SET_BIT(SPCR,CPHA);
#elif SPI_CLK_PHASE_SELECT==SPI_CLK_PHASE_LEADING_SAMPLE_TRAILING_SETUP
	/*SPI Clock Phase Sample on Leading and Setup on Trailing*/
	CLR_BIT(SPCR,CPHA);
#else
#warning "Invalid CLK Phase ......"
#endif
	/*SPI Set Prescaler*/
	SPCR&=SPI_PRESCALER_MASK;
	SPCR|=SPI_PRESCALER_SELECT;

#if SPI2X_STATE==SPI2X_ENABLE
	/*Enable SPI Double Speed*/
	SET_BIT(SPSR,SPI2X);
#elif SPI2X_STATE==SPI2X_DISABLE
	/*Disable SPI Double Speed*/
	CLR_BIT(SPSR,SPI2X);
#else
#warning "Invalid SPI2X....."
#endif
	/*Enable SPI*/
	SET_BIT(SPCR,SPE);
}

/*************************************************************************************************/
/********************************Function: Send and Receive Data(Char)****************************/
/********************************Inputs:Data(Char)************************************************/
/*************************************************************************************************/
u8 SPI_u8Send_Receive_Char(u8 A_u8Char)
{
	/*Send Char*/
	SPDR=A_u8Char;
	/*wait on SPI Interrupt Flag is set to one*/
	while(GET_BIT(SPSR,7)==0);
	return SPDR;
}

/*************************************************************************************************/
/********************************Function: Send Data(Char)****************************************/
/********************************Inputs:Data(Char)************************************************/
/*************************************************************************************************/
void SPI_u8Send_Char(u8 A_u8Char)
{
	/*Send Data*/
	SPDR=A_u8Char;
	/*wait until SPI interrupt flag=1 (data is sent correctly)*/
	while(GET_BIT(SPSR,7)==0);
}

/*************************************************************************************************/
/********************************Function: Receive Data(Char)*************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
u8 SPI_Receive_Char(void)
{
	/*wait until SPI interrupt flag=1(data is receive correctly)*/
	while(GET_BIT(SPSR,7)==0);
	/*return the received byte from SPI data register*/
	return SPDR;
}

/*************************************************************************************************/
/********************************Function: Send String********************************************/
/********************************Inputs:Address of first char in array of char********************/
/*************************************************************************************************/
void SPI_u8Send_String(const u8 *Str)
{
	u8 i = 0;
	while(Str[i] != '\0')
	{
		SPI_u8Send_Char(Str[i]);
		i++;
	}
}

/*************************************************************************************************/
/********************************Function: Receive String*****************************************/
/********************************Inputs:Address of first char in array of char********************/
/*************************************************************************************************/
void SPI_Receive_String(s8 *Str)
{
	u8 i = 0;
	Str[i] = SPI_Receive_Char();
	while(Str[i] != '#')
	{
		i++;
		Str[i] = SPI_Receive_Char();
	}
	Str[i] = '\0';
}

/*************************************************************************************************/
/********************************Function: CALL Back Function of STC******************************/
/********************************Inputs:pointer to Application function***************************/
/*************************************************************************************************/
void SPI_vCallBack_STC(void(*Fptr)(void))
{
	G_PTRF_SPI_STC=Fptr;
}

/*************************************************************************************************/
/********************************Function: STC ISR Function***************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void __vector_12 (void)
{
	G_PTRF_SPI_STC();
}
