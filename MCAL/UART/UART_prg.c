/*
 * UART_prg.c
 *
 *  Created on: Sep 21, 2022
 *      Author: mostafa_ebrahim
 */

#include"../../LIB/STD_TYPES.h"
#include"../../LIB/BIT_MATH.h"
#include "../DIO/DIO_int.h"
#include <stdarg.h>
#include "UART_reg.h"
#include "UART_pri.h"
#include "UART_cfg.h"
#include "UART_int.h"

/*************************************************************************************************/
/********************************Function: UART Initialization************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void UART_vInit(void)
{
	u8 L_u8UCSRC=0;
	SET_BIT(L_u8UCSRC,URSEL);
#if UART_SET_MODE==UART_ASYNC_MODE
	CLR_BIT(L_u8UCSRC,UMSEL);
#elif UART_SET_MODE==UART_SYNC_MODE
	SET_BIT(L_u8UCSRC,UMSEL);
#else
#warning "Invalid Mode of UART......"
#endif
#if UART_PARITY_MODE==UART_DISABLED_PARITY
	CLR_BIT(L_u8UCSRC,UPM0);
	CLR_BIT(L_u8UCSRC,UPM1);

#elif UART_PARITY_MODE==UART_EVEN_PARITY
	CLR_BIT(L_u8UCSRC,UPM0);
	SET_BIT(L_u8UCSSRC,UPM1);

#elif UART_PARITY_MODE== UART_ODD_PARITY
	SET_BIT(L_u8UCSSRC,UPM0);
	SET_BIT(L_u8UCSSRC,UPM1);
#else
#warning "Invalid Parity Mode of UART....."
#endif

#if	UART_STOP_BIT_SELECT==UART_ONE_STOP_BIT
	CLR_BIT(L_u8UCSRC,USBS);

#elif UART_STOP_BIT_SELECT==UART_TWO_STOP_BIT
	SET_BIT(L_u8UCSRC,USBS);

#else
#warning "Invalid Stop Bit Mode of UART....."
#endif

#if UART_CHAR_FRAME_SIZE == UART_5_BITS
	L_u8UCSRC&=UART_CHAR_SIZE_MASK;

#elif UART_CHAR_FRAME_SIZE==UART_6_BITS
	L_u8UCSRC&=UART_CHAR_SIZE_MASK;
	L_u8UCSRC|=UART_6_BITS;

#elif  UART_CHAR_FRAME_SIZE==UART_7_BITS
	L_u8UCSRC&=UART_CHAR_SIZE_MASK;
	L_u8UCSRC|=UART_7_BITS;

#elif  UART_CHAR_FRAME_SIZE==UART_8_BITS
	L_u8UCSRC&=UART_CHAR_SIZE_MASK;
	L_u8UCSRC|=UART_8_BITS;

#elif  UART_CHAR_FRAME_SIZE==UART_9_BITS
	L_u8UCSRC&=UART_CHAR_SIZE_MASK;
	L_u8UCSRC|=UART_8_BITS;
	SET_BIT(UCSRB,UCSZ2);


#else
#warning "Invalid Data Size Mode of UART....."
#endif

#if UART_CLK_PARITY== UART_CLK_PARITY_RISING_TX_FALIING_RX
	CLR_BIT(L_u8UCSRC,UCPOL);

#elif UART_CLK_PARITY==UART_CLK_PARITY_FALIING_TX_RISING_RX
	SET_BIT(L_u8UCSRC,UCPOL);

#else
#warning "Invalid Clock Polarity Mode of UART....."
#endif

	UCSRC=L_u8UCSRC;

#if  UART_RXC_INT_STATE==UART_INT_ENABLE
	SET_BIT(UCSRB,RXCIE);
#else
	CLR_BIT(UCSRB,RXCIE);
#endif

#if  UART_TXC_INT_STATE==UART_INT_ENABLE
	SET_BIT(UCSRB,TXCIE);
#else
	CLR_BIT(UCSRB,TXCIE);
#endif

#if UDRT_DRE_INT_STATE==UART_INT_ENABLE
	SET_BIT(UCSRB,UDRIE);
#else
	CLR_BIT(UCSRB,UDRIE);
#endif

	SET_BIT(UCSRB,RXEN);	/*Enable Receiver*/
	SET_BIT(UCSRB,TXEN);    /*Enable Transmitter*/

	UBBRL=51; /*equation BUD :BUD=f_cpu/(16*(UBR+1))*/

	DIO_vPinDir(PORTD_ID,PIN0_ID,DIR_INPUT); /*RX configure as Input*/
	DIO_vPinDir(PORTD_ID,PIN1_ID,DIR_OUTPUT);/*TX configure as Output*/
}


/*************************************************************************************************/
/********************************Function: Transmit Data******************************************/
/********************************Inputs:Data******************************************************/
/*************************************************************************************************/
void UART_vSendData(u8 A_u8Data)
{
	while(GET_BIT(UCSRA,UDRE)==0);
	UDR=A_u8Data;
}

/*************************************************************************************************/
/********************************Function: Receive Data*******************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
u8 UART_u8GetData(void)
{
	while (GET_BIT(UCSRA,RXC)==0);
	return UDR;
}

/*************************************************************************************************/
/********************************Function: Sending Data*******************************************/
/********************************Inputs:String(address of first element in array of char)*********/
/*************************************************************************************************/
void UART_vSend_String(u8* A_u8String)
{
	u8 L_u8CountChar=0;
	while(A_u8String[L_u8CountChar]!=ADDRESS_NULL)
	{
		UART_vSendData(A_u8String[L_u8CountChar]);
		L_u8CountChar++;
	}
	UART_vSendData('\0');

}

/*************************************************************************************************/
/********************************Function: Receive Data*******************************************/
/********************************Inputs:String(address of first element in array of char)*********/
/*************************************************************************************************/
void UART_vGet_String(u8* A_u8String)
{
	u8 L_u8CountChar=0;
	A_u8String[L_u8CountChar]=UART_u8GetData();
	while(A_u8String[L_u8CountChar]!=ADDRESS_NULL)
	{
		L_u8CountChar++;
		A_u8String[L_u8CountChar]=UART_u8GetData();
	}
	A_u8String[L_u8CountChar]='\0';
}
void Usart_SendNumber(u64 num) {
    u64 reversed = 0;
    u8 counter = 0;
    if (num < 0)
    {
    	UART_vSendData('-');
        num *= -1;
    }
    do
    {
        reversed = reversed * 10 + (num % 10);
        num = num / 10;
        counter++;
    } while (num != 0);
    do
    {
    	UART_vSendData(reversed % 10 + '0');
        reversed = reversed / 10;
        counter--;
    } while (counter != 0);
}

void Usart_Print(const u8 *str, ...) {
    u8 i = 0;
    va_list valist;
    va_start(valist, str);
    while (str[i] != '\0')
    {
        if (str[i] == '%') {

            i++;
            switch (str[i])
            {
            case 'c':
            	 UART_vSendData((u8)va_arg(valist, int));
                break;
            case 'd':
                Usart_SendNumber(va_arg(valist, int));
                break;
            case 'u':
                if (str[i+1] == 'l') {
                    i++;
                    Usart_SendNumber(va_arg(valist, unsigned long));
                }
                else {
                    Usart_SendNumber(va_arg(valist, unsigned int));
                }
                break;
            case 'l':
                Usart_SendNumber(va_arg(valist, long));
                break;


            default:
                if (str[i] == '\0') {
                    i--;
                }
                else {
                	UART_vSendData(str[i]);
                }
                break;
            }
        }
        else {
        	UART_vSendData(str[i]);
        }
        i++;
    }
    va_end(valist);
}

/*************************************************************************************************/
/********************************Function: CALL Back Function of RXC******************************/
/********************************Inputs:pointer to Application function***************************/
/*************************************************************************************************/
void UART_vCallBack_RXC(void(*Fptr)(void))
{
	G_PTRF_UART_RXC=Fptr;
}

/*************************************************************************************************/
/********************************Function: CALL Back Function of TXC******************************/
/********************************Inputs:pointer to Application function***************************/
/*************************************************************************************************/
void UART_vCallBack_TXC(void(*Fptr)(void))
{
	G_PTRF_UART_TXC=Fptr;
}

/*************************************************************************************************/
/********************************Function: CALL Back Function of UDRE*****************************/
/********************************Inputs:pointer to Application function***************************/
/*************************************************************************************************/
void UART_vCallBack_UDRE(void(*Fptr)(void))
{
	G_PTRF_UART_UDRE=Fptr;
}

/*************************************************************************************************/
/********************************Function: RXC ISR Function***************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/

void __vector_13 (void)
{
	G_PTRF_UART_RXC();
}

/*************************************************************************************************/
/********************************Function: UDRE ISR Function**************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void __vector_14 (void)
{
	G_PTRF_UART_UDRE();
}

/*************************************************************************************************/
/********************************Function: TXC ISR Function***************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void __vector_15 (void)
{
	G_PTRF_UART_TXC();
}





