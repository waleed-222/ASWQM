/*
 * I2C_prg.c
 *
 *  Created on: Sep 26, 2022
 *      Author: hp
 */
#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"
#include"../DIO/DIO_int.h"
#include "I2C_reg.h"
#include "I2C_pri.h"
#include "I2C_int.h"
#include <util/twi.h>



/*************************************************************************************************/
/********************************Function: Initialization of I2C**********************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void I2C_vInit(void)
{
    /* Bit Rate: 400.000 kbps using zero pre-scaler TWPS=00 and F_CPU=8Mhz */
    TWBR = 0x02;
	TWSR = 0x00;

    /* Two Wire Bus address my address if any master device want to call me: 0x1 (used in case this MC is a slave device)
       General Call Recognition: Off */
    TWAR = 0b00000010; /* my address = 0x01 */

    TWCR = (1<<TWEN); /* enable TWI */
}

/*************************************************************************************************/
/********************************Function: Start Condition of I2C*********************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void I2C_vStart(void)
{
    /*
	 * Clear the TWINT flag before sending the start bit TWINT=1
	 * send the start bit by TWSTA=1
	 * Enable TWI Module TWEN=1
	 */
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    /* Wait for TWINT flag set in TWCR Register (start bit is send successfully) */
    while(GET_BIT(TWCR,TWINT)==0);
}

/*************************************************************************************************/
/********************************Function: Stop Condition of I2C**********************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void I2C_vStop(void)
{
    /*
	 * Clear the TWINT flag before sending the stop bit TWINT=1
	 * send the stop bit by TWSTO=1
	 * Enable TWI Module TWEN=1
	 */
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

/*************************************************************************************************/
/********************************Function: Send Data (Char)***************************************/
/********************************Inputs:Data(Char)************************************************/
/*************************************************************************************************/
void I2C_vWrite(u8 A_u8data)
{
    /* Put data On TWI data Register */
    TWDR = A_u8data;
    /*
	 * Clear the TWINT flag before sending the data TWINT=1
	 * Enable TWI Module TWEN=1
	 */
    TWCR = (1 << TWINT) | (1 << TWEN);
    /* Wait for TWINT flag set in TWCR Register(data is send successfully) */
    while(GET_BIT(TWCR,TWINT)==0);
}

/*************************************************************************************************/
/********************************Function: Receive Data with Acknowledge**************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
u8 I2C_u8Read_With_ACK(void)
{
	/*
	 * Clear the TWINT flag before reading the data TWINT=1
	 * Enable sending ACK after reading or receiving data TWEA=1
	 * Enable TWI Module TWEN=1
	 */
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    /* Wait for TWINT flag set in TWCR Register (data received successfully) */
    while(GET_BIT(TWCR,TWINT)==0);
    /* Read Data */
    return TWDR;
}

/*************************************************************************************************/
/********************************Function: Receive Data with Not Acknowledge**********************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
u8 I2C_u8Read_With_NACK(void)
{
	/*
	 * Clear the TWINT flag before reading the data TWINT=1
	 * Enable TWI Module TWEN=1
	 */
    TWCR = (1 << TWINT) | (1 << TWEN);
    /* Wait for TWINT flag set in TWCR Register (data received successfully) */
    while(GET_BIT(TWCR,TWINT)==0);
    /* Read Data */
    return TWDR;
}

/*************************************************************************************************/
/********************************Function: Read Current Status************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
u8 I2C_u8Get_Status(void)
{
    u8 status;
    /* masking to eliminate first 3 bits and get the last 5 bits (status bits) */
    status = TWSR & 0xF8;
    return status;
}

void I2C_vSetAddress(u8 address)
{
  // set twi slave address (skip over TWGCE bit)
  TWAR = address << 1;
}

u8 I2C_u8readFrom(u8 address, u8* data, u8 length, u8 sendStop)
{
  	u8 i;
#ifdef TWI_DEBUG
  	UART_Printf("In twi_readFrom(%x, %x, %x, %x)\n\r" , address , data , length , sendStop);
#endif
  	// ensure data will fit into buffer
  	if(TWI_BUFFER_LENGTH < length)
	{
#ifdef TWI_DEBUG
  		UART_Printf("Buffer_Length (0x%x) < length (0x%x) -> Wont fit...\n\r" , TWI_BUFFER_LENGTH , length);
#endif
    		return 0;
  	}

#ifdef TWI_DEBUG
  	UART_Printf("wait until twi is ready, become master receiver");
#endif
  // wait until twi is ready, become master receiver
  	while(TWI_READY != twi_state)
  	{
#ifdef TWI_DEBUG
  		UART_Printf(".");
#endif
    		continue;
  	}
#ifdef TWI_DEBUG
  		UART_Printf("DONE!\n\r");
#endif
  	twi_state = TWI_MRX;
  	twi_sendStop = sendStop;
  	// reset error state (0xFF.. no error occured)
  	twi_error = 0xFF;

  	// initialize buffer iteration vars
  	twi_masterBufferIndex = 0;
  	twi_masterBufferLength = length-1;  // This is not intuitive, read on...
  	// On receive, the previously configured ACK/NACK setting is transmitted in
  	// response to the received byte before the interrupt is signalled.
  	// Therefor we must actually set NACK when the _next_ to last byte is
  	// received, causing that NACK to be sent in response to receiving the last
  	// expected byte of data.

  	// build sla+w, slave device address + w bit
	twi_slarw = 0;
  	twi_slarw = TW_READ;
  	twi_slarw |= address << 1;

#ifdef TWI_DEBUG
  	UART_Printf("Will read from ddress %x\n\r" , twi_slarw);
#endif

  	if (true == twi_inRepStart)
	{
    		// if we're in the repeated start state, then we've already sent the start,
    		// (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
    		// We need to remove ourselves from the repeated start state before we enable interrupts,
    		// since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
    		// up. Also, don't enable the START interrupt. There may be one pending from the
    		// repeated start that we sent outselves, and that would really confuse things.
    		twi_inRepStart = false;			// remember, we're dealing with an ASYNC ISR
    		TWDR = twi_slarw;
    		TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);	// enable INTs, but not START
  	}
  	else
	{
    		// send start condition
    		TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
	}
#ifdef TWI_DEBUG
  	UART_Printf("Waiting for read operation to complete");
#endif

  	// wait for read operation to complete
  	while(TWI_MRX == twi_state)
	{
#ifdef TWI_DEBUG
  		UART_Printf(".");
#endif
    		continue;
  	}
#ifdef TWI_DEBUG
  		UART_Printf("DONE!\n\r");
#endif

  	if (twi_masterBufferIndex < length)
  	{
    		length = twi_masterBufferIndex;	//If more data was requested than was available set length to received cnt
	}

  	// copy twi buffer to data
  	for(u8 i = 0; i < length; ++i)
	{
    		data[i] = twi_masterBuffer[i]; //fill received data in the buffer
  	}

  	return length; //Return the cnt of received bytes
}

u8 I2C_u8writeTo(u8 address, u8* data, u8 length, u8 wait, u8 sendStop)
{
  u8 i;
#ifdef TWI_DEBUG
  	UART_Printf("In twi_writeTo(%x, %x, %x, %x, %x)\n\r" , address , data , length , wait , sendStop);
#endif
  	// ensure data will fit into buffer
  	if(TWI_BUFFER_LENGTH < length)
	{
#ifdef TWI_DEBUG
  		UART_Printf("Buffer_Length (0x%x) < length (0x%x) -> Wont fit...\n\r" , TWI_BUFFER_LENGTH , length);
#endif
    		return 1;
  	}

#ifdef TWI_DEBUG
  	UART_Printf("Wait until twi is ready, become master transmitter");
#endif
  	// wait until twi is ready, become master transmitter
  	while(TWI_READY != twi_state)
  	{
#ifdef TWI_DEBUG
  		UART_Printf(".");
#endif
    		continue;
  	}
#ifdef TWI_DEBUG
  	UART_Printf("DONE!\n\r");
#endif

  	twi_state = TWI_MTX;
  	twi_sendStop = sendStop;
  	// reset error state (0xFF.. no error occured)
  	twi_error = 0xFF;

  	// initialize buffer iteration vars
  	twi_masterBufferIndex = 0;
  	twi_masterBufferLength = length;

  	// copy data to twi buffer
  	for(i = 0; i < length; ++i)
	{
    		twi_masterBuffer[i] = data[i];
#ifdef TWI_DEBUG
  		UART_Printf("Send-Buffer[%x]: %x\n\r", i, twi_masterBuffer[i]);
#endif
  	}

  	// build sla+w, slave device address + w bit
	twi_slarw = 0;
  	twi_slarw = TW_WRITE;
  	twi_slarw |= address << 1;

#ifdef TWI_DEBUG
  	UART_Printf("Will write to address %x\n\r" , twi_slarw);
#endif

  	// if we're in a repeated start, then we've already sent the START
  	// in the ISR. Don't do it again.
  	//
  	if (true == twi_inRepStart)
	{
    		// if we're in the repeated start state, then we've already sent the start,
    		// (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
    		// We need to remove ourselves from the repeated start state before we enable interrupts,
    		// since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
    		// up. Also, don't enable the START interrupt. There may be one pending from the
    		// repeated start that we sent outselves, and that would really confuse things.
    		twi_inRepStart = false;			// remember, we're dealing with an ASYNC ISR
    		TWDR = twi_slarw;
    		TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);	// enable INTs, but not START
  	}
  	else
	{
    		// send start condition
    		TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA);	// enable INTs
	}

#ifdef TWI_DEBUG
  	UART_Printf("Wait for write operation to complete");
#endif
  	// wait for write operation to complete
  	while(wait && (TWI_MTX == twi_state))
	{
#ifdef TWI_DEBUG
  		UART_Printf(".");
#endif
    		continue;
  	}
#ifdef TWI_DEBUG
  	UART_Printf("DONE!\n\r");
#endif

  	if (twi_error == 0xFF)
    	{
		return 0;	// success
	}
  	else if (twi_error == TW_MT_SLA_NACK)
    	{
		return 2;	// error: address send, nack received
	}
  	else if (twi_error == TW_MT_DATA_NACK)
    	{
		return 3;	// error: data send, nack received
	}
  	else
	{
    		return 4;	// other twi error
	}
}



