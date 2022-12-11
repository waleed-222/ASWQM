/*
 * I2C_int.h
 *
 *  Created on: Sep 26, 2022
 *      Author: hp
 */

#ifndef MCAL_I2C_I2C_INT_H_
#define MCAL_I2C_I2C_INT_H_

/*I2C Status Value*/
#define START_ACK                   0x08
#define REP__START_ACK              0x10
#define SLAVE_ADD_AND_WR_ACK        0x18
#define SLAVE_ADD_AND_RD_ACK        0x40
#define MASTER_WR_BYTE_ACK          0x28
#define MASTER_RD_BYTE_ACK          0x50
#define MASTER_RD_BYTE_NACK         0x58

static volatile u8 twi_state;
static volatile u8 twi_slarw;
static volatile u8 twi_sendStop;			// should the transaction end with a stop
static volatile u8 twi_inRepStart;			// in the middle of a repeated start

#ifndef TWI_BUFFER_LENGTH
 #define TWI_BUFFER_LENGTH 32
 #endif
static void (*twi_onSlaveTransmit)(void);
static void (*twi_onSlaveReceive)(u8*, int);

static u8 twi_masterBuffer[TWI_BUFFER_LENGTH];
static volatile u8 twi_masterBufferIndex;
static volatile u8 twi_masterBufferLength;

static u8 twi_txBuffer[TWI_BUFFER_LENGTH];
static volatile u8 twi_txBufferIndex;
static volatile u8 twi_txBufferLength;

static u8 twi_rxBuffer[TWI_BUFFER_LENGTH];
static volatile u8 twi_rxBufferIndex;

static volatile u8 twi_error;



  #define TWI_READY 0
  #define TWI_MRX   1
  #define TWI_MTX   2
  #define TWI_SRX   3
  #define TWI_STX   4

  #define false     0
  #define true 	    1

/*I2C Initialization*/
void I2C_vInit(void);

/*I2C Start Condition*/
void I2C_vStart(void);

/*I2C Stop Condition*/
void I2C_vStop(void);

/*I2C send Data*/
void I2C_vWrite(u8 A_u8data);

/*I2C Receive Data with Acknowledge*/
u8 I2C_u8Read_With_ACK(void);

/*I2C Receive Data with not Acknowledge*/
u8 I2C_u8Read_With_NACK(void);

/*I2C Read Current Status*/
u8 I2C_u8Get_Status(void);

u8 I2C_u8writeTo(u8 address, u8* data, u8 length, u8 wait, u8 sendStop);
u8 I2C_u8readFrom(u8 address, u8* data, u8 length, u8 sendStop);
void I2C_vSetAddress(u8 address);





#endif /* MCAL_I2C_I2C_INT_H_ */
