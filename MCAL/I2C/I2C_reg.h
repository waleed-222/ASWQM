/*
 * I2C_reg.h
 *
 *  Created on: Sep 26, 2022
 *      Author: hp
 */

#ifndef MCAL_I2C_I2C_REG_H_
#define MCAL_I2C_I2C_REG_H_

#define TWBR *((volatile u8*)0x20)
#define TWCR *((volatile u8*)0x56)
#define TWSR *((volatile u8*)0x21)
#define TWDR *((volatile u8*)0x23)
#define TWAR *((volatile u8*)0x22)


/*TWCR*/
#define TWIE       0
#define TWEN       2
#define TWWC       3
#define TWSTO      4
#define TWSTA      5
#define TWEA       6
#define TWINT      7

/*TWSR*/
#define TWPS0      0
#define TWPS1      1
#define TWS3       3
#define TWS4       4
#define TWS5       5
#define TWS6       6
#define TWS7       7


#endif /* MCAL_I2C_I2C_REG_H_ */
