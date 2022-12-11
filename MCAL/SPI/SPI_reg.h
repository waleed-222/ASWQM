/*
 * SPI_reg.h
 *
 *  Created on: Sep 24, 2022
 *      Author: mostafa_ebrahim
 */

#ifndef MCAL_SPI_SPI_REG_H_
#define MCAL_SPI_SPI_REG_H_

#define SPCR *((u8*)0x2D)
#define SPSR *((volatile u8*)0x2E)
#define SPDR *((volatile u8*)0x2F)


/*SPCR*/
#define SPR0     0
#define SPR1     1
#define CPHA     2
#define CPOL     3
#define MSTR     4
#define DORD     5
#define SPE      6
#define SPIE     7


/*SPSR*/
#define SPI2X 0
#define WCOL  6
#define SPIF  7


#endif /* MCAL_SPI_SPI_REG_H_ */
