/*
 * BIT_math.h
 *
 *  Created on: Aug 15, 2022
 *      Author: hp
 */

#ifndef LIB_BIT_MATH_H_
#define LIB_BIT_MATH_H_

/*
 * WRT_BIT is to assign 1 or 0 to  a specific bit in a register
 * while maintaining all the bits' value as it is.
 * Steps:
 * 		1- Clear the specific bit you want to write on
 * 		2- Insert the new value you want to write
 */
#define WRT_BIT(REG, BIT, VAL) ( (REG) = ( ( (REG) & (~(1<<(BIT))) ) | ( (VAL) << (BIT)  ) ) )

/*
 * SET_BIT is used to assign 1 to a specific bit in a register
 * while maintaining all other bits' value as it is.
 */
#define SET_BIT(REG, BIT)  ( (REG) |=  (1<<(BIT)) )

/*
 * CLR_BIT is used to assign 0 to a specific bit in a register
 * while maintaining all other bits' value as it is.
 */
#define CLR_BIT(REG, BIT)  ( (REG) &= (~(1<<(BIT))) )

/*
 * TOG_BIT is used to flip a specific bit in a register from 0 to 1 or from 1 to 0
 * while maintaining all other bits' value as it is.
 */
#define TOG_BIT(REG, BIT)  ( (REG) ^=  (1<<(BIT)) )

/*
 * GET_BIT is used to read the value of a specific bit in a register
 * while maintaining all the bits' value as it is.
 */
#define GET_BIT(REG, BIT)  ( ( (REG) >> (BIT) ) & 1 )


/*
 * IS_BIT_SET is used to check if the value of a specific bit in a register
 * is one .
 */
#define IS_BIT_SET(reg,bit) ((reg&(1<<bit))>>bit)

/*
 * IS_BIT_SCLR is used to check if the value of a specific bit in a register
 * is zero .
 */
#define IS_BIT_CLR(reg,bit) !((reg&(1<<bit))>>bit)

/*
 * ROR is used to shift the value of every bit in a register
 * to right.
 */

#define ROR(reg,num) reg=((reg<<REGISTER_SIZE-num)|(reg>>num))

/*
 * ROL is used to shift the value of every bit in a register
 * to left.
 */

#define ROL(reg,num) reg=((reg>>REGISTER_SIZE-num)|(reg<<num))



#endif /* LIB_BIT_MATH_H_ */
