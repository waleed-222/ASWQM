/*
 * GIE_int.h
 *
 *  Created on: Aug 29, 2022
 *      Author: hp
 */

#ifndef MCAL_GIE_GIE_INT_H_
#define MCAL_GIE_GIE_INT_H_

/*Interrupt Options*/
#define DISABLE 0
#define ENABLE 1

/*Enable Global Interrupt*/
void GIE_vEnableGlobalInterrupt(void);

/*Disable Global Interrupt*/
void GIE_vDisableGlobalInterrupt(void);



#endif /* MCAL_GIE_GIE_INT_H_ */
