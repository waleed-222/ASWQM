	/*
 * main.c
 *
 *  Created on: Oct 27, 2022
 *      Author: hp
 */
#include "../LIB/STD_TYPES.h"
#include "../LIB/BIT_MATH.h"
# define F_CPU 8000000ul
#include "../MCAL/DIO/DIO_int.h"

#include "../HAL/PH_SENSOR/PH_SENSOR_int.h"
#include "../HAL/TDS_SENSOR/TDS_SENSOR_int.h"
#include  "../HAL/LCD/LCD_int.h"
#include <util/delay.h>
#include "MPU_PID/MPU_PID.h"

int main (void)
{
	LCD_vInit();
	TDS_SENSOR_vInit();
	LCD_vDispStr((u8*)"TDS=");
	//u8 L_u8Temp=0;
	u32 L_u16TDS=0;
	LCD_vDispNum(L_u16TDS);
	_delay_ms(1000);
	while(1)
	{
		L_u16TDS=TDS_SENSOR_u16GetReading();

		LCD_vCLR();
		LCD_vDispStr((u8*)"TDS=");
		LCD_vDispNum(L_u16TDS);
		LCD_vDispStr((u8*)"PPM");

		_delay_ms(1000);

	}


	return 0;
}
