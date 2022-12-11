/*
 * TDS_SENSOR_prg.c
 *
 *  Created on: Oct 31, 2022
 *      Author: hp
 */
#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"

#include "../../MCAL/DIO/DIO_int.h"
#include "../../MCAL/ADC/ADC_int.h"

#include "TDS_SENSOR_pri.h"
#include "TDS_SENSOR_cfg.h"
#include "TDS_SENSOR_int.h"

#include <util/delay.h>

u8 G_u8TDS_Temperature=25;

/*************************************************************************************************/
/********************************Function: Initialization of TDS Sensor***************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
void TDS_SENSOR_vInit(void)
{
	/*Initialization of ADC*/
	ADC_vInt();
}

/*************************************************************************************************/
/********************************Function: Get Value of TDS***************************************/
/********************************Inputs:void******************************************************/
/*************************************************************************************************/
u32 TDS_SENSOR_u16GetReading(void)
{
	u32 analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
	u32 analogBufferTemp[SCOUNT];

	u32 copyIndex = 0;

	f32 averageVoltage = 0;
	u32 tdsValue = 0;
	u8 temperature = 25;       // current temperature for compensation



	_delay_ms(40);     //every 40 milliseconds,read the analog value from the ADC
	for(u8 analogBufferIndex=0;analogBufferIndex<SCOUNT;analogBufferIndex++ )
	{
		analogBuffer[analogBufferIndex] = ADC_u16GetDigValSync(TDS_SENSOR_ADC_CHANNEL);    //read the analog value and store into the buffer
		_delay_ms(40);     //every 40 milliseconds,read the analog value from the ADC
	}

	_delay_us(800);

for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
	analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

	// read the analog value more stable by the median filtering algorithm, and convert to voltage value
	averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)  5 / 1024.0;

	//temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
	f32 compensationCoefficient = 1.0+(float)0.02*(temperature-25.0);
	//temperature compensation
	f32 compensationVoltage=(float)averageVoltage/compensationCoefficient;

	//convert voltage value to tds value
	tdsValue=((float)133.42*compensationVoltage*compensationVoltage*compensationVoltage - (float)255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;



	return tdsValue;
}

// median filtering algorithm
u32 getMedianNum(u32 bArray[], u32 iFilterLen)
{
	u32 bTab[iFilterLen];
	for (u8 i = 0; i<iFilterLen; i++)
		bTab[i] = bArray[i];
	u32 i, j, bTemp;
	for (j = 0; j < iFilterLen - 1; j++) {
		for (i = 0; i < iFilterLen - j - 1; i++) {
			if (bTab[i] > bTab[i + 1]) {
				bTemp = bTab[i];
				bTab[i] = bTab[i + 1];
				bTab[i + 1] = bTemp;
			}
		}
	}
	if ((iFilterLen & 1) > 0){
		bTemp =(float) bTab[(iFilterLen - 1) / 2];
	}
	else {
		bTemp = (float)((float)bTab[iFilterLen / 2] + (float)bTab[iFilterLen / 2 - 1]) / 2;
	}
	return bTemp;
}
