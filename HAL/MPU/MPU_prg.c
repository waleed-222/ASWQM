/*
 * MPU_prg.c
 *
 *  Created on: Dec 1, 2022
 *      Author: hp
 */
#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"

#include "../../MCAL/DIO/DIO_int.h"
#include "../../MCAL/UART/UART_int.h"
#include "../../MCAL/I2C/I2C_int.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "MPU_reg.h"
#include "MPU_pri.h"
#include "MPU_cfg.h"
#include "MPU_int.h"



#include <util/delay.h>

u8 Gscale = GFS_250DPS;
u8 Ascale = AFS_2G;

u8 Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution

f32 beta = 0.8660254 * GyroMeasError; //sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
f32 zeta = 0.8660254 * GyroMeasDrift; //sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

u8 Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

f32 deltat = 0, sum = 0;        // integration interval for both filter schemes

f32 gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer

f32 magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias

f32 magBias[3] = {0 ,0 ,0}, magScale[3] = {0, 0, 0};


void mpu9250_setup()
{
	//  TWBR = 12;  // 400 kbit/sec I2C speed
	// Set up the interrupt pin, its set as active high, push-pull
	//int intPin = 12; -> PIN12 = Pin B4
	//pinMode(intPin, INPUT);
	DIO_vPinDir(PORTB_ID,PIN4_ID,DIR_OUTPUT);
	//digitalWrite(intPin, LOW);
	DIO_vSetPinVal(PORTB_ID,PIN4_ID,VAL_LOW);
	//pinMode(myLed, OUTPUT);
	//int myLed = 13; -> Pin13 = Pin B5
	DIO_vPinDir(PORTB_ID,PIN5_ID,DIR_OUTPUT);
	//digitalWrite(myLed, HIGH);
	DIO_vSetPinVal(PORTB_ID,PIN5_ID,VAL_HIGH);

	UART_vSend_String((u8*)"MPU9250\n\r");
	UART_vSend_String((u8*)"9-DOF 16-bit\n\r");
	UART_vSend_String((u8*)"motion sensor\n\r");
	UART_vSend_String((u8*)"60 ug LSB\n\r");
	_delay_ms(800);

	// Read the WHO_AM_I register, this is a good test of communication
	//readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	u8 whoami = 0;

	whoami = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
	UART_vSend_String((u8*)"MPU9250\n\r I AM  ");
	UART_vSendData(whoami);
	UART_vSend_String((u8*)"I should be 0x71\n\r");
	////  display.setCursor(20,0);
	//  UART_vSend_String((u8*)"MPU9250");
	////  display.setCursor(0,10);
	//  UART_vSend_String((u8*)"I AM");
	////  display.setCursor(0,20);
	//  UART_vSend_String((u8*)c, HEX);
	////  display.setCursor(0,30);
	//  UART_vSend_String((u8*)"I Should Be");
	////  display.setCursor(0,40);
	//  UART_vSend_String((u8*)0x71, HEX);
	//  display.display();
	_delay_ms(800);

	if (whoami == 0x71) // WHO_AM_I should always be 0x68
	{
		UART_vSend_String((u8*)"MPU9250 is online...\n\r");

		MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
		UART_vSend_String((u8*)"x-axis self test: acceleration trim within : ");
		UART_vSendData(SelfTest[0]);
		UART_vSend_String((u8*)"of factory value\n\r");
		UART_vSend_String((u8*)"y-axis self test: acceleration trim within : ");
		UART_vSendData(SelfTest[1]);
		UART_vSend_String((u8*)"of factory value\n\r");
		UART_vSend_String((u8*)"z-axis self test: acceleration trim within : ");
		UART_vSendData(SelfTest[2]);
		UART_vSend_String((u8*)"of factory value\n\r");
		UART_vSend_String((u8*)"x-axis self test: gyration trim within : ");
		UART_vSendData(SelfTest[3]);
		UART_vSend_String((u8*)"of factory value\n\r");
		UART_vSend_String((u8*)"y-axis self test: gyration trim within : ");
		UART_vSendData(SelfTest[4]);
		UART_vSend_String((u8*)"of factory value\n\r");
		UART_vSend_String((u8*)"z-axis self test: gyration trim within : ");
		UART_vSendData(SelfTest[5]);
		UART_vSend_String((u8*)"of factory value\n\r");

		calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

		UART_vSend_String((u8*)"MPU9250 bias\n\r");

		UART_vSend_String((u8*)" x   y   z\n\r");

UART_vSendData((u8)1000*accelBias[0]);
UART_vSendData((u8)1000*accelBias[1]);
UART_vSendData((u8)1000*accelBias[2]);

		UART_vSend_String((u8*)"mg\n\r" );

		UART_vSendData((u8)gyroBias[0]);
		UART_vSendData((u8)gyroBias[1]);
		UART_vSendData((u8)gyroBias[2]);

		UART_vSend_String((u8*)"°/s\n\r" );
		_delay_ms(1000);

		initMPU9250();
		UART_vSend_String((u8*)"MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature


		u8 BypassTrue = 0;

		while(BypassTrue == 0)
		{
			writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
			writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
			u8 PinCFG = readByte(MPU9250_ADDRESS, INT_PIN_CFG);	//0x22);
			u8 MasterDis = readByte(MPU9250_ADDRESS, I2C_MST_CTRL); //0x00); // Disable I2C master
			u8 IntEna = readByte(MPU9250_ADDRESS, INT_ENABLE); //0x01); // Disable I2C master
			whoami = 0;
			whoami = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
			UART_vSend_String((u8*)"PinCFG: ");
			UART_vSendData((u8)PinCFG);
			UART_vSend_String((u8*)"= 0x22 MasterDisable");
			UART_vSendData((u8)MasterDis);
			UART_vSend_String((u8*)"= 0x00 Interrupts");
			UART_vSendData((u8)IntEna);
			UART_vSend_String((u8*)"= 0x01 Whoami");
			UART_vSendData((u8)whoami);
			UART_vSend_String((u8*)"= 0x71\n\r");

			if(PinCFG == 0x22 && MasterDis == 0x00 && IntEna == 0x01)
			{
				BypassTrue = 1;
			}
			_delay_ms(800);
		}

		// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
		whoami = readByte_Debug(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
		UART_vSend_String((u8*)"AK8963\n\rI AM ");
		UART_vSendData((u8)whoami);
		UART_vSend_String((u8*)"I should be 0x48\n\r");

		if(whoami == 0x48)
		{

			_delay_ms(1000);

			// Get magnetometer calibration from AK8963 ROM
			initAK8963(magCalibration);
			UART_vSend_String((u8*)"AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
			getMres();
			magcalMPU9250(magBias,magScale);

#ifdef SerialDebug
			UART_vSend_String((u8*)"Calibration values:\n\r");
			UART_vSend_String((u8*)"X-Axis sensitivity adjustment value");
			UART_vSendData( magCalibration[0]);
			UART_vSend_String((u8*)"\nY-Axis sensitivity adjustment value ");
			UART_vSendData( magCalibration[1]);
			UART_vSend_String((u8*)"\nZ-Axis sensitivity adjustment value ");
			UART_vSendData( magCalibration[2]);
#endif

			UART_vSend_String((u8*)"ASAX ");
			UART_vSendData( magCalibration[0]);
			UART_vSend_String((u8*)"\nASAY %d\n");
			UART_vSendData( magCalibration[1]);
			UART_vSend_String((u8*)"\nASAZ ");
			UART_vSendData( magCalibration[2]);
			_delay_ms(1000);
		}
		else
		{
			UART_vSend_String((u8*)"\nCould not connect to AK8963: 0x");
			UART_vSendData((u8)whoami);
		}

		whoami = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
		UART_vSend_String((u8*)"MPU9250\n\r I AM ");
		UART_vSendData((u8)whoami);
		UART_vSend_String((u8*)"I should be 0x71\n\r");

	}
	else
	{
		UART_vSend_String((u8*)"Could not connect to MPU9250: 0x");
		UART_vSendData((u8)whoami);
	}
	UART_vSend_String((u8*)"Init done!\n\r");
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getMres()
{
	switch (Mscale)
	{
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mRes = 10.*4912./8190.; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
		break;
	}
}

void getGres()
{
	switch (Gscale)
	{
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		gRes = 250.0/32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0/32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0/32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0/32768.0;
		break;
	}
}

void getAres()
{
	switch (Ascale)
	{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		aRes = 2.0/32768.0;
		break;
	case AFS_4G:
		aRes = 4.0/32768.0;
		break;
	case AFS_8G:
		aRes = 8.0/32768.0;
		break;
	case AFS_16G:
		aRes = 16.0/32768.0;
		break;
	}
}


void readAccelData(u16 * destination)
{
	u8 rawData[6];  // x/y/z accel register data stored here
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = ((u16)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((u16)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((u16)rawData[4] << 8) | rawData[5] ;
}

void readGyroData(u16 * destination)
{
	u8 rawData[6];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((u16)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((u16)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((u16)rawData[4] << 8) | rawData[5] ;
}

void readMagData(u16 * destination)
{
	u8 rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	//if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
	while(!(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01))
	{
#ifdef byteWrite_DEBUG
		UART_vSend_String((u8*)"End of readBytes()\n\r");
#endif
	}

	//{ // wait for magnetometer data ready bit to be set
	readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array

	u8 c = rawData[6]; // End data read by reading ST2 register
	if(!(c & 0x08))
	{ // Check if magnetic sensor overflow set, if not then report data
		destination[0] = ((u16)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
		destination[1] = ((u16)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
		destination[2] = ((u16)rawData[5] << 8) | rawData[4] ;
	}
	//}
}

u16 readTempData()
{
	u8 rawData[2];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
	return ((u16)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(f32 * destination)
{
	// First extract the factory calibration for each magnetometer axis
	u8 rawData[3];  // x/y/z gyro calibration data stored here
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	_delay_ms(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	_delay_ms(10);
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] =  (f32)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (f32)(rawData[1] - 128)/256. + 1.;
	destination[2] =  (f32)(rawData[2] - 128)/256. + 1.;
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	_delay_ms(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	_delay_ms(10);
}


void initMPU9250()
{
	// wake up device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	_delay_ms(100); // Wait for all registers to reset

	// get stable time source
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	_delay_ms(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum _delay_ms time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	u8 c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
	//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0]
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
	// writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

	// Set accelerometer full-scale range configuration
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
	//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	_delay_ms(100);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(f32 * dest1, f32 * dest2)
{
	u8 data[12]; // data array to hold accelerometer and gyro x, y, z, data
	u16 ii, packet_count, fifo_count;
	u32 gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	_delay_ms(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	_delay_ms(200);

	// Configure device for bias calculation
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	_delay_ms(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	u16  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	u16  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	_delay_ms(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((u16)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		u16 accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (u16) (((u16)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (u16) (((u16)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (u16) (((u16)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (u16) (((u16)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (u16) (((u16)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (u16) (((u16)data[10] << 8) | data[11]) ;

		accel_bias[0] += (u32) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (u32) accel_temp[1];
		accel_bias[2] += (u32) accel_temp[2];
		gyro_bias[0]  += (u32) gyro_temp[0];
		gyro_bias[1]  += (u32) gyro_temp[1];
		gyro_bias[2]  += (u32) gyro_temp[2];

	}
	accel_bias[0] /= (u32) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (u32) packet_count;
	accel_bias[2] /= (u32) packet_count;
	gyro_bias[0]  /= (u32) packet_count;
	gyro_bias[1]  /= (u32) packet_count;
	gyro_bias[2]  /= (u32) packet_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (u32) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (u32) accelsensitivity;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	dest1[0] = (f32) gyro_bias[0]/(f32) gyrosensitivity;
	dest1[1] = (f32) gyro_bias[1]/(f32) gyrosensitivity;
	dest1[2] = (f32) gyro_bias[2]/(f32) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	u32 accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (u32) (((u16)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (u32) (((u16)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (u32) (((u16)data[0] << 8) | data[1]);

	u32 mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	u8 mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

	// Output scaled accelerometer biases for display in the main program
	dest2[0] = (f32)accel_bias[0]/(f32)accelsensitivity;
	dest2[1] = (f32)accel_bias[1]/(f32)accelsensitivity;
	dest2[2] = (f32)accel_bias[2]/(f32)accelsensitivity;


}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(f32 * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	u8 rawData[6] = {0, 0, 0, 0, 0, 0};
	u8 selfTest[6];
	u16 gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
	f32 factoryTrim[6];
	u8 FS = 0;

	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

	for( int ii = 0; ii < 200; ii++)
	{  // get average current values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
		aAvg[0] += (u16)(((u16)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (u16)(((u16)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (u16)(((u16)rawData[4] << 8) | rawData[5]) ;

		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
		gAvg[0] += (u16)(((u16)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (u16)(((u16)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (u16)(((u16)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii =0; ii < 3; ii++)
	{  // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	_delay_ms(25);  // Delay a while to let the device stabilize

	for( int ii = 0; ii < 200; ii++)
	{  // get average self-test values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
		aSTAvg[0] += (u16)(((u16)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (u16)(((u16)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (u16)(((u16)rawData[4] << 8) | rawData[5]) ;

		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (u16)(((u16)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (u16)(((u16)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (u16)(((u16)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii =0; ii < 3; ii++)
	{  // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
	_delay_ms(25);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg

	readBytes(MPU9250_ADDRESS, SELF_TEST_X_ACCEL , 1 , &selfTest[0]); // X-axis accel self-test results
	readBytes(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL, 1 , &selfTest[1]); // Y-axis accel self-test results
	selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (f32)(2620/1<<FS)*(pow( 1.01 , ((f32)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (f32)(2620/1<<FS)*(pow( 1.01 , ((f32)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (f32)(2620/1<<FS)*(pow( 1.01 , ((f32)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (f32)(2620/1<<FS)*(pow( 1.01 , ((f32)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (f32)(2620/1<<FS)*(pow( 1.01 , ((f32)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (f32)(2620/1<<FS)*(pow( 1.01 , ((f32)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++)
	{
		destination[i]   = 100.0*((f32)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
		destination[i+3] = 100.0*((f32)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
	}

}

void MadgwickQuaternionUpdate(f32 ax, f32 ay, f32 az, f32 gx, f32 gy, f32 gz, f32 mx, f32 my, f32 mz, f32 * quaternionBuffer)
{
	f32 q1 = quaternionBuffer[0], q2 = quaternionBuffer[1], q3 = quaternionBuffer[2], q4 = quaternionBuffer[3];   // short name local variable for readability
	f32 norm;
	f32 hx, hy, _2bx, _2bz;
	f32 s1, s2, s3, s4;
	f32 qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	f32 _2q1mx;
	f32 _2q1my;
	f32 _2q1mz;
	f32 _2q2mx;
	f32 _4bx;
	f32 _4bz;
	f32 _2q1 = 2.0f * q1;
	f32 _2q2 = 2.0f * q2;
	f32 _2q3 = 2.0f * q3;
	f32 _2q4 = 2.0f * q4;
	f32 _2q1q3 = 2.0f * q1 * q3;
	f32 _2q3q4 = 2.0f * q3 * q4;
	f32 q1q1 = q1 * q1;
	f32 q1q2 = q1 * q2;
	f32 q1q3 = q1 * q3;
	f32 q1q4 = q1 * q4;
	f32 q2q2 = q2 * q2;
	f32 q2q3 = q2 * q3;
	f32 q2q4 = q2 * q4;
	f32 q3q3 = q3 * q3;
	f32 q3q4 = q3 * q4;
	f32 q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	quaternionBuffer[0] = q1 * norm;
	quaternionBuffer[1] = q2 * norm;
	quaternionBuffer[2] = q3 * norm;
	quaternionBuffer[3] = q4 * norm;

}

/*++++++++++++++++++++++++++++++++++++++++++++ Write to Device ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// Wire.h read and write protocols
void writeByte(u8 address, u8 subAddress, u8 data)
{
	u8 pui8_sendbuffer[2];
	pui8_sendbuffer[0] = subAddress;
	pui8_sendbuffer[1] = data;
#ifdef byteWrite_Errors							 //send a stop-condition
	u8 ui8_return = 0;
#endif
	//Wire.beginTransmission(address);  // Initialize the Tx buffer
#ifdef byteWrite_DEBUG
	UART_vSend_String((u8*)"Address is %x\n\r" , address);
#endif
	//i2c_start(address+I2C_WRITE);
	//Wire.write(subAddress);           // Put slave register address in Tx buffer
#ifdef byteWrite_DEBUG
	UART_vSend_String((u8*)"SubAddress to send is %x\n\r" , subAddress);
#endif

#ifdef byteWrite_DEBUG
	UART_vSend_String((u8*)"Data to send is %x\n\r" , data);
#endif

#ifdef byteWrite_Errors
	ui8_return =
#endif


			/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifdef byteWrite_Errors							 //send a stop-condition
			if(ui8_return != 0)
			{
				UART_vSend_String((u8*)"Whoopsie got the following error code during write process: %x\n\r" , ui8_return);
			}
#endif

	//i2c_stop();	//Free the bus for other operations
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*+++++++++++++++++++++++++++++++++++++++++ READ from Device ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

u8 readByte(u8 address, u8 subAddress)
{
#ifdef byteWrite_DEBUG
	UART_vSend_String((u8*)"Begin of readByte()\n\r");
#endif
	u8 ui8_data; // `data` will store the register data

	readBytes(address, subAddress, 1, &ui8_data);

	return ui8_data;                             // Return data read from slave register
}

void readBytes(u8 address, u8 subAddress, u8 byteCount, u8 * destinationBuffer)
{
#ifdef byteWrite_DEBUG
	UART_vSend_String((u8*)"Begin of readBytes()\n\r");
#endif
	u8 ui8_return = 0;

	//Wire.beginTransmission(address);         // Initialize the Tx buffer
#ifdef byteWrite_DEBUG
	UART_vSend_String((u8*)"Address is %x\n\r" , address);
#endif
	//i2c_start_wait(address+I2C_WRITE);
	//Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	//ui8_return = i2c_write(subAddress);
#ifdef byteWrite_DEBUG
	UART_vSend_String((u8*)"SubAddress to send is %x\n\r" , subAddress);
#endif

	//Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	//ui8_return = i2c_rep_start(address + I2C_READ);
	ui8_return = I2C_u8writeTo(address , &subAddress , 1 , 1 ,(u8) false); // Send to address content of *subAddress
	//with length 1 , wait for end of transmit and
	//don't send a stop-condition
	if(ui8_return == 0)
	{
#ifdef byteWrite_DEBUG
		UART_vSend_String((u8*)"Send subAddress %x to address %x\n\r" , subAddress , address);
#endif
	}
#ifdef byteWrite_Errors
	else if(ui8_return == 2)
	{
		UART_vSend_String((u8*)"Address was send received a NACK!\n\r");
	}
	else if(ui8_return == 3)
	{
		UART_vSend_String((u8*)"subAddress was send received a NACK!\n\r");
	}
	else
	{
		UART_vSend_String((u8*)"ERROR! Status-Code: %x!\n\r" , ui8_return);
	}
#endif
	//Wire.requestFrom(address, count);  // Read bytes from slave register address
	ui8_return = I2C_u8readFrom(address, destinationBuffer, byteCount, (u8)true); //Read byteCount-Bytes from
	//address an put it into the
	//destinationBuffer then send a
	//stop-condition
#ifdef byteWrite_Errors
	if(ui8_return != byteCount)
	{
		UART_vSend_String((u8*)"Whoopsie read 0x%x Bytes instead of 0x%x Byte...\n\r" , ui8_return , byteCount);
	}
#endif
	//while (Wire.available()) {
	//dest[i++] = Wire.read(); }         // Put read results in the Rx buffer <-- We don't need this because we let
	//					TWI Write directly into the memory region because we dont use a
	//					system buffer

	//i2c_stop();	//Free the bus for other operations <- This happens automatically at the end of twi_readFrom
	//		  				       because of the "true" as last parameter
#ifdef byteWrite_DEBUG
	UART_vSend_String((u8*)"End of readBytes()\n\r");
#endif
}

/*---------------------------------- Debug --------------------------------------------------------------------------------*/

u8 readByte_Debug(u8 address, u8 subAddress)
{
	UART_vSend_String((u8*)"Begin of readByte()\n\r");
	u8 ui8_data; // `data` will store the register data

	readBytes_Debug(address, subAddress, 1, &ui8_data);

	return ui8_data;                             // Return data read from slave register
}

void readBytes_Debug(u8 address, u8 subAddress, u8 byteCount, u8 * destinationBuffer)
{
	UART_vSend_String((u8*)"Begin of readBytes()\n\r");
	u8 ui8_return = 0;

	//Wire.beginTransmission(address);         // Initialize the Tx buffer
	UART_vSend_String((u8*)"Address is ");
	UART_vSendData(address);
	//i2c_start_wait(address+I2C_WRITE);
	//Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	//ui8_return = i2c_write(subAddress);
	UART_vSend_String((u8*)"SubAddress to send is ");
	UART_vSendData(subAddress);

	//Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	//ui8_return = i2c_rep_start(address + I2C_READ);
	ui8_return = I2C_u8writeTo(address , &subAddress , 1 , 1 ,(u8) false); // Send to address content of *subAddress
	//with length 1 , wait for end of transmit and
	//don't send a stop-condition
	if(ui8_return == 0)
	{
		UART_vSend_String((u8*)"Send subAddress ");
		UART_vSendData(subAddress);
		UART_vSend_String((u8*)"to address");
		UART_vSendData(address);
	}
	else if(ui8_return == 2)
	{
		UART_vSend_String((u8*)"Address was send received a NACK!\n\r");
	}
	else if(ui8_return == 3)
	{
		UART_vSend_String((u8*)"subAddress was send received a NACK!\n\r");
	}
	else
	{
		UART_vSend_String((u8*)"ERROR! Status-Code:");
		UART_vSendData( ui8_return);
	}
	//Wire.requestFrom(address, count);  // Read bytes from slave register address
	ui8_return = I2C_u8readFrom(address, destinationBuffer, byteCount, (u8)true); //Read byteCount-Bytes from
	//address an put it into the
	//destinationBuffer then send a
	//stop-condition
	if(ui8_return != byteCount)
	{
		UART_vSend_String((u8*)"Whoopsie read 0x");
		UART_vSendData(ui8_return);
		UART_vSend_String((u8*)"  Bytes instead of 0x%x Byte...\n\r");
		UART_vSendData(byteCount);
	}
	//while (Wire.available()) {
	//dest[i++] = Wire.read(); }         // Put read results in the Rx buffer <-- We don't need this because we let
	//					TWI Write directly into the memory region because we dont use a
	//					system buffer

	//i2c_stop();	//Free the bus for other operations <- This happens automatically at the end of twi_readFrom
	//		  				       because of the "true" as last parameter
	UART_vSend_String((u8*)"End of readBytes()\n\r");
}



/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


void magcalMPU9250(f32 * dest1, f32 * dest2)
{
	u16 ii = 0, sample_count = 0;
	u32 mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	u16 mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

	UART_vSend_String((u8*)"Mag Calibration: Wave device in a figure eight until done!");

	sample_count = 128;
	for(ii = 0; ii < sample_count; ii++) {
		readMagData(mag_temp);  // Read the mag data
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		_delay_ms(135);  // at 8 Hz ODR, new mag data is available every 125 ms
	}

	// Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	dest1[0] = (f32) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
	dest1[1] = (f32) mag_bias[1]*mRes*magCalibration[1];
	dest1[2] = (f32) mag_bias[2]*mRes*magCalibration[2];

	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	f32 avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	dest2[0] = avg_rad/((f32)mag_scale[0]);
	dest2[1] = avg_rad/((f32)mag_scale[1]);
	dest2[2] = avg_rad/((f32)mag_scale[2]);

	UART_vSend_String((u8*)"Mag Calibration done!");
}
