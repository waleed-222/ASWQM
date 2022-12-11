/*
 * MPU&PID.c
 *
 *  Created on: Dec 6, 2022
 *      Author: hp
 */


#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"

#include "../../MCAL/DIO/DIO_int.h"
#include "../PID/PID.h"
#include "MPU_PID.h"
void MPU_PID_vInt()
{

PIDController pid =
{
		PID_KP, PID_KI, PID_KD,
		PID_TAU,
		PID_LIM_MIN, PID_LIM_MAX,
		PID_LIM_MIN_INT, PID_LIM_MAX_INT,
		SAMPLE_TIME_S
};

PIDController_Init(&pid);

/* Simulate response using test system */

}

f32 TestSystem_Update(f32 inp) {

    static f32 output = 0.0f;
    static const f32 alpha = 0.02f;

    output = (SAMPLE_TIME_S * inp + output) / (1.0f + alpha * SAMPLE_TIME_S);

    return output;
}

f32 PID_f32UpdateYaw(f32 A_f32Yaw)
{
	return TestSystem_Update(A_f32Yaw);
}

f32 PID_f32UpdatePitch(f32 A_f32Pitch)
{
	return TestSystem_Update(A_f32Pitch);
}
f32 PID_f32UpdateRoll(f32 A_f32Roll)
{
	return TestSystem_Update(A_f32Roll);
}
