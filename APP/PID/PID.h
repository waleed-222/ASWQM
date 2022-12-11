/*
 * PID.h
 *
 *  Created on: Dec 6, 2022
 *      Author: hp
 */

#ifndef PID_PID_H_
#define PID_PID_H_

typedef struct {

	/* Controller gains */
	f32 Kp;
	f32 Ki;
	f32 Kd;

	/* Derivative low-pass filter time constant */
	f32 tau;

	/* Output limits */
	f32 limMin;
	f32 limMax;

	/* Integrator limits */
	f32 limMinInt;
	f32 limMaxInt;

	/* Sample time (in seconds) */
	f32 T;

	/* Controller "memory" */
	f32 integrator;
	f32 prevError;			/* Required for integrator */
	f32 differentiator;
	f32 prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	f32 out;

} PIDController;

void  PIDController_Init(PIDController *pid);
f32 PIDController_Update(PIDController *pid, f32 setpoint, f32 measurement);

#endif /* PID_PID_H_ */
