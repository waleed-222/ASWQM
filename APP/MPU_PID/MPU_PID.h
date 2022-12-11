/*
 * MPU&PID.h
 *
 *  Created on: Dec 6, 2022
 *      Author: hp
 */

#ifndef MPU_PID_MPU_PID_H_
#define MPU_PID_MPU_PID_H_

//f32 setPoint = 1.0f;/*HELP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!should be used in main*/

/* Controller parameters */
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 4.0f

/* Simulated dynamical system (first order) */
f32 TestSystem_Update(float inp);
void MPU_PID_vInt();

#endif /* MPU_PID_MPU_PID_H_ */
