/*
 * param.h
 *
 *  Created on: 16-Sep-2025
 *      Author: nidhi
 */

#ifndef MAIN_ALGO_PARAM_H_
#define MAIN_ALGO_PARAM_H_

typedef struct {
	int	x, y, z;			/* Three Axes of an IMU */
} imu_t;

#define sampling_freq 100
#define buffer_len (3*sampling_freq)
#define PI 3.1415926535897932384f
#define DEGPERSEC 2000.0f / 32768.0f
#define DEGTORAD PI / 180.0f

#define DT (1.0f/sampling_freq)

extern int filter_state ; // 0: not converged - to use large beta, 1: converged - to use small beta
extern float beta ;

//imu_t a,g,m;

extern float quaternions[] ;
extern float eulers[] ;


void update_quaternions(float * newQuaternion, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt, float beta);

#endif /* MAIN_ALGO_PARAM_H_ */
