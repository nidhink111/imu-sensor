/*
 * param.c
 *
 *  Created on: 16-Sep-2025
 *      Author: nidhi
 */

#include "madgwick.h"
#include "param.h"

#define HIGH_BETA 2.0f

int filter_state = 0; // 0: not converged - to use large beta, 1: converged - to use small beta
float beta = HIGH_BETA;

//imu_t a,g,m;

float quaternions[] = {1.0, 0.0, 0.0, 0.0};
float eulers[] = {0.0f, 0.0f, 0.0f};

void update_quaternions(float * newQuaternion, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt, float beta){

    static struct madgwick update = {.q_est_t.a = 1.0};

    struct euclidean mag, accel, gyro;

    update.beta = beta;

    mag.x = mx;
    mag.y = my;
    mag.z = mz;

    accel.x = ax;
    accel.y = ay;
    accel.z = az;

    gyro.x = gx;
    gyro.y = gy;
    gyro.z = gz;

    madgwick_update(&update, &mag, &accel, &gyro, dt);

    newQuaternion[0] = update.q_est_t.a;
    newQuaternion[1] = update.q_est_t.b;
    newQuaternion[2] = update.q_est_t.c;
    newQuaternion[3] = update.q_est_t.d;
}