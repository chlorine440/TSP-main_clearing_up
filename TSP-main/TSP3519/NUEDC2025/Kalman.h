#ifndef __KALMAN_H
#define __KALMAN_H 
//卡尔曼滤波
#include "tsp_common_headfile.h"

// 卡尔曼滤波器状态结构体
typedef struct {
    float angle;      // 最优角度估计值
    float bias;       // 陀螺仪偏差估计
    float P[2][2];    // 误差协方差矩阵
} KalmanState;


void Angle_Calcu(void);
void Kalman_Update(KalmanState *state, float acc_angle, float gyro_rate);



#endif
