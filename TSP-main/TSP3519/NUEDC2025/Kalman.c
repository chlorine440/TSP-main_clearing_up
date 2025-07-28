#include <math.h>
#include <stdio.h>
#include "Kalman.h"  // 可保留你自己的头文件或根据需要修改



// 卡尔曼滤波参数（可调）
#define Q_ANGLE   0.001f    // 角度过程噪声协方差
#define Q_GYRO    0.003f    // 陀螺仪过程噪声协方差
#define R_ANGLE   0.5f      // 加速度计测量噪声协方差

// 采样周期（20ms = 0.02s）
#define DT        0.02f

/**
 * @brief 卡尔曼滤波更新函数（标准实现）
 * @param state 卡尔曼状态结构体
 * @param acc_angle 加速度计计算的角度（测量值）
 * @param gyro_rate 陀螺仪角速度
 */
void Kalman_Update(KalmanState *state, float acc_angle, float gyro_rate) {
    // 1. 先验估计（预测）
    float rate = gyro_rate - state->bias;
    state->angle += DT * rate;

    // 2. 误差协方差预测
    float P00_temp = state->P[0][0];
    float P01_temp = state->P[0][1];
    float P10_temp = state->P[1][0];
    float P11_temp = state->P[1][1];

    state->P[0][0] += DT * (DT * P11_temp - P01_temp - P10_temp + Q_ANGLE);
    state->P[0][1] -= DT * P11_temp;
    state->P[1][0] -= DT * P11_temp;
    state->P[1][1] += Q_GYRO * DT;

    // 3. 计算卡尔曼增益
    float y = acc_angle - state->angle;  // 测量残差
    float S = state->P[0][0] + R_ANGLE;  // 残差协方差
    float K0 = state->P[0][0] / S;
    float K1 = state->P[1][0] / S;

    // 4. 更新后验估计
    state->angle += K0 * y;
    state->bias   += K1 * y;

    // 5. 更新误差协方差
    state->P[0][0] -= K0 * state->P[0][0];
    state->P[0][1] -= K0 * state->P[0][1];
    state->P[1][0] -= K1 * state->P[0][0];
    state->P[1][1] -= K1 * state->P[0][1];
}

// 初始化卡尔曼滤波器状态
KalmanState kalman_x = {0};
KalmanState kalman_y = {0};

/**
 * @brief 姿态解算主函数，调用频率应为 50Hz（20ms）
 */
void Angle_Calcu(void) {
    float acc[3], gyro[3];

    Get_Acc(acc);    // 获取加速度计数据
    Get_Gyro(gyro);  // 获取陀螺仪数据

    float ax = acc[0], ay = acc[1], az = acc[2];
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];

    // 使用 atan2 计算加速度计角度（更稳定）
    float angle_x_acc = atan2(ay, az) * 180.0f / 3.1415926f;
    float angle_y_acc = atan2(ax, az) * 180.0f / 3.1415926f;

    // 卡尔曼滤波更新
    Kalman_Update(&kalman_x, angle_x_acc, gx);
    Kalman_Update(&kalman_y, angle_y_acc, gy);

    // 保存最终角度
    float Angle_X_Final = kalman_x.angle;
    float Angle_Y_Final = kalman_y.angle;

    // 显示数据（根据你的平台保留）
    char buffer[50];
    sprintf(buffer, "X: %.2f, Y: %.2f", Angle_X_Final, Angle_Y_Final);
    tsp_tft18_show_str(0, 1, buffer); // 显示角度数据

    sprintf(buffer,"X: %.2f, Y: %.2f", ax, ay);
    tsp_tft18_show_str(0, 2, buffer); // 显示加速度数据

    sprintf(buffer,"X: %.2f, Y: %.2f", gx, gy);
    tsp_tft18_show_str(0, 3, buffer); // 显示陀螺仪数据
}