#ifndef _TSP_PID_H_
#define _TSP_PID_H_

#include "tsp_common_headfile.h"

// // 电机速度PID控制
// void tsp_motor_speed_pid(uint16_t target_speed_pid, uint8_t motor);

// 舵机PID控制
void tsp_servo_control_pid(float target_x, float target_y, float current_x, float current_y);

// 通用PID控制函数
float tsp_pid_control(uint16_t target, uint16_t current, float kp, float ki, float kd);

#endif /* _TSP_PID_H_ */