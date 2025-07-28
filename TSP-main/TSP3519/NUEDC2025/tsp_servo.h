#ifndef TSP_SERVO_H
#define TSP_SERVO_H

#include <stdint.h>

// 角度转脉宽
uint16_t tsp_angle_to_pwm(float angle);

// 距离转角度
float tsp_length_to_angle(float length, float distance);

// 画圆函数
void tsp_servo_draw_circle(float radius, uint16_t steps, float distance);

// 通过给定点坐标操控舵机
void tsp_servo_goto(uint16_t point_x, uint16_t point_y, float distance);

void tsp_servo_control_pid(float target_x, float target_y, float current_x, float current_y);

// 闭环控制云台激光笔巡线
void tsp_servo_line_follower(void);

#endif // TSP_SERVO_H