#include "ti_msp_dl_config.h"
#include "tsp_tft18.h"
#include "tsp_pwm.h"
#include "tsp_isr.h"
#include "tsp_servo.h"
#include <math.h>
#include "tsp_pid.h"

#define SERVO_CENTER_X 1100
#define SERVO_CENTER_Y 1500
float k_angle_to_duty = 795.0f / 90.0f; // 90度对应795个脉宽单位

extern uint16_t route_x[999]; // 路径点X坐标数组
extern uint16_t route_y[999]; // 路径点Y坐标数组
extern uint16_t route_index; // 路径点索引
extern float kp_servo; // 舵机控制的比例系数
extern float ki_servo; // 舵机控制的积分系数
extern float kd_servo; // 舵机控制的微分系数

uint16_t servo1_x = SERVO_CENTER_X;
uint16_t servo2_y = SERVO_CENTER_Y;

extern uint16_t point_x;
extern uint16_t point_y;// 当前激光点坐标

// 角度转脉宽函数
uint16_t tsp_angle_to_pwm(float angle)
{
    return (uint16_t)(SERVO_CENTER_X + (angle) * k_angle_to_duty);
}

// 距离转角度函数
float tsp_length_to_angle(float length, float distance)
{
    return atan2f(length, distance) * (180.0f / 3.1415926f); // atan2返回弧度，转换为角度
}


// 通过给定点坐标操控舵机
void tsp_servo_goto(uint16_t point_x, uint16_t point_y, float distance){
    float angle_x = tsp_length_to_angle(point_x, distance);
    float angle_y = tsp_length_to_angle(point_y, distance);
    // 将坐标映射到 PWM
    uint16_t servo1_pwm = tsp_angle_to_pwm(angle_x);
    uint16_t servo2_pwm = tsp_angle_to_pwm(angle_y);
    tsp_servo_angle(SERVO1, servo1_pwm);
    tsp_servo_angle(SERVO2, servo2_pwm);
}

// 画圆函数
void tsp_servo_draw_circle(float radius, uint16_t steps, float distance) // radius为半径，单位cm，steps为步数，distance为云台到屏幕的距离
{
    for (uint16_t i = 0; i < steps; i++) {
        float theta = 2.0f * 3.1415926f * i / steps;
       
        float x = (float)(radius * cosf(theta));
        float y = (float)(radius * sinf(theta));
        tsp_servo_goto(x, y, distance);

        //delay_1ms(200); // 延时，保证舵机运动平滑
    }
}

// 闭环控制云台激光笔巡线
void tsp_servo_line_follower(void){

    if (route_index >= 999) return; // 防止越界
    if(route_x[route_index] == '\0' && route_y[route_index] == '\0') return; // 如果当前点无效，直接返回

    // 目标点
    uint16_t target_x = route_x[route_index];
    uint16_t target_y = route_y[route_index];

    int16_t output_x = tsp_pid_control(target_x, point_x, kp_servo, ki_servo, kd_servo);
    int16_t output_y = tsp_pid_control(target_y, point_y, kp_servo, ki_servo, kd_servo);

    servo1_x -= output_x;
    servo2_y += output_y;

    tsp_servo_angle(SERVO1, servo1_x);
    tsp_servo_angle(SERVO2, servo2_y);

    // 判断是否到达目标点
    float error_x = target_x - point_x;
    float error_y = target_y - point_y;
    if (fabsf(error_x) < 3.0f && fabsf(error_y) < 3.0f) {
        route_index++;
    }
}