#include "tsp_pid.h"

// int16_t dc = 0; // 电机控制的占空比，全局变量

float kp_motor = 0.1f; // 电机控制的比例系数
float ki_motor = 0.0f;
float kd_motor = 0.0f; // 电机控制的微分系数
float kp_servo = 0.1f; // 舵机控制的比例系数
float ki_servo = 0.0f;
float kd_servo = 0.0f; // 舵机控制的微分系数

extern uint16_t servo1_x ;
extern uint16_t servo2_y;

// // PID控制电机速度
// void tsp_motor_speed_pid(uint16_t target_speed_pid, uint8_t motor)
// {
//     float current_speed = Get_speed(motor);
//     int16_t output = tsp_pid_control(target_speed_pid, current_speed, kp_motor, ki_motor, kd_motor);
//     dc += output; // 增加或减少占空比
// 	// 限制输出范围
// 	if (dc > MOTOR_DC_LIMIT) dc = MOTOR_DC_LIMIT;
// 	if (dc < -MOTOR_DC_LIMIT) dc = -MOTOR_DC_LIMIT;

// 	tsp_motor_control(dc, motor);
// }

// 由于电机速度采用增量式pid控制，故不采用本文件中的pid控制方法。
// 电机速度的闭环控制定义在tsp_motor.c中

// 舵机控制的PID函数

void tsp_servo_control_pid(float target_x, float target_y, float current_x, float current_y){

    int16_t output_x = tsp_pid_control(target_x, current_x, kp_servo, ki_servo, kd_servo);
    int16_t output_y = tsp_pid_control(target_y, current_y, kp_servo, ki_servo, kd_servo);
    if(output_x > 30) output_x = 30; // 限制最大偏移
    if(output_x < -30) output_x = -30; // 限制最小偏移
    if(output_y > 30) output_y = 30; // 限制最大偏移
    if(output_y < -30) output_y = -30; // 限制最小偏移
    // 控制舵机
    servo1_x -= output_x;
    servo2_y -= output_y;
    tsp_tft18_show_int16(80, 1, output_x);
    tsp_tft18_show_int16(80, 2, output_y);
    if (servo1_x < 500) servo1_x = 500;
    if (servo1_x > 2050) servo1_x = 2050;
    if (servo2_y < 1100) servo2_y = 1100;
    if (servo2_y > 1700) servo2_y = 1700;
    tsp_tft18_show_int16(80, 3, servo1_x);
    tsp_tft18_show_int16(80, 4, servo2_y);
    tsp_servo_angle(SERVO1, servo1_x);
    tsp_servo_angle(SERVO2, servo2_y);
    // while(PUSH()){}
}

// 通用pid控制函数
int16_t tsp_pid_control(uint16_t target, uint16_t current, float kp, float ki, float kd)
{
    static float integral = 0.0f; // 积分项
    static float prev_error = 0.0f; // 上一次误差
    float error = target - current;
    integral += error;
    float derivative = error - prev_error;

    float output = kp * error + ki * integral + kd * derivative;

    prev_error = error;
    return (uint16_t)output;
    // 注意：这里没有限制输出范围，调用者需要自行处理
}
