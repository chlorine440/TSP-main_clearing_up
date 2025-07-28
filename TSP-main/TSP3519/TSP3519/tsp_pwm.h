#ifndef TSP_PWM_H
#define TSP_PWM_H

#include <stdint.h>

// 舵机通道定义
#define SERVO1  0
#define SERVO2  1

// 电机方向定义
#define MOTORF  0 // 前进
#define MOTORB  1 // 后退

#define MOTOR1  0
#define MOTOR2  1

#define LEFT  0
#define RIGHT 1

#define CH1_LOWER_LIMIT	500U
#define CH1_UPPER_LIMIT	1900U
#define CH2_LOWER_LIMIT	500U
#define CH2_UPPER_LIMIT	1900U

#define MOTOR_DC_LIMIT	50

// PID参数变量声明
extern float kp_motor;
extern float ki_motor;
extern float kd_motor;
extern float kp_servo;
extern float ki_servo;
extern float kd_servo;
extern float kp_turn_motor;
extern float ki_turn_motor;
extern float kd_turn_motor;
extern float kp_angle_to_err;

// 其他外部变量声明
extern float yaw;
extern float roll;
extern float pitch;
extern int16_t target_speed_qei1;
extern int16_t target_speed_qei2;
extern int16_t current_speed_qei1;
extern int16_t current_speed_qei2;

// 函数声明
void pwm_init(void);
void tsp_servo_angle(uint8_t channel, uint16_t pulse_width);
void tsp_motor_voltage(uint8_t dir, uint16_t duty_cycle, uint8_t motor);
void tsp_motor_control(int16_t dc, uint8_t motor);
void tsp_update_current_speed(void);

#endif // TSP_PWM_H