#ifndef TSP_MOTOR_H
#define TSP_MOTOR_H

#include "tsp_common_headfile.h"

// 编码器初始化
void tsp_encoder_init(void);

// 获取编码器值
void tsp_encoder_get_value(uint32_t *value);

// 清零编码器
void tsp_encoder_clear(void);

// 电机测试
void Motor_test(void);

void tsp_motor_speed_pid(uint16_t target_speed_pid, uint8_t motor);

// 电机速度闭环控制
void tsp_speed_close_loop(void);

// 电机电压控制
void tsp_motor_voltage(uint8_t dir, uint16_t duty_cycle, uint8_t motor);

// 电机控制
void tsp_motor_control(int16_t output, uint8_t motor);

// 停车
void tsp_motor_stop(void);

// 原地转向
void tsp_motor_turn_inplace(uint8_t dir, uint16_t duty_cycle_limit, uint16_t angle);

// 差速底盘巡线
void tsp_line_follower(uint16_t err);

float Get_speed(uint8_t motor);

#endif // TSP_MOTOR_H