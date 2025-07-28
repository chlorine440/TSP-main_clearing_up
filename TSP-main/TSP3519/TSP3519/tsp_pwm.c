#include "ti_msp_dl_config.h"
#include "tsp_pwm.h"
#include "tsp_tft18.h"
#include "tsp_qei.h"

#define CH1_LOWER_LIMIT	300U
#define CH1_UPPER_LIMIT	2100U
#define CH2_LOWER_LIMIT	500U
#define CH2_UPPER_LIMIT	1900U

#define MOTOR_DC_LIMIT	4000

extern float kp_motor;
extern float ki_motor;
extern float kd_motor; // 电机控制的微分系数
extern float kp_servo; // 舵机控制的比例系数
extern float ki_servo; // 舵机控制的积分系数
extern float kd_servo; // 舵机控制的微分系数
extern float kp_turn_motor; // 原地转向电机控制的比例系数
extern float ki_turn_motor; // 原地转向电机控制的积分系数
extern float kd_turn_motor; // 原地转向电机控制的微分系数

extern float kp_angle_to_err; // 角度转误差的比例系数
extern float yaw; // 当前航向角
extern float roll; // 当前滚转角
extern float pitch; // 当前俯仰角


extern int16_t target_speed_qei1; // 目标速度 QEI1
extern int16_t target_speed_qei2; // 目标速度 QEI2
extern int16_t current_speed_qei1; // 当前速度 QEI1
extern int16_t current_speed_qei2; // 当前速度 QEI2


void pwm_init(void){
   // SYSCFG_DL_Servo_init();
    // SYSCFG_DL_Motor_init();
	DL_GPIO_setPins(PORTB_PORT,PORTB_SLEEP_PIN);
}

// 舵机驱动函数
// servo1为水平（减为顺时针，加为逆时针）
// servo2为垂直（减为抬头，加为低头）
void tsp_servo_angle(uint8_t channel, uint16_t pulse_width){

	uint16_t duty;
	
	duty = pulse_width;
	switch (channel)
	{
		case SERVO1:
			if (CH1_LOWER_LIMIT > pulse_width)
				duty = CH1_LOWER_LIMIT;
			if (CH1_UPPER_LIMIT < pulse_width)
				duty = CH1_UPPER_LIMIT;
			DL_TimerG_setCaptureCompareValue(Servo_INST, duty, DL_TIMER_CC_0_INDEX);
            tsp_tft18_show_str_color(0, 1, "Servo1 Angle Set", BLUE, YELLOW);
			break;
		case SERVO2:
			if (CH2_LOWER_LIMIT > pulse_width)
				duty = CH2_LOWER_LIMIT;
			if (CH2_UPPER_LIMIT < pulse_width)
				duty = CH2_UPPER_LIMIT;
			DL_TimerG_setCaptureCompareValue(Servo_INST, duty, DL_TIMER_CC_1_INDEX);
			break;
		default:
			break;
	}
}

// 电机驱动函数
void tsp_motor_voltage(uint8_t dir, uint16_t duty_cycle, uint8_t motor)
{
	uint16_t dc;
	
	dc = duty_cycle;
	if (MOTOR_DC_LIMIT < duty_cycle)
		dc = MOTOR_DC_LIMIT;
	tsp_tft18_show_str_color(0,2, "Motor Voltage Set", BLUE, YELLOW);
	DL_GPIO_setPins(GPIO_Motor_C0_PORT, GPIO_Motor_C0_PIN);
	DL_GPIO_setPins(GPIO_Motor_C1_PORT, GPIO_Motor_C1_PIN);
	DL_GPIO_setPins(GPIO_Motor_C2_PORT, GPIO_Motor_C2_PIN);
	DL_GPIO_setPins(GPIO_Motor_C3_PORT, GPIO_Motor_C3_PIN);
	switch (dir)
	{
		case MOTORF:
			if(motor == MOTOR1){
				DL_TimerG_setCaptureCompareValue(Motor_INST, dc, DL_TIMER_CC_0_INDEX);
				DL_TimerG_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_1_INDEX);
			}
			else if(motor == MOTOR2){
				DL_TimerG_setCaptureCompareValue(Motor_INST, dc, DL_TIMER_CC_2_INDEX);
				DL_TimerG_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_3_INDEX);
			}
			break;
		case MOTORB:
			if(motor == MOTOR1){
				DL_TimerG_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_0_INDEX);
				DL_TimerG_setCaptureCompareValue(Motor_INST, dc, DL_TIMER_CC_1_INDEX);
			}
			else if(motor == MOTOR2){
				DL_TimerG_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_2_INDEX);
				DL_TimerG_setCaptureCompareValue(Motor_INST, dc, DL_TIMER_CC_3_INDEX);
			}
			break;
		default:
			break;
	}
}

// 封装后的电机控制函数
void tsp_motor_control(int16_t dc, uint8_t motor){
	if(dc > 0 || dc == 0){
		tsp_motor_voltage(MOTORF, dc, motor);
	}
	else if(dc < 0){
		tsp_motor_voltage(MOTORB, -dc, motor);
	}
}

// 更新当前速度
void tsp_update_current_speed(void){
	current_speed_qei1 = tsp_qei1_get_speed();
	current_speed_qei2 = tsp_qei2_get_speed();
}