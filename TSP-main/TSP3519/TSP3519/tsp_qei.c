#include "tsp_qei.h"
#include "tsp_common_headfile.h"

// QEI相关变量
static volatile int32_t qei1_last_count = 0;
static volatile int32_t qei2_last_count = 0;
static volatile int8_t qei1_direction = 0;
static volatile int8_t qei2_direction = 0;


int16_t encoder_pulse_qei1;
int16_t encoder_pulse_qei2;

int16_t encoder_speed_motor1;
int16_t encoder_speed_motor2;

extern int16_t current_speed_qei1;
extern int16_t current_speed_qei2;



#define QEI_FREQ 500 // 假设QEI的频率为500Hz
#define PI 3.14159265358979323846 // 圆周率
#define R 32.5f // 假设轮子的半径为32.5毫米

// 假定使用 QEI_1_INST 作为编码器硬件实例
// 可根据实际硬件修改


void TSP_QEI_Init(void)
{
    // 初始化 QEI硬件（由SysConfig生成的初始化函数）
    //SYSCFG_DL_QEI_1_init();
    //SYSCFG_DL_QEI_2_init();
    DL_TimerG_startCounter(QEI_1_INST);
    DL_TimerG_startCounter(QEI_2_INST);
    qei1_last_count = 0; // 初始化QEI1的计数器为0
    qei2_last_count = 0; // 初始化QEI2的计数器为0
    qei1_direction = 0; // 初始化QEI1的方向为0
    qei2_direction = 0; // 初始化QEI2的方向为0
}


int32_t TSP_QEI1_GetCount(void)
{
    // 获取 QEI1 的计数值
    // 使用 DL_Timer_getTimerCount 函数获取计数值
    qei1_last_count = DL_Timer_getTimerCount(QEI_1_INST);
    return qei1_last_count;
    
}

int32_t TSP_QEI2_GetCount(void)
{
    qei2_last_count = DL_Timer_getTimerCount(QEI_2_INST);
    return qei2_last_count;
}


void TSP_QEI1_ResetCount(void)
{
    DL_Timer_setCounterValueAfterEnable(QEI_1_INST, 0);
    qei1_last_count = 0;
    qei1_direction = 0;
}

void TSP_QEI2_ResetCount(void)
{
    DL_Timer_setCounterValueAfterEnable(QEI_2_INST, 0);
    qei2_last_count = 0;
    qei2_direction = 0;
}


int8_t TSP_QEI1_GetDirection(void)
{
    int32_t now = DL_Timer_getTimerCount(QEI_1_INST);
    if (now > qei1_last_count)
        qei1_direction = 1;
    else if (now < qei1_last_count)
        qei1_direction = -1;
    else
        qei1_direction = 0;
    qei1_last_count = now;
    return qei1_direction;
}

int8_t TSP_QEI2_GetDirection(void)
{
    int32_t now = DL_Timer_getTimerCount(QEI_2_INST);
    if (now > qei2_last_count)
        qei2_direction = 1;
    else if (now < qei2_last_count)
        qei2_direction = -1;
    else
        qei2_direction = 0;
    qei2_last_count = now;
    return qei2_direction;
}

// 如需中断处理，可在此添加QEI相关中断服务函数
// void QEI_IRQHandler(void) {
//     // 在此处添加QEI中断处理逻辑
// }

// 获取QEI1和QEI2的速度，单位mm/s
int16_t tsp_qei1_get_speed(void){
    return encoder_speed_motor1 * 50 / QEI_FREQ * PI * 2 * R / 100 / 117 * 102;
}

int16_t tsp_qei2_get_speed(void){
    return -encoder_speed_motor2 * 50 / QEI_FREQ * PI * 2 * R / 100 / 117 * 102;
}