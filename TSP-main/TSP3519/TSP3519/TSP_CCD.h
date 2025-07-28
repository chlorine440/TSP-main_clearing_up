// TSP_CCD.h
#ifndef TSP_CCD_H
#define TSP_CCD_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "tsp_gpio.h"
#include "tsp_adc.h"

/** 线阵 CCD 像素数 */
#define CCD_PIXEL_COUNT   128

/** 线阵 CCD 一行数据类型 */
typedef uint16_t ccd_t[CCD_PIXEL_COUNT];



/**
 * @brief 初始化 CCD 接口
 *  - SYSCFG_DL_init(): GPIO & ADC 时钟多路复用设置
 *  - ADC_Init(): 配置 CCD 用 ADC 通道
 *  - 初始化 SI、CLK 引脚为数字输出
 */
void tsp_ccd_init(void);

/**
 * @brief 行清空：复位 CCD 内积累的电荷
 */
void tsp_ccd_flush(void);

/**
 * @brief 行快照：先清空上一帧，再触发新帧积分，最后依次读出 CCD_PIXEL_COUNT 个像素
 * @param buf 用户分配的长度为 CCD_PIXEL_COUNT 的数组
 * @return true 成功，false 失败
 */
bool tsp_ccd_snapshot(ccd_t buf);

/**
 * @brief CCD 时序所需最小时钟延时（根据时钟频率调整空循环次数）
 */
void tsp_ccd_delay(void);

/**
 * @brief 显示 CCD 数据
 * @param data CCD 数据数组
 */
void tsp_ccd_show(ccd_t data);
/**
 * @brief 显示 CCD 演示界面
 */
void tsp_demo_frame_ccd(void);

void CCD_test(void);
#endif // TSP_CCD_H
