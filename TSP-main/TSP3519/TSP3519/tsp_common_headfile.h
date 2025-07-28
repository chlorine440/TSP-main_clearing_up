#ifndef _tsp_common_headfile_h_
#define _tsp_common_headfile_h_

#include "stdio.h"
#include "stdint.h"
#include "string.h"

//===================================================芯片 SDK 底层===================================================
#include "ti_msp_dl_config.h"
//===================================================芯片 SDK 底层===================================================

//===================================================芯片外设驱动层===================================================
#include "tsp_adc.h"
#include "tsp_gpio.h"
#include "tsp_uart.h"
#include "tsp_i2c.h"
#include "tsp_qei.h"
#include "tsp_pwm.h"

#include "TSP_MPU6050.h"
#include "TSP_TFT18.h"
#include "TSP_CCD.h"
#include "tsp_battery.h"
//===================================================芯片外设驱动层===================================================

//===================================================用户自定义文件===================================================
#include "tsp_isr.h"
#include "tsp_motor.h"
#include "tsp_servo.h"
#include "tsp_ccd_image.h"
#include "tsp_menu.h"
#include "Kalman.h"

//===================================================用户自定义文件===================================================

#endif
