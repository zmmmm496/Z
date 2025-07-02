#ifndef __SENSOR_MODULE_H
#define __SENSOR_MODULE_H

#include "stm32f10x.h"
#include "MPU6050.h"
#include "Kalman.h"
#include "OLED.h"
#include "Servo.h"

// 模块初始化函数
void SensorModule_Init(void);
// 模块运行函数（在TIM2中断中调用）
void SensorModule_Run(void);

#endif
