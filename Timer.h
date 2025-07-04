#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"  // Device header

// 定时器相关宏定义
#define TIMER_TIMX                	TIM1
#define TIMER_IRQn 					TIM1_UP_IRQn // 标准库中断名

void Timer_Init(void);

#endif
