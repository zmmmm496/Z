#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"  // Device header

// 编码器初始化函数
void Encoder_Init(uint8_t encoder_num);

// 获取编码器速度函数
int16_t Encoder_Get(uint8_t encoder_num);

#endif
