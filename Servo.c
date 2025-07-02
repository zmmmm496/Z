#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Servo_Init(void)
{
    PWM_Init();
}

void Servo_SetAngle(float Angle)
{
    PWM_SetCompare3(Angle / 180 * 2000 + 500); // 修改为 TIM3 通道 3
}
