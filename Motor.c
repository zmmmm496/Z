#include "stm32f10x.h"                  // Device header
#include "PWM1.h"

void Motor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    PWM1_Init();
}

void Motor_SetSpeed(int8_t Speed1, int8_t Speed2)
{
    // 控制电机1
    if (Speed1 >= 0)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_4);  // IN1 = 1
        GPIO_ResetBits(GPIOA, GPIO_Pin_5); // IN2 = 0
        PWM_SetCompare1(Speed1); // 使用TIM1的通道1
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4); // IN1 = 0
        GPIO_SetBits(GPIOA, GPIO_Pin_5);  // IN2 = 1
        PWM_SetCompare1(-Speed1); // 使用TIM1的通道1
    }

    // 控制电机2
    if (Speed2 >= 0)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_6);  // IN3 = 1
        GPIO_ResetBits(GPIOA, GPIO_Pin_7); // IN4 = 0
        PWM_SetCompare2(Speed2); // 使用TIM1的通道2
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_6); // IN3 = 0
        GPIO_SetBits(GPIOA, GPIO_Pin_7);  // IN4 = 1
        PWM_SetCompare2(-Speed2); // 使用TIM1的通道2
    }
}
