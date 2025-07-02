#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // PB0 所在的 GPIOB

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;       // 设置复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;            // 修改为 PB0
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM3);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;       // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;       // PSC
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                     // CCR
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);               // 修改为 TIM3 的通道 3（CH3）

    TIM_Cmd(TIM3, ENABLE);
}

void PWM_SetCompare3(uint16_t Compare)
{
    TIM_SetCompare3(TIM3, Compare);                        // 修改为 TIM3 的通道 3（CH3）
}
