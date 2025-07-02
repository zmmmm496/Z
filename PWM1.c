#include "stm32f10x.h"                  // Device header

void PWM1_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // 修正 RCC_APB1 为 RCC_APB2
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // TIM1_CH1 和 TIM1_CH2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    TIM_InternalClockConfig(TIM1);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1; // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1; // PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // CCR
    
    TIM_OC1Init(TIM1, &TIM_OCInitStructure); // 初始化通道1
    TIM_OC2Init(TIM1, &TIM_OCInitStructure); // 初始化通道2
    
    TIM_Cmd(TIM1, ENABLE);
	// 高级定时器需使能主输出
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM_SetCompare1(uint16_t Compare)
{
    if (Compare > 99) Compare = 99; // 防止超出范围
    TIM_SetCompare1(TIM1, Compare); // 使用 TIM1 的通道1
}

void PWM_SetCompare2(uint16_t Compare)
{
    if (Compare > 99) Compare = 99; // 防止超出范围
    TIM_SetCompare2(TIM1, Compare); // 使用 TIM1 的通道2
}
