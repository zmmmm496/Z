#include "stm32f10x.h"                  // Device header
#include "Timer.h"

void Timer_Init(void)
{
    // 使能定时器TIM1的时钟，注意TIM1属于APB2总线
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    // 配置定时器TIM1
    TIM_InternalClockConfig(TIMER_TIMX);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 7200 - 1; // 设置自动重装载寄存器的值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 100 - 1; // 设置预分频器，定时器频率为1kHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIMER_TIMX, &TIM_TimeBaseInitStructure);
    
    TIM_ClearFlag(TIMER_TIMX, TIM_FLAG_Update);
    TIM_ITConfig(TIMER_TIMX, TIM_IT_Update, ENABLE);
    
    // 配置中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn; // 使用TIM1_UP_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级1（0-3，数值越小优先级越高）
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 子优先级0（0-3）
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIMER_TIMX, ENABLE);
}

/*
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
*/
