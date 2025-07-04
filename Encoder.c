#include "stm32f10x.h"                  // Device header

void Encoder_Init(uint8_t encoder_num)
{
    if (encoder_num == 1)
    {
        // 初始化第一个编码器（TIM3, GPIOA Pin6, Pin7）
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
        TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1; // ARR
        TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1; // PSC
        TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

        TIM_ICInitTypeDef TIM_ICInitStructure;
        TIM_ICStructInit(&TIM_ICInitStructure);
        TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; // 第一个编码器绑定到通道1
        TIM_ICInitStructure.TIM_ICFilter = 0xF;
        TIM_ICInit(TIM3, &TIM_ICInitStructure);
        TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; // 第一个编码器绑定到通道2
        TIM_ICInit(TIM3, &TIM_ICInitStructure);

        TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

        TIM_Cmd(TIM3, ENABLE);
    }
    else if (encoder_num == 2)
    {
        // 初始化第二个编码器（TIM4, GPIOB Pin6, Pin7）
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
        TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1; // ARR
        TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1; // PSC
        TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

        TIM_ICInitTypeDef TIM_ICInitStructure;
        TIM_ICStructInit(&TIM_ICInitStructure);
        TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; // 第二个编码器绑定到通道1
        TIM_ICInitStructure.TIM_ICFilter = 0xF;
        TIM_ICInit(TIM4, &TIM_ICInitStructure);
        TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; // 第二个编码器绑定到通道2
        TIM_ICInit(TIM4, &TIM_ICInitStructure);

        TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

        TIM_Cmd(TIM4, ENABLE);
    }
}

int16_t Encoder_Get(uint8_t encoder_num)
{
    int16_t Temp;
    if (encoder_num == 1)
    {
        Temp = TIM_GetCounter(TIM3); // 获取第一个编码器的速度
        TIM_SetCounter(TIM3, 0);     // 清零计数器
    }
    else if (encoder_num == 2)
    {
        Temp = TIM_GetCounter(TIM4); // 获取第二个编码器的速度
        TIM_SetCounter(TIM4, 0);     // 清零计数器
    }
    return Temp;
}
