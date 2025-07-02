#include "stm32f10x.h"                  // Device header



/**************************************************************************
�������ܣ���ʱ�жϳ�ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����3
**************************************************************************/
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_Period = arr;     //��װ��ֵARR
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc;  //Ԥ��Ƶϵ��PSC
	TIM_TimeBaseInitStruct.TIM_ClockDivision =0; //ʱ�ӷָ�
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);  //ʹ�ܶ�ʱ���ж�
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;   //ʹ���ⲿ�ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;   //ʹ���ⲿ�ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�1
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;    //��Ӧ���ȼ�3
	NVIC_Init(&NVIC_InitStruct);

	TIM_Cmd(TIM1,ENABLE);	  //ʹ�ܶ�ʱ��3
}
