#include "stm32f10x.h"                  // Device header
#include "PWM.h"

//void PWM_Init_TIM1(u16 Psc,u16 Per)
//{
//	GPIO_InitTypeDef GPIO_InitStruct;
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
//	TIM_OCInitTypeDef TIM_OCInitStruct;
////	 | RCC_APB2Periph_AFIO,ENABLE
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1,ENABLE);//����ʱ��
//	
//	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//��ʼ��GPIO--PA8��PA11Ϊ�����������
//	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8 |GPIO_Pin_11;
//	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA,&GPIO_InitStruct);
//	
//	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);//��ʼ����ʱ����
//	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
//	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
//	TIM_TimeBaseInitStruct.TIM_Period=Per;
//	TIM_TimeBaseInitStruct.TIM_Prescaler=Psc;
//	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);/*��2��*///TIM2
//	
//	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;//��ʼ������Ƚ�
//	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
//	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
//	TIM_OCInitStruct.TIM_Pulse=0;
//	TIM_OC1Init(TIM1,&TIM_OCInitStruct);
//	TIM_OC4Init(TIM1,&TIM_OCInitStruct);
//	
//	TIM_CtrlPWMOutputs(TIM1,ENABLE);//�߼���ʱ��ר��--MOE�����ʹ��
//	
//	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);/*��3��*///ENABLE//OC1Ԥװ�ؼĴ���ʹ��
//	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);//ENABLE//OC4Ԥװ�ؼĴ���ʹ��
//	TIM_ARRPreloadConfig(TIM1,ENABLE);//TIM1��ARR��Ԥװ�ؼĴ���ʹ��
//	
//	TIM_Cmd(TIM1,ENABLE);//����ʱ����
//}


void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(PWM_TIM_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_A|PWM_PIN_B;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PWM_GPIO, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(PWM_TIMX);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(PWM_TIMX, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR
	TIM_OC3Init(PWM_TIMX, &TIM_OCInitStructure);
	TIM_OC2Init(PWM_TIMX, &TIM_OCInitStructure);//PA1  LEFT
	TIM_Cmd(PWM_TIMX, ENABLE);
}


