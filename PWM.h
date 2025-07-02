#ifndef __PWM_H
#define __PWM_H

#define PWM_MAX  50
#define PWM_MIN -50

#define PWM_PIN_A               GPIO_Pin_1  //PWM�������A��������CH1��CH2
#define PWM_PIN_B               GPIO_Pin_2  //PWM�������B
#define PWM_GPIO                GPIOA 
#define PWM_TIMX                TIM2  //��Ӧʱ��
#define PWM_PORT_CLK			RCC_APB2Periph_GPIOA
#define PWM_TIM_CLK            RCC_APB1Periph_TIM2


void PWM_Init(void);
//void PWM_SetCompare1(uint16_t Compare);
//void PWM_SetCompare2(uint16_t Compare);

#endif
