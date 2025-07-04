#ifndef __PWM_H
#define __PWM_H

#define PWM_MAX  100
#define PWM_MIN -100

#define PWM_PIN_A               GPIO_Pin_1  //PWMʤ³öҽ½Ł£¬±ؐ늇CH1º̓H2
#define PWM_PIN_B               GPIO_Pin_2  //PWMʤ³öҽ½ł
#define PWM_GPIO                GPIOA 
#define PWM_TIMX                TIM2  //¶ԓ¦ʱ֓
#define PWM_PORT_CLK			RCC_APB2Periph_GPIOA
#define PWM_TIM_CLK            RCC_APB1Periph_TIM2


void PWM_Init(void);
//void PWM_SetCompare1(uint16_t Compare);
//void PWM_SetCompare2(uint16_t Compare);

#endif
