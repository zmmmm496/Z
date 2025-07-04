#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Encoder.h"
#include "Motor.h"

uint8_t KeyNum;

volatile int16_t Speed1 = 0;
volatile int16_t Speed2 = 0;

void TIM1_UP_IRQHandler(void);

int main(void)
{
	OLED_Init();
    Motor_Init();
	Timer_Init(); // 初始化定时器
    Encoder_Init(1); // 初始化第一个编码器
    Encoder_Init(2); // 初始化第二个编码器
	Set_Speed(40, 40);  // 设置左右电机的初始速度为30
	
	OLED_ShowString(1, 1, "Speed1:");
    OLED_ShowString(2, 1, "Speed2:");
    while (1)
    {
        OLED_ShowSignedNum(1, 8, Speed1*10, 5);
        OLED_ShowSignedNum(2, 8, Speed2*10, 5);
        Delay_ms(100); // 减少OLED刷新频率，避免闪烁
    }
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIMER_TIMX, TIM_IT_Update) == SET)
    {
		// 获取原始脉冲计数
        int16_t raw1 = Encoder_Get(1);
        int16_t raw2 = Encoder_Get(2);
		
        Speed1 = raw1 * 10;
        Speed2 = raw2 * 10;
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
