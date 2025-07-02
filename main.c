#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Kalman.h"                    // 引入 Kalman 滤波器头文件
#include "Servo.h"
#include "stm32f10x_tim.h"
#include "SensorModule.h"
#include "Motor.h"

uint8_t ID;
int16_t AX, AY, AZ, GX, GY, GZ;
float AngleX = 0, AngleY = 0, AngleZ = 0;          // 定义融合后的角度数据

// TIM2初始化（100Hz中断，10ms周期）
void TIM2_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Prescaler = 7200 - 1;  // 72MHz / 7200 = 10kHz
    TIM_TimeBaseStruct.TIM_Period = 100 - 1;      // 10kHz / 100 = 100Hz
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
    
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 0);  // 最高优先级
    TIM_Cmd(TIM2, ENABLE);
}

// TIM2中断服务函数
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        SensorModule_Run();  // 调用模块运行函数
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}


int main(void) {
    // 初始化模块
    SensorModule_Init();
    // 初始化TIM2中断
    TIM2_Init();
	
	Motor_Init();

    while (1) {
        // 主循环可完全空闲，或处理其他低优先级任务
        // 示例：添加阻塞式延时（测试中断独立性）
        Motor_SetSpeed(75, 75);  // 设置电机1正转50%，电机2正转75%
		
        Delay_ms(5000);  // 即使主循环被阻塞5秒，TIM2中断仍会触发
        Motor_SetSpeed(50, 50);  // 设置电机1正转50%，电机2正转75%
        Delay_ms(5000);  // 即使主循环被阻塞5秒，TIM2中断仍会触发
		
    }
}
