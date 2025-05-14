/* 小车MPU6050姿态控制系统 - 基于STM32F103C8T6 */

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "math.h"
#include "delay.h"
#include "stdio.h"

// MPU6050定义
#define MPU6050_ADDR          0xD0 // 0x68 << 1 (I2C地址是7位，需要左移一位)
#define MPU_ACCEL_XOUTH_REG   0x3B  // 加速度值寄存器
#define MPU_GYRO_XOUTH_REG    0x43  // 陀螺仪值寄存器
#define MPU_PWR_MGMT1_REG     0x6B  // 电源管理寄存器1
#define MPU_GYRO_CFG_REG      0x1B  // 陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG     0x1C  // 加速度计配置寄存器
#define MPU_SAMPLE_RATE_REG   0x19  // 采样率分频寄存器
#define MPU_INT_EN_REG        0x38  // 中断使能寄存器
#define MPU_USER_CTRL_REG     0x6A  // 用户控制寄存器
#define MPU_FIFO_EN_REG       0x23  // FIFO使能寄存器
#define MPU_INTBP_CFG_REG     0x37  // 中断/旁路设置寄存器
#define MPU_CONFIG_REG        0x1A  // 配置寄存器

// 电机控制引脚定义
#define MOTOR_LEFT_FORWARD    GPIO_Pin_5  // IN1
#define MOTOR_LEFT_BACKWARD   GPIO_Pin_6  // IN2
#define MOTOR_RIGHT_FORWARD   GPIO_Pin_7  // IN3
#define MOTOR_RIGHT_BACKWARD  GPIO_Pin_8  // IN4
#define MOTOR_LEFT_ENABLE     GPIO_Pin_8  // ENA
#define MOTOR_RIGHT_ENABLE    GPIO_Pin_9  // ENB

// 系统状态和操作模式
#define SYS_MODE_STANDBY      0  // 待机模式
#define SYS_MODE_FORWARD      1  // 直线前进模式
#define SYS_MODE_TURNING      2  // 转弯模式
#define SYS_MODE_CALIBRATION  3  // 传感器校准模式
#define SYS_MODE_TESTING      4  // 系统测试模式

// PID控制参数 - 直线行走
#define KP_STRAIGHT  3.5f    // 比例系数
#define KI_STRAIGHT  0.02f   // 积分系数
#define KD_STRAIGHT  1.2f    // 微分系数

// PID控制参数 - 转弯
#define KP_TURN      4.5f    // 比例系数 - 转弯时更大以提高响应速度
#define KI_TURN      0.0f    // 积分系数 - 转弯时不需要积分
#define KD_TURN      2.0f    // 微分系数 - 转弯时更大以减小超调

// 卡尔曼滤波参数
#define Q_ANGLE     0.001f   // 过程噪声方差
#define Q_GYRO      0.003f   // 角速度噪声方差
#define R_ANGLE     0.03f    // 测量噪声方差

// 电机控制相关常量
#define MAX_MOTOR_SPEED       999  // 最大PWM值
#define BASE_SPEED_STRAIGHT   600  // 直线行走基础速度
#define TURN_SPEED            400  // 转弯基础速度
#define TURN_THRESHOLD        2.0f // 转弯完成的角度阈值(度)
#define ACC_STEP              10   // 加速步长
#define DEC_STEP              20   // 减速步长

// 系统状态相关常量
#define ERR_NONE              0x00 // 无错误
#define ERR_MPU_INIT_FAILED   0x01 // MPU6050初始化失败
#define ERR_MPU_READ_FAILED   0x02 // MPU6050读取数据失败
#define ERR_MOTOR_CTRL_FAILED 0x03 // 电机控制失败
#define ERR_SYSTEM_UNSTABLE   0x04 // 系统不稳定(倾角过大)

// 全局变量
// 传感器数据
typedef struct {
    int16_t accX, accY, accZ;     // 加速度计原始数据
    int16_t gyroX, gyroY, gyroZ;  // 陀螺仪原始数据
    float pitch;                  // 俯仰角(前后倾角)
    float roll;                   // 横滚角(左右倾角)
    float yaw;                    // 航向角(水平旋转角)
    float temperature;            // 温度
} MPU6050_Data_TypeDef;

// 卡尔曼滤波结构体
typedef struct {
    float Q_angle;    // 过程噪声协方差
    float Q_bias;     // 过程噪声协方差
    float R_measure;  // 测量噪声协方差
    
    float angle;      // 角度
    float bias;       // 角速度偏差
    float rate;       // 角速度
    
    float P[2][2];    // 协方差矩阵
} Kalman_Filter_TypeDef;

// PID控制结构体
typedef struct {
    float Kp, Ki, Kd;       // PID参数
    float error;            // 当前误差
    float last_error;       // 上一次误差
    float error_sum;        // 误差积分
    float error_diff;       // 误差微分
    float output;           // PID输出
    float output_max;       // 输出限幅
    float deadband;         // 死区范围
} PID_TypeDef;

// 电机控制结构体
typedef struct {
    uint16_t left_target;   // 左电机目标速度
    uint16_t right_target;  // 右电机目标速度
    uint16_t left_current;  // 左电机当前速度
    uint16_t right_current; // 右电机当前速度
    uint8_t direction;      // 方向控制: 0-停止, 1-前进, 2-后退, 3-左转, 4-右转
} Motor_Control_TypeDef;

// 系统状态结构体
typedef struct {
    uint8_t mode;           // 系统模式
    uint8_t error_code;     // 错误代码
    uint8_t battery_level;  // 电池电量(百分比)
    uint8_t is_calibrated;  // 是否已校准
    uint32_t run_time;      // 系统运行时间(ms)
    uint8_t debug_mode;     // 调试模式开关
} System_Status_TypeDef;

// 全局变量定义
MPU6050_Data_TypeDef mpu6050_data;
Kalman_Filter_TypeDef kalman_pitch, kalman_roll;
PID_TypeDef pid_straight, pid_turn;
Motor_Control_TypeDef motor_control;
System_Status_TypeDef system_status;

// 传感器零点偏移补偿值
float acc_offset[3] = {0, 0, 0};
float gyro_offset[3] = {0, 0, 0};

// 功能函数声明
void System_Init(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void I2C_Configuration(void);
void TIM_Configuration(void);
void USART_Configuration(void);
void Delay_Configuration(void);

uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadData(void);
void MPU6050_Calibrate(void);
void Calculate_Angle(void);

void Kalman_Filter_Init(Kalman_Filter_TypeDef *kalman, float Q_angle, float Q_bias, float R_measure);
float Kalman_Filter_Update(Kalman_Filter_TypeDef *kalman, float newAngle, float newRate, float dt);

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_max, float deadband);
float PID_Calculate(PID_TypeDef *pid, float target, float current);

void Motor_Init(void);
void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed);
void Motor_SetDirection(uint8_t direction);
void Motor_Control_Update(void);
void Motor_Stop(void);
void Motor_Smooth_Control(uint16_t left_target, uint16_t right_target);

void Go_Straight(uint16_t speed);
void Turn_Angle(float angle);
void Turn_Left_90(void);
void Turn_Right_90(void);
void System_Control_Loop(void);
void System_Error_Handler(uint8_t error_code);

uint8_t I2C_WriteOneByte(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
uint8_t I2C_ReadOneByte(uint8_t deviceAddr, uint8_t regAddr);
void I2C_ReadMultiBytes(uint8_t deviceAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

void USART_SendByte(uint8_t byte);
void USART_SendString(char *str);
void Debug_Print(const char *format, ...);

// 主函数
int main(void)
{
    // 初始化系统
    System_Init();
    
    // 延时等待MPU6050稳定
    delay_ms(200);
    
    // 校准MPU6050
    if (system_status.debug_mode) {
        Debug_Print("Calibrating MPU6050...\r\n");
    }
    MPU6050_Calibrate();
    system_status.is_calibrated = 1;
    
    if (system_status.debug_mode) {
        Debug_Print("System initialized and ready!\r\n");
    }
    
    // 启动系统
    system_status.mode = SYS_MODE_STANDBY;
    
    // 等待2秒后开始
    delay_ms(2000);
    
    // 演示模式：直线行走5秒，右转90度，直线行走3秒，左转90度，直线行走5秒
    if (system_status.debug_mode) {
        Debug_Print("Starting demo mode...\r\n");
    }
    
    // 直线行走5秒
    system_status.mode = SYS_MODE_FORWARD;
    uint32_t start_time = delay_read_ms();
    while (delay_read_ms() - start_time < 5000) {
        System_Control_Loop();
        delay_ms(10);
    }
    
    // 右转90度
    if (system_status.debug_mode) {
        Debug_Print("Turning right 90 degrees...\r\n");
    }
    Turn_Right_90();
    
    // 直线行走3秒
    system_status.mode = SYS_MODE_FORWARD;
    start_time = delay_read_ms();
    while (delay_read_ms() - start_time < 3000) {
        System_Control_Loop();
        delay_ms(10);
    }
    
    // 左转90度
    if (system_status.debug_mode) {
        Debug_Print("Turning left 90 degrees...\r\n");
    }
    Turn_Left_90();
    
    // 直线行走5秒
    system_status.mode = SYS_MODE_FORWARD;
    start_time = delay_read_ms();
    while (delay_read_ms() - start_time < 5000) {
        System_Control_Loop();
        delay_ms(10);
    }
    
    // 停止
    Motor_Stop();
    system_status.mode = SYS_MODE_STANDBY;
    
    if (system_status.debug_mode) {
        Debug_Print("Demo completed!\r\n");
    }
    
    // 主循环
    while (1) {
        System_Control_Loop();
        delay_ms(10);
    }
}

// 系统初始化
void System_Init(void)
{
    // 初始化系统时钟
    SystemInit();
    
    // 配置RCC
    RCC_Configuration();
    
    // 初始化延时函数
    Delay_Configuration();
    
    // 配置GPIO
    GPIO_Configuration();
    
    // 配置NVIC
    NVIC_Configuration();
    
    // 配置I2C
    I2C_Configuration();
    
    // 配置TIM
    TIM_Configuration();
    
    // 配置USART
    USART_Configuration();
    
    // 初始化MPU6050
    if (MPU6050_Init() != 0) {
        System_Error_Handler(ERR_MPU_INIT_FAILED);
    }
    
    // 初始化卡尔曼滤波器
    Kalman_Filter_Init(&kalman_pitch, Q_ANGLE, Q_GYRO, R_ANGLE);
    Kalman_Filter_Init(&kalman_roll, Q_ANGLE, Q_GYRO, R_ANGLE);
    
    // 初始化PID控制器
    PID_Init(&pid_straight, KP_STRAIGHT, KI_STRAIGHT, KD_STRAIGHT, 300, 1.0f);
    PID_Init(&pid_turn, KP_TURN, KI_TURN, KD_TURN, 300, 0.5f);
    
    // 初始化电机控制
    Motor_Init();
    
    // 初始化系统状态
    system_status.mode = SYS_MODE_STANDBY;
    system_status.error_code = ERR_NONE;
    system_status.battery_level = 100;  // 假设电池满电
    system_status.is_calibrated = 0;
    system_status.run_time = 0;
    system_status.debug_mode = 1;  // 默认开启调试模式
}

// 配置RCC
void RCC_Configuration(void)
{
    // 复位所有RCC外设寄存器
    RCC_DeInit();
    
    // 开启HSE
    RCC_HSEConfig(RCC_HSE_ON);
    
    // 等待HSE稳定
    if (RCC_WaitForHSEStartUp() == SUCCESS) {
        // 配置HCLK, PCLK1, PCLK2
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        
        // 配置PLL
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);  // 72MHz
        
        // 使能PLL
        RCC_PLLCmd(ENABLE);
        
        // 等待PLL稳定
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        
        // 选择PLL作为系统时钟源
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        
        // 等待PLL成为系统时钟源
        while (RCC_GetSYSCLKSource() != 0x08);
    }
    
    // 启用I2C1外设时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    
    // 启用GPIOA、GPIOB外设时钟和复用功能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    
    // 启用定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    // 启用USART1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
}

// 配置GPIO
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 配置I2C1引脚(SCL: PB10, SDA: PB11)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;  // 复用开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 配置电机控制引脚(IN1-IN4: PB5-PB8)
    GPIO_InitStructure.GPIO_Pin = MOTOR_LEFT_FORWARD | MOTOR_LEFT_BACKWARD | 
                                  MOTOR_RIGHT_FORWARD | MOTOR_RIGHT_BACKWARD;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 配置电机使能引脚(ENA: PA8, ENB: PA9)为复用推挽输出，用于PWM输出
    GPIO_InitStructure.GPIO_Pin = MOTOR_LEFT_ENABLE | MOTOR_RIGHT_ENABLE;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置USART1引脚(TX: PA9, RX: PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // 浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置LED指示灯(PC13)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // 初始状态：LED关闭
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

// 配置NVIC
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 配置NVIC组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // 配置I2C1中断
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    // 配置USART1中断
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    
    // 配置TIM2中断(用于系统定时)
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

// 配置I2C
void I2C_Configuration(void)
{
    I2C_InitTypeDef I2C_InitStructure;
    
    // 复位I2C1
    I2C_DeInit(I2C1);
    
    // 配置I2C1
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;  // 400kHz
    
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}

// 配置定时器(用于PWM输出和系统定时)
void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 配置TIM1，用于产生PWM信号
    TIM_TimeBaseStructure.TIM_Period = 999;  // ARR值，PWM周期为1000
    TIM_TimeBaseStructure.TIM_Prescaler = 71;  // 72分频，72MHz/72=1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    // 配置PWM通道1 (ENA - PA8)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;  // 初始占空比为0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    
    // 配置PWM通道2 (ENB - PA9)
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;  // 初始占空比为0
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);  // 高级定时器需要使能主输出
    TIM_Cmd(TIM1, ENABLE);
    
    // 配置TIM2，用于系统定时(10ms中断)
    TIM_TimeBaseStructure.TIM_Period = 9999;  // ARR值
    TIM_TimeBaseStructure.TIM_Prescaler = 71;  // 72分频，72MHz/72=1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

// 配置USART
void USART_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    
    // 配置USART1
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

// 配置延时函数
void Delay_Configuration(void)
{
    // 调用库函数初始化延时
    delay_init();
}

// MPU6050初始化
uint8_t MPU6050_Init(void)
{
    uint8_t res;
    
    // 复位MPU6050
    I2C_WriteOneByte(MPU6050_ADDR, MPU_PWR_MGMT1_REG, 0x80);
    delay_ms(100);
    
    // 唤醒MPU6050
    I2C_WriteOneByte(MPU6050_ADDR, MPU_PWR_MGMT1_REG, 0x00);
    
    // 设置陀螺仪满量程范围：±2000dps
    I2C_WriteOneByte(MPU6050_ADDR, MPU_GYRO_CFG_REG, 0x18);
    
    // 设置加速度计满量程范围：±2g
    I2C_WriteOneByte(MPU6050_ADDR, MPU_ACCEL_CFG_REG, 0x00);
    
    // 设置采样率为100Hz
    I2C_WriteOneByte(MPU6050_ADDR, MPU_SAMPLE_RATE_REG, 0x09);
    
    //    // 设置数字低通滤波器
    I2C_WriteOneByte(MPU6050_ADDR, MPU_CONFIG_REG, 0x04);
    
    // 使能陀螺仪和加速度计
    I2C_WriteOneByte(MPU6050_ADDR, MPU_PWR_MGMT1_REG, 0x01);
    
    // 使能FIFO
    I2C_WriteOneByte(MPU6050_ADDR, MPU_FIFO_EN_REG, 0x00);
    
    // 检查设备ID是否正确
    res = I2C_ReadOneByte(MPU6050_ADDR, 0x75);
    if(res != 0x68) {
        if (system_status.debug_mode) {
            Debug_Print("MPU6050 ID ERROR: 0x%02X\r\n", res);
        }
        return 1;
    }
    
    if (system_status.debug_mode) {
        Debug_Print("MPU6050 ID: 0x%02X [OK]\r\n", res);
    }
    
    return 0;
}

// MPU6050数据读取
uint8_t MPU6050_ReadData(void)
{
    uint8_t buf[14];
    
    // 一次性读取所有传感器数据
    I2C_ReadMultiBytes(MPU6050_ADDR, MPU_ACCEL_XOUTH_REG, 14, buf);
    
    // 合成加速度计数据
    mpu6050_data.accX = ((int16_t)buf[0] << 8) | buf[1];
    mpu6050_data.accY = ((int16_t)buf[2] << 8) | buf[3];
    mpu6050_data.accZ = ((int16_t)buf[4] << 8) | buf[5];
    
    // 温度
    mpu6050_data.temperature = (((int16_t)buf[6] << 8) | buf[7]) / 340.0f + 36.53f;
    
    // 合成陀螺仪数据
    mpu6050_data.gyroX = ((int16_t)buf[8] << 8) | buf[9];
    mpu6050_data.gyroY = ((int16_t)buf[10] << 8) | buf[11];
    mpu6050_data.gyroZ = ((int16_t)buf[12] << 8) | buf[13];
    
    // 减去零点偏移
    mpu6050_data.accX -= (int16_t)acc_offset[0];
    mpu6050_data.accY -= (int16_t)acc_offset[1];
    mpu6050_data.accZ -= (int16_t)acc_offset[2];
    
    mpu6050_data.gyroX -= (int16_t)gyro_offset[0];
    mpu6050_data.gyroY -= (int16_t)gyro_offset[1];
    mpu6050_data.gyroZ -= (int16_t)gyro_offset[2];
    
    return 0;
}

// MPU6050校准
void MPU6050_Calibrate(void)
{
    int32_t acc_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};
    int i;
    
    // 读取100次数据，求平均值
    for(i = 0; i < 100; i++) {
        uint8_t buf[14];
        
        I2C_ReadMultiBytes(MPU6050_ADDR, MPU_ACCEL_XOUTH_REG, 14, buf);
        
        acc_sum[0] += ((int16_t)buf[0] << 8) | buf[1];
        acc_sum[1] += ((int16_t)buf[2] << 8) | buf[3];
        acc_sum[2] += ((int16_t)buf[4] << 8) | buf[5];
        
        gyro_sum[0] += ((int16_t)buf[8] << 8) | buf[9];
        gyro_sum[1] += ((int16_t)buf[10] << 8) | buf[11];
        gyro_sum[2] += ((int16_t)buf[12] << 8) | buf[13];
        
        delay_ms(5);
    }
    
    // 求平均值
    acc_offset[0] = acc_sum[0] / 100;
    acc_offset[1] = acc_sum[1] / 100;
    // Z轴校准时需要减去1g (16384 LSB)
    acc_offset[2] = acc_sum[2] / 100 - 16384;
    
    gyro_offset[0] = gyro_sum[0] / 100;
    gyro_offset[1] = gyro_sum[1] / 100;
    gyro_offset[2] = gyro_sum[2] / 100;
    
    if (system_status.debug_mode) {
        Debug_Print("ACC Offset: %d, %d, %d\r\n", (int)acc_offset[0], (int)acc_offset[1], (int)acc_offset[2]);
        Debug_Print("GYRO Offset: %d, %d, %d\r\n", (int)gyro_offset[0], (int)gyro_offset[1], (int)gyro_offset[2]);
    }
}

// 角度计算
void Calculate_Angle(void)
{
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float pitch, roll;
    static uint32_t last_time = 0;
    uint32_t now = delay_read_ms();
    float dt = (now - last_time) / 1000.0f;
    last_time = now;
    
    // 确保dt的合理性
    if(dt < 0.001f || dt > 0.5f) dt = 0.01f;
    
    // 将原始数据转换为实际物理量
    accX = mpu6050_data.accX / 16384.0f;  // ±2g量程下，16384 LSB/g
    accY = mpu6050_data.accY / 16384.0f;
    accZ = mpu6050_data.accZ / 16384.0f;
    
    gyroX = mpu6050_data.gyroX / 16.4f;  // ±2000°/s量程下，16.4 LSB/(°/s)
    gyroY = mpu6050_data.gyroY / 16.4f;
    gyroZ = mpu6050_data.gyroZ / 16.4f;
    
    // 加速度计计算俯仰角(pitch)和横滚角(roll)
    pitch = atan2(accX, sqrt(accY * accY + accZ * accZ)) * 57.296f;  // 弧度转角度(乘以180/π)
    roll = atan2(accY, accZ) * 57.296f;
    
    // 使用卡尔曼滤波融合加速度计和陀螺仪数据
    mpu6050_data.pitch = Kalman_Filter_Update(&kalman_pitch, pitch, gyroY, dt);
    mpu6050_data.roll = Kalman_Filter_Update(&kalman_roll, roll, gyroX, dt);
    
    // 航向角(yaw)单独处理，仅使用陀螺仪Z轴数据积分
    mpu6050_data.yaw += gyroZ * dt;
    
    // 限制航向角在-180~180范围内
    if (mpu6050_data.yaw > 180) mpu6050_data.yaw -= 360;
    if (mpu6050_data.yaw < -180) mpu6050_data.yaw += 360;
    
    // 如果倾角过大，视为系统不稳定
    if (fabs(mpu6050_data.pitch) > 45 || fabs(mpu6050_data.roll) > 45) {
        System_Error_Handler(ERR_SYSTEM_UNSTABLE);
    }
}

// 卡尔曼滤波器初始化
void Kalman_Filter_Init(Kalman_Filter_TypeDef *kalman, float Q_angle, float Q_bias, float R_measure)
{
    kalman->Q_angle = Q_angle;
    kalman->Q_bias = Q_bias;
    kalman->R_measure = R_measure;
    
    kalman->angle = 0.0f;
    kalman->bias = 0.0f;
    
    kalman->P[0][0] = 1.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 1.0f;
}

// 卡尔曼滤波器更新
float Kalman_Filter_Update(Kalman_Filter_TypeDef *kalman, float newAngle, float newRate, float dt)
{
    float rate = newRate - kalman->bias;
    kalman->angle += dt * rate;
    
    // 更新估计误差协方差矩阵
    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;
    
    // 计算卡尔曼增益
    float S = kalman->P[0][0] + kalman->R_measure;
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;
    
    // 计算角度和偏差的估计
    float y = newAngle - kalman->angle;
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;
    
    // 更新估计误差协方差矩阵
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];
    
    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;
    
    return kalman->angle;
}

// PID控制器初始化
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_max, float deadband)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_max = output_max;
    pid->deadband = deadband;
    
    pid->error = 0;
    pid->last_error = 0;
    pid->error_sum = 0;
    pid->error_diff = 0;
    pid->output = 0;
}

// PID控制器计算
float PID_Calculate(PID_TypeDef *pid, float target, float current)
{
    // 计算误差
    pid->error = target - current;
    
    // 应用死区
    if (fabs(pid->error) < pid->deadband) {
        pid->error = 0;
    }
    
    // 计算误差微分
    pid->error_diff = pid->error - pid->last_error;
    
    // 计算误差积分(带抗积分饱和)
    pid->error_sum += pid->error;
    
    // 积分限幅
    float integral_limit = pid->output_max / pid->Ki;
    if (pid->error_sum > integral_limit) 
        pid->error_sum = integral_limit;
    else if (pid->error_sum < -integral_limit) 
        pid->error_sum = -integral_limit;
    
    // PID计算输出
    pid->output = pid->Kp * pid->error + pid->Ki * pid->error_sum + pid->Kd * pid->error_diff;
    
    // 输出限幅
    if (pid->output > pid->output_max) 
        pid->output = pid->output_max;
    else if (pid->output < -pid->output_max) 
        pid->output = -pid->output_max;
    
    // 保存当前误差
    pid->last_error = pid->error;
    
    return pid->output;
}

// 电机初始化
void Motor_Init(void)
{
    // 初始化电机控制结构体
    motor_control.left_target = 0;
    motor_control.right_target = 0;
    motor_control.left_current = 0;
    motor_control.right_current = 0;
    motor_control.direction = 0;  // 停止
    
    // 初始状态：电机停止
    GPIO_ResetBits(GPIOB, MOTOR_LEFT_FORWARD | MOTOR_LEFT_BACKWARD | 
                         MOTOR_RIGHT_FORWARD | MOTOR_RIGHT_BACKWARD);
    
    // 设置PWM为0
    Motor_SetSpeed(0, 0);
}

// 设置电机速度
void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed)
{
    // 限制速度范围
    if (left_speed > MAX_MOTOR_SPEED) left_speed = MAX_MOTOR_SPEED;
    if (right_speed > MAX_MOTOR_SPEED) right_speed = MAX_MOTOR_SPEED;
    
    // 设置PWM占空比
    TIM_SetCompare1(TIM1, left_speed);  // 左电机
    TIM_SetCompare2(TIM1, right_speed); // 右电机
    
    // 更新当前速度
    motor_control.left_current = left_speed;
    motor_control.right_current = right_speed;
    
    if (system_status.debug_mode) {
        static uint32_t last_debug_time = 0;
        uint32_t now = delay_read_ms();
        
        // 每500ms输出一次调试信息
        if (now - last_debug_time > 500) {
            Debug_Print("Motor Speed L:%d R:%d\r\n", left_speed, right_speed);
            last_debug_time = now;
        }
    }
}

// 设置电机方向
void Motor_SetDirection(uint8_t direction)
{
    motor_control.direction = direction;
    
    switch (direction) {
        case 0:  // 停止
            GPIO_ResetBits(GPIOB, MOTOR_LEFT_FORWARD | MOTOR_LEFT_BACKWARD | 
                                 MOTOR_RIGHT_FORWARD | MOTOR_RIGHT_BACKWARD);
            break;
        
        case 1:  // 前进
            GPIO_SetBits(GPIOB, MOTOR_LEFT_FORWARD | MOTOR_RIGHT_FORWARD);
            GPIO_ResetBits(GPIOB, MOTOR_LEFT_BACKWARD | MOTOR_RIGHT_BACKWARD);
            break;
        
        case 2:  // 后退
            GPIO_SetBits(GPIOB, MOTOR_LEFT_BACKWARD | MOTOR_RIGHT_BACKWARD);
            GPIO_ResetBits(GPIOB, MOTOR_LEFT_FORWARD | MOTOR_RIGHT_FORWARD);
            break;
        
        case 3:  // 左转
            GPIO_SetBits(GPIOB, MOTOR_LEFT_BACKWARD | MOTOR_RIGHT_FORWARD);
            GPIO_ResetBits(GPIOB, MOTOR_LEFT_FORWARD | MOTOR_RIGHT_BACKWARD);
            break;
        
        case 4:  // 右转
            GPIO_SetBits(GPIOB, MOTOR_LEFT_FORWARD | MOTOR_RIGHT_BACKWARD);
            GPIO_ResetBits(GPIOB, MOTOR_LEFT_BACKWARD | MOTOR_RIGHT_FORWARD);
            break;
    }
}

// 电机控制更新
void Motor_Control_Update(void)
{
    // 平滑控制电机速度
    Motor_Smooth_Control(motor_control.left_target, motor_control.right_target);
}

// 停止电机
void Motor_Stop(void)
{
    Motor_SetDirection(0);  // 停止
    motor_control.left_target = 0;
    motor_control.right_target = 0;
    Motor_SetSpeed(0, 0);
}

// 平滑控制电机速度
void Motor_Smooth_Control(uint16_t left_target, uint16_t right_target)
{
    uint16_t left_current = motor_control.left_current;
    uint16_t right_current = motor_control.right_current;
    
    // 平滑增加或减少左电机速度
    if (left_current < left_target) {
        left_current += ACC_STEP;
        if (left_current > left_target) left_current = left_target;
    } else if (left_current > left_target) {
        left_current -= DEC_STEP;
        if (left_current < left_target) left_current = left_target;
    }
    
    // 平滑增加或减少右电机速度
    if (right_current < right_target) {
        right_current += ACC_STEP;
        if (right_current > right_target) right_current = right_target;
    } else if (right_current > right_target) {
        right_current -= DEC_STEP;
        if (right_current < right_target) right_current = right_target;
    }
    
    // 更新电机速度
    Motor_SetSpeed(left_current, right_current);
}

// 直线行走
void Go_Straight(uint16_t speed)
{
    // 设置方向为前进
    Motor_SetDirection(1);
    
    // 计算PID输出，基于俯仰角保持直线
    float correction = PID_Calculate(&pid_straight, 0, mpu6050_data.roll);
    
    // 调整左右电机速度保持直线
    uint16_t left_speed = speed - (int16_t)correction;
    uint16_t right_speed = speed + (int16_t)correction;
    
    // 确保速度不为负
    if (left_speed < 0) left_speed = 0;
    if (right_speed < 0) right_speed = 0;
    
    // 设置目标速度
    motor_control.left_target = left_speed;
    motor_control.right_target = right_speed;
}

// 精确转向控制
void Turn_Angle(float angle)
{
    float start_yaw = mpu6050_data.yaw;
    float target_yaw = start_yaw + angle;
    
    // 规范化目标航向角到[-180, 180]
    if (target_yaw > 180) target_yaw -= 360;
    if (target_yaw < -180) target_yaw += 360;
    
    // 选择最短的转向方向
    uint8_t direction;
    float diff = target_yaw - start_yaw;
    
    // 处理角度环绕
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    
    if (diff > 0) {
        // 右转
        direction = 4;
    } else {
        // 左转
        direction = 3;
    }
    
    // 更新系统模式
    system_status.mode = SYS_MODE_TURNING;
    
    // 设置转向方向
    Motor_SetDirection(direction);
    
    // 设置适当的转向速度
    uint16_t turn_speed = TURN_SPEED;
    motor_control.left_target = turn_speed;
    motor_control.right_target = turn_speed;
    
    if (system_status.debug_mode) {
        Debug_Print("Turning from %.1f to %.1f (diff: %.1f)\r\n", start_yaw, target_yaw, diff);
    }
    
    // 等待转向完成
    while (1) {
        // 更新传感器数据
        MPU6050_ReadData();
        Calculate_Angle();
        
        // 计算当前航向角与目标航向角的差异
        float current_diff = target_yaw - mpu6050_data.yaw;
        
        // 处理角度环绕
        if (current_diff > 180) current_diff -= 360;
        if (current_diff < -180) current_diff += 360;
        
        // 计算剩余角度的绝对值
        float abs_diff = fabs(current_diff);
        
        // 如果接近目标角度，减速
        if (abs_diff < 20) {
            turn_speed = TURN_SPEED / 2;
            motor_control.left_target = turn_speed;
            motor_control.right_target = turn_speed;
        }
        
        // 如果非常接近目标角度，进一步减速
        if (abs_diff < 10) {
            turn_speed = TURN_SPEED / 4;
            motor_control.left_target = turn_speed;
            motor_control.right_target = turn_speed;
        }
        
        // 更新电机控制
        Motor_Control_Update();
        
        // 如果达到目标角度，停止转向
        if (abs_diff < TURN_THRESHOLD) {
            break;
        }
        
        delay_ms(10);
    }
    
    // 停止电机
    Motor_Stop();
    
    // 短暂停顿
    delay_ms(500);
    
    if (system_status.debug_mode) {
        Debug_Print("Turn completed, current yaw: %.1f\r\n", mpu6050_data.yaw);
    }
}

// 左转90度
void Turn_Left_90(void)
{
    Turn_Angle(-90.0f);
}

// 右转90度
void Turn_Right_90(void)
{
    Turn_Angle(90.0f);
}

// 系统控制循环
void System_Control_Loop(void)
{
    // 更新系统运行时间
    system_status.run_time = delay_read_ms();
    
    // 读取MPU6050数据
    if (MPU6050_ReadData() != 0) {
        System_Error_Handler(ERR_MPU_READ_FAILED);
        return;
    }
    
    // 计算姿态角
    Calculate_Angle();
    
    // 根据系统模式执行不同的控制策略
    switch (system_status.mode) {
        case SYS_MODE_STANDBY:
            // 待机模式：电机停止
            Motor_Stop();
            break;
        
        case SYS_MODE_FORWARD:
            // 直线前进模式：根据姿态调整电机速度
            Go_Straight(BASE_SPEED_STRAIGHT);
            break;
        
        case SYS_MODE_TURNING:
            // 转弯模式：在Turn_Angle函数中单独处理
            break;
        
        case SYS_MODE_CALIBRATION:
            // 校准模式：在MPU6050_Calibrate函数中单独处理
            break;
        
        case SYS_MODE_TESTING:
            // 测试模式：闪烁LED指示灯
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)((1-GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))));
            delay_ms(200);
            break;
    }
    
    // 更新电机控制
    Motor_Control_Update();
}

// 系统错误处理
void System_Error_Handler(uint8_t error_code)
{
    // 更新错误代码
    system_status.error_code = error_code;
    
    // 紧急停止电机
    Motor_Stop();
    
    if (system_status.debug_mode) {
        switch (error_code) {
            case ERR_MPU_INIT_FAILED:
                Debug_Print("ERROR: MPU6050 initialization failed!\r\n");
                break;
            
            case ERR_MPU_READ_FAILED:
                Debug_Print("ERROR: MPU6050 data read failed!\r\n");
                break;
            
            case ERR_MOTOR_CTRL_FAILED:
                Debug_Print("ERROR: Motor control failed!\r\n");
                break;
            
            case ERR_SYSTEM_UNSTABLE:
                Debug_Print("ERROR: System unstable! Angle too large!\r\n");
                break;
            
            default:
                Debug_Print("ERROR: Unknown error (code: %d)!\r\n", error_code);
                break;
        }
    }
    
    // 如果是严重错误，进入死循环并闪烁LED指示灯
    if (error_code == ERR_MPU_INIT_FAILED || error_code == ERR_SYSTEM_UNSTABLE) {
        while (1) {
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)((1-GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))));
            delay_ms(200);
        }
    }
}

// I2C写一个字节
uint8_t I2C_WriteOneByte(uint8_t deviceAddr, uint8_t regAddr, uint8_t data)
{
    uint32_t timeout = 10000;
    
    // 等待I2C总线空闲
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 1;
    
    // 发送起始信号
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 2;
    
    // 发送设备地址+写命令
    I2C_Send7bitAddress(I2C1, deviceAddr, I2C_Direction_Transmitter);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 3;
    
    // 发送寄存器地址
    I2C_SendData(I2C1, regAddr);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 4;
    
    // 发送数据
    I2C_SendData(I2C1, data);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 5;
    
    // 发送停止信号
    I2C_GenerateSTOP(I2C1, ENABLE);
    
    return 0;
}

// I2C读一个字节
uint8_t I2C_ReadOneByte(uint8_t deviceAddr, uint8_t regAddr)
{
    uint8_t data;
    uint32_t timeout = 10000;
    
    // 等待I2C总线空闲
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 0;
    
    // 发送起始信号
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 0;
    
    // 发送设备地址+写命令
    I2C_Send7bitAddress(I2C1, deviceAddr, I2C_Direction_Transmitter);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 0;
    
    // 发送寄存器地址
    I2C_SendData(I2C1, regAddr);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 0;
    
    // 发送重新起始信号
    I2C_GenerateSTART(I2C1, ENABLE);
        timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 0;
    
    // 发送设备地址+读命令
    I2C_Send7bitAddress(I2C1, deviceAddr, I2C_Direction_Receiver);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 0;
    
    // 禁用自动应答
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    
    // 发送停止信号
    I2C_GenerateSTOP(I2C1, ENABLE);
    
    // 等待接收数据
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) && timeout) {
        timeout--;
    }
    if (timeout == 0) return 0;
    
    // 读取数据
    data = I2C_ReceiveData(I2C1);
    
    // 重新启用自动应答
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    
    return data;
}

// I2C连续读多个字节
void I2C_ReadMultiBytes(uint8_t deviceAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    uint32_t timeout = 10000;
    uint8_t i;
    
    // 等待I2C总线空闲
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) && timeout) {
        timeout--;
    }
    if (timeout == 0) return;
    
    // 发送起始信号
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && timeout) {
        timeout--;
    }
    if (timeout == 0) return;
    
    // 发送设备地址+写命令
    I2C_Send7bitAddress(I2C1, deviceAddr, I2C_Direction_Transmitter);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout) {
        timeout--;
    }
    if (timeout == 0) return;
    
    // 发送寄存器地址
    I2C_SendData(I2C1, regAddr);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout) {
        timeout--;
    }
    if (timeout == 0) return;
    
    // 发送重新起始信号
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && timeout) {
        timeout--;
    }
    if (timeout == 0) return;
    
    // 发送设备地址+读命令
    I2C_Send7bitAddress(I2C1, deviceAddr, I2C_Direction_Receiver);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout) {
        timeout--;
    }
    if (timeout == 0) return;
    
    // 启用自动应答
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    
    // 读取数据
    for (i = 0; i < length; i++) {
        if (i == length - 1) {
            // 最后一个字节，禁用自动应答
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            // 发送停止信号
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        
        // 等待接收数据
        timeout = 10000;
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) && timeout) {
            timeout--;
        }
        if (timeout == 0) return;
        
        // 读取数据
        data[i] = I2C_ReceiveData(I2C1);
    }
    
    // 重新启用自动应答
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

// USART发送一个字节
void USART_SendByte(uint8_t byte)
{
    // 等待发送缓冲区为空
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    
    // 发送数据
    USART_SendData(USART1, byte);
}

// USART发送字符串
void USART_SendString(char *str)
{
    while (*str) {
        USART_SendByte(*str++);
    }
}

// 调试输出函数
void Debug_Print(const char *format, ...)
{
    if (!system_status.debug_mode) return;
    
    static char buffer[256];
    va_list args;
    
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);
    
    USART_SendString(buffer);
}

// TIM2中断处理函数(10ms定时)
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // 每10ms执行一次控制循环
        if (system_status.mode != SYS_MODE_STANDBY && 
            system_status.mode != SYS_MODE_CALIBRATION) {
            // 更新传感器数据
            MPU6050_ReadData();
            Calculate_Angle();
            
            // LED指示灯闪烁(500ms周期)
            if ((system_status.run_time % 500) < 250) {
                GPIO_SetBits(GPIOC, GPIO_Pin_13);
            } else {
                GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            }
        }
    }
}

// I2C1事件中断处理函数
void I2C1_EV_IRQHandler(void)
{
    // 这里可以处理I2C事件中断
}

// I2C1错误中断处理函数
void I2C1_ER_IRQHandler(void)
{
    // 清除错误标志
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
        I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    }
    
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BERR)) {
        I2C_ClearFlag(I2C1, I2C_FLAG_BERR);
    }
    
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_OVR)) {
        I2C_ClearFlag(I2C1, I2C_FLAG_OVR);
    }
    
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_ARLO)) {
        I2C_ClearFlag(I2C1, I2C_FLAG_ARLO);
    }
}

// USART1中断处理函数
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        // 接收数据
        uint8_t data = USART_ReceiveData(USART1);
        
        // 简单的命令处理
        switch (data) {
            case 'f':  // 前进
                system_status.mode = SYS_MODE_FORWARD;
                Debug_Print("Command: Forward\r\n");
                break;
            
            case 's':  // 停止
                system_status.mode = SYS_MODE_STANDBY;
                Debug_Print("Command: Stop\r\n");
                break;
            
            case 'l':  // 左转
                if (system_status.mode != SYS_MODE_TURNING) {
                    Debug_Print("Command: Turn Left 90 degrees\r\n");
                    Turn_Left_90();
                    system_status.mode = SYS_MODE_FORWARD;
                }
                break;
            
            case 'r':  // 右转
                if (system_status.mode != SYS_MODE_TURNING) {
                    Debug_Print("Command: Turn Right 90 degrees\r\n");
                    Turn_Right_90();
                    system_status.mode = SYS_MODE_FORWARD;
                }
                break;
            
            case 'c':  // 校准
                Debug_Print("Command: Calibrating MPU6050...\r\n");
                system_status.mode = SYS_MODE_CALIBRATION;
                MPU6050_Calibrate();
                system_status.is_calibrated = 1;
                system_status.mode = SYS_MODE_STANDBY;
                Debug_Print("Calibration completed\r\n");
                break;
            
            case 'd':  // 切换调试模式
                system_status.debug_mode = !system_status.debug_mode;
                if (system_status.debug_mode) {
                    Debug_Print("Debug mode enabled\r\n");
                } else {
                    USART_SendString("Debug mode disabled\r\n");
                }
                break;
            
            case 't':  // 测试模式
                system_status.mode = SYS_MODE_TESTING;
                Debug_Print("Command: Test Mode\r\n");
                break;
            
            case 'i':  // 查询系统信息
                Debug_Print("\r\n--- System Info ---\r\n");
                Debug_Print("Mode: %d\r\n", system_status.mode);
                Debug_Print("Error: %d\r\n", system_status.error_code);
                Debug_Print("Battery: %d%%\r\n", system_status.battery_level);
                Debug_Print("Run Time: %lu ms\r\n", system_status.run_time);
                Debug_Print("Calibrated: %d\r\n", system_status.is_calibrated);
                Debug_Print("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n", 
                           mpu6050_data.pitch, mpu6050_data.roll, mpu6050_data.yaw);
                Debug_Print("Motor L: %d, R: %d\r\n", 
                           motor_control.left_current, motor_control.right_current);
                break;
        }
    }
}

// 自定义的延时函数，基于SysTick
void delay_ms(uint32_t ms)
{
    uint32_t i;
    
    SysTick_Config(SystemCoreClock / 1000);
    
    for(i = 0; i < ms; i++) {
        // 等待SysTick中断发生
        while(!((SysTick->CTRL) & (1 << 16)));
    }
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

// 获取系统运行时间(毫秒)
uint32_t delay_read_ms(void)
{
    static uint32_t tick_count = 0;
    
    // 这里简单地增加tick计数，实际应用中可以使用定时器或SysTick来获取精确的时间
    tick_count += 10;  // 假设每次调用间隔10ms
    
    return tick_count;
}

// 延时初始化
void delay_init(void)
{
    // 配置SysTick
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}