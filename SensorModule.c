#include "SensorModule.h"

// 模块内全局变量（使用 static 限制作用域）
static volatile float AngleX, AngleY, AngleZ;
static KalmanFilter_t kfAccX, kfAccY, kfAccZ;
static KalmanFilter_t kfGyroX, kfGyroY, kfGyroZ;

// 初始化模块
void SensorModule_Init(void) {
    OLED_Init();
    MPU6050_Init();
    Servo_Init();

    // 初始化卡尔曼滤波器
    KalmanFilter_Init(&kfAccX, 0.001f, 0.03f, 0.0f);
    KalmanFilter_Init(&kfAccY, 0.001f, 0.03f, 0.0f);
    KalmanFilter_Init(&kfAccZ, 0.001f, 0.03f, 0.0f);
    KalmanFilter_Init(&kfGyroX, 0.001f, 0.03f, 0.0f);
    KalmanFilter_Init(&kfGyroY, 0.001f, 0.03f, 0.0f);
    KalmanFilter_Init(&kfGyroZ, 0.001f, 0.03f, 0.0f);

    // 显示初始信息
    OLED_ShowString(1, 1, "ID:");
    uint8_t ID = MPU6050_GetID();
    OLED_ShowHexNum(1, 4, ID, 2);
    Servo_SetAngle(100);  // 初始舵机角度
}

// 模块运行逻辑（在TIM2中断中执行）
void SensorModule_Run(void) {
    int16_t AX, AY, AZ, GX, GY, GZ;
    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

    // 卡尔曼滤波
    int16_t filteredAX = (int16_t)KalmanFilter_Update(&kfAccX, (float)AX);
    int16_t filteredAY = (int16_t)KalmanFilter_Update(&kfAccY, (float)AY);
    int16_t filteredAZ = (int16_t)KalmanFilter_Update(&kfAccZ, (float)AZ);
    int16_t filteredGX = (int16_t)KalmanFilter_Update(&kfGyroX, (float)GX);
    int16_t filteredGY = (int16_t)KalmanFilter_Update(&kfGyroY, (float)GY);
    int16_t filteredGZ = (int16_t)KalmanFilter_Update(&kfGyroZ, (float)GZ);

    // 角度融合（互补滤波）
    AngleX = 0.98 * (AngleX + filteredGX * 0.01) + (1 - 0.98) * filteredAX;
    AngleY = 0.98 * (AngleY + filteredGY * 0.01) + (1 - 0.98) * filteredAY;
    AngleZ = 0.98 * (AngleZ + filteredGZ * 0.01) + (1 - 0.98) * filteredAZ;

    // 更新OLED显示
    OLED_ShowSignedNum(2, 1, (int16_t)AngleX, 5);
    OLED_ShowSignedNum(3, 1, (int16_t)AngleY, 5);
    OLED_ShowSignedNum(4, 1, (int16_t)AngleZ, 5);

    // 控制舵机
    if (AngleZ < 1755) {
        Servo_SetAngle(75);
    } else if (AngleZ > 1775) {
        Servo_SetAngle(105);
    }
}
