#ifndef __MPU6050_H
#define __MPU6050_H

#include "Kalman.h"

// 声明卡尔曼滤波器实例（使用 extern 避免重复定义）
extern KalmanFilter_t kfAccX, kfAccY, kfAccZ;
extern KalmanFilter_t kfGyroX, kfGyroY, kfGyroZ;

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
