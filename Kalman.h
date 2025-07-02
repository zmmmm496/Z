#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

typedef struct
{
    float Q;       // 过程噪声协方差
    float R;       // 测量噪声协方差
    float P;       // 误差协方差
    float K;       // 卡尔曼增益
    float X;       // 估计值
} KalmanFilter_t;

// 初始化卡尔曼滤波器
void KalmanFilter_Init(KalmanFilter_t *kf, float q, float r, float initial_value);

// 更新卡尔曼滤波器，并返回滤波后的值
float KalmanFilter_Update(KalmanFilter_t *kf, float measurement);

#endif
