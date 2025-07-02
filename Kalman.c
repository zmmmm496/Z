#include "Kalman.h"

void KalmanFilter_Init(KalmanFilter_t *kf, float q, float r, float initial_value)
{
    kf->Q = q;                      // 过程噪声协方差
    kf->R = r;                      // 测量噪声协方差
    kf->P = 1.0;                    // 初始误差协方差
    kf->X = initial_value;          // 初始值
    kf->K = 0.0;                    // 卡尔曼增益
}

float KalmanFilter_Update(KalmanFilter_t *kf, float measurement)
{
    // 预测阶段
    kf->P += kf->Q;

    // 更新阶段
    kf->K = kf->P / (kf->P + kf->R);
    kf->X = kf->X + kf->K * (measurement - kf->X);
    kf->P = (1 - kf->K) * kf->P;

    return kf->X;
}
