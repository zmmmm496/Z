#ifndef __ENCODER_H
#define __ENCODER_H


//int Incremental_PI (int Encoder,int Target);
//int TIM1_IRQHandler(void);

float Track_err(void);

int PID_out(float error,int Target);
void Final_Speed(int pid_out ,int base_speed);



#endif







