#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_Init(void);

int myabs(int a);
void Limit(int *motor_left,int *motor_right);
void Set_Speed(int motor_l ,int motor_r);

//void Motor_left_SetSpeed(int32_t Speed);
//void Motor_right_SetSpeed(int32_t Speed);


#endif



