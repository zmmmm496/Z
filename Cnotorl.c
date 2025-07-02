#include "stm32f10x.h"              
#include "Motor.h"
#include "Timer.h"
#include "TRACK.h"

float error;


#define Kp    4
#define Kd    3

float Track_err(void)
{
	u8 state =  Get_Infrared_State();
	//�����ߵ��ұ�errΪ��ֵ�����Ϊ����
	switch(state)
	{  

		case 0:   //0000 0000   
		error= 0 ;	  break;
		case 16:   //0001 0000   
		error= 0 ;	  break;
		case 8:   //0000 1000   
		error= 0 ;	  break;	
		case 24:   //0001 1000   
		error= 0 ;	  break;		
		case 60:   //0011 1100   
		error= 0 ;	  break;		
		case 126:   //0111 1110   
		error= 0 ;	  break;
		
		case 48:   //0011 0000   //С����ƫ��errΪ��
		error= 2.5  ;	  break;
		case 32:   //0010 0000   
		error= 3 ;	  break;
		case 64:   //0100 0000   
		error= 5 ;	  break;
		case 96:   //0110 0000   
		error= 4.5 ;	  break;
		case 128:   //1000 0000   
		error= 7 ;	  break;	
		case 192:   //1100 0000   
		error= 6.5 ;	  break;


		
		case 12:   //0000 1100   //С����ƫ��errΪ��
		error= -2.5 ;	  break;		
		case 4:   //0000 0100   
		error= -3 ;	  break;		
		case 2:   //0000 0010   
		error= -5 ;	  break;
		case 6:   //0000 0110   
		error= -4.5 ;	  break;
		case 1:   //0000 0001   
		error= -7 ;	  break;
		case 3:   //0000 0011   
		error= -6.5 ;	  break;
		
		
		default: 
		error=0;   break;
	}
	return error;
}



int PID_out(float error,int Target)//poistion_pid:��ڲ�����1.ѭ��ģ�鷵�ص����;2.Ŀ����Ҳ����0����С�����ߵ��м�+Ki*add_err
{
    int err=error;
    int last_err;
	int out;
	int add_err;
	add_err+=err;
	out=Kp*err+Kd*(err-last_err);
	last_err=err;
	return out;

}




void Final_Speed(int pid_out ,int base_speed)
{

  Set_Speed(base_speed-pid_out,base_speed+pid_out);

}

