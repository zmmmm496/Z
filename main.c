#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "TRACK.h"
#include "Cnotorl.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "buzzer.h"
#include "LED.h"

#define  ka1p	1.3
#define  ka1i   0.1
#define  ka1d 	5  


#define  ka2p	1.2
#define  ka2i    0
#define  ka2d 	4.2  // 基础速度70的pid

#define  kvp	0.1
#define  kvd	0.1


uint8_t TRACK1;
uint8_t TRACK2;
uint8_t TRACK3;
uint8_t TRACK4;
uint8_t TRACK5;
uint8_t TRACK6;
uint8_t TRACK7;
uint8_t TRACK8;

	
int left,right;
int statement=-1;
int pid_out;
int find_err;
void scan(void);
int pid_angle1(int target,int yaw);
int pid_angle2(int target,int yaw);

float xj_error,xj_pidout,xj_set;
float Pitch,Roll,Yaw;								//俯仰角默认跟中值一样，翻滚角，偏航角
int16_t ax,ay,az,gx,gy,gz;							//加速度，陀螺仪角速度


 int left,right  ,aleft,aright;
float Yaw0_360;
int ack=0;
uint8_t all_state=0;
int step=1;	
int count=0;

int main(void)
{	
  
	LED_Init();
	buzzer_Init();
	Motor_Init();


	Key_Init();
	Track_Init();
	OLED_Init();
	MPU6050_Init();
	MPU6050_DMP_Init();	
	
	while (1)
	{	

		buzzer_off();
		LED1_ON ();
	while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
	ack=1;
	Delay_ms (10)	;	
	OLED_ShowSignedNum(1,1,Yaw,5);
	OLED_ShowHexNum(2,1,all_state,4);
	OLED_ShowSignedNum(3,1,step,3);
	OLED_ShowBinNum(4,1,Get_Infrared_State(),8);		
	switch (Key_GetNum ())
	{
		case 1:
		if 			((all_state & 0x000f)==0x0000)	all_state=0x0001;//第一问
		else if		((all_state & 0x000f)==0x0001)	all_state=0x0002;//第二问
		else if		((all_state & 0x000f)==0x0002)	all_state=0x0003;//第三问
		else if		((all_state & 0x000f)==0x0003)	all_state=0x0004;//第四问
		else if		((all_state & 0x000f)==0x0004)	all_state=0x0005;//第五问
		else if		((all_state & 0x000f)==0x0005)	all_state=0x0006;//第六问		
		break;                    
	
		case 2:	
		all_state  = all_state  | 0x0010  ;//两个十六进制按位或运算，是把16进制的一位换成二进制的四位
		break;
	}

	delay_ms (10);
	
	
	switch(all_state )
	{
		
//====================第一问状态=================================================
//===============================================================================		
		case 17:
				delay_ms (300);
			while (Get_Infrared_State()==0)
			{	
				
				while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
				ack=1;
				aleft=30-pid_angle2(0,Yaw);
				aright=30+pid_angle2(0,Yaw);	//第一次直线
				Limit(&aleft,&aright);
				Set_Speed(aleft,aright);	
				OLED_ShowSignedNum(1,1,Yaw,5);
			}				
			Set_Speed(0,0);
			LED1_Turn();
			buzzer_turn();
			Delay_ms (100);
			buzzer_off();
			LED1_OFF ();
			
			
		break;
//===============================================================================			
//===============================================================================



			
//==============第二问状态=======================================================
//===============================================================================
		case 18:
				buzzer_off();
				LED1_ON ();
				delay_ms (100);
		while (1)
		{			
			switch (step )
			{
				delay_ms (500);
				case 1:
				{
					while (Get_Infrared_State()==0)//
					{	
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
						
							aleft=25-pid_angle2(0,Yaw);
							aright=25+pid_angle2(0,Yaw);	//第一次斜线
							Limit(&aleft,&aright);
							Set_Speed(aleft,aright);	
							OLED_ShowSignedNum(1,1,Yaw,5);
					}
					if  (Get_Infrared_State()!=0)  Set_Speed(15,15),delay_ms(10), step=2;				
			
				
				}break;
			
				case 2:
				{   
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	

						
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第一次循迹
							Final_Speed(pid_out,25); 
					}		
					while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
					
					if (Get_Infrared_State()==0 )
					{
					LED1_Turn();
					buzzer_turn();		
						
					Set_Speed(0,0);
					delay_ms(300);					
						
					while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
					ack=1;

					step=3;
					
					
					}
					else step=2;
				
				}break;			
				
				case 3:{  

//   

					while (Get_Infrared_State()==0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
//					
						if(Yaw<0)  Yaw0_360 =Yaw +360;
						else       Yaw0_360=Yaw;	
							aleft=25-pid_angle2(178,Yaw0_360);  
							aright=25+pid_angle2(178,Yaw0_360);	//第二次斜线
							Limit(&aleft,&aright);
							Set_Speed(aleft,aright);
							OLED_ShowSignedNum(1,1,Yaw,5);
						
					}
					
					if (Get_Infrared_State()!=0   )  Set_Speed(15,15),delay_ms(10), step=4  ;
					else  step=3;
					}break;	
							
				
				
				case 4:{   
					
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第二次循迹
							Final_Speed(pid_out,22); 
					}
					if (Get_Infrared_State()==0)     step=5;// Set_Speed(18,18),delay_ms(4),
					else  step=4;
				}break;				
								
				
				case 5:{   
					
					LED1_Turn();
					buzzer_turn();					
					
					while(1)
					{
						OLED_ShowSignedNum(1,1,Yaw,5);
						Set_Speed(0,0);
	

						step=999;
			
					}
				}break;	
			}				
			}		
			
		break;
//===============================================================================
//===============================================================================		
	

			
//===================第三问======================================================
//===============================================================================
		case 19:
				delay_ms (10);
		while (1)
		{			
			switch (step )
			{
				case 1:
				{
					delay_ms (100);
					while (Get_Infrared_State()==0)//
					{	
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
						
							aleft=25-pid_angle2(-38,Yaw);
							aright=25+pid_angle2(-38,Yaw);	//第一次斜线
							Limit(&aleft,&aright);
							Set_Speed(aleft,aright);	
							OLED_ShowSignedNum(1,1,Yaw,5);
					}
					if  (Get_Infrared_State()!=0)  Set_Speed(15,15),delay_ms(10), step=2;//				Set_Speed(0,0),delay_ms(10),
			
				
				}break;
			
				case 2:
				{   
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;							
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第一次循迹
							Final_Speed(pid_out,20); 
					}	
					LED1_Turn();
					buzzer_turn();					
					if (Get_Infrared_State()==0 )  Set_Speed(0,0),Delay_ms(300), step=3;//Set_Speed(30,30),Delay_ms(15),
					else step=2;
				
				}break;			
				
				case 3:{  
	
					while (Get_Infrared_State()==0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
							if(Yaw<0)  Yaw0_360 =Yaw +360;
							else       Yaw0_360=Yaw;
				
							
							aleft=25-pid_angle2(217,Yaw0_360);  
							aright=25+pid_angle2(217,Yaw0_360);	//第二次斜线
							Limit(&aleft,&aright);
							Set_Speed(aleft,aright);
							OLED_ShowSignedNum(1,1,Yaw,5);
						
					}
					
					if (Get_Infrared_State()!=0   )  Set_Speed(0,0),delay_ms(10), step=4  ;//Set_Speed(0,0),delay_ms(10),
					else  step=3;
					}break;	
							
				
				
				case 4:{   
					
					LED1_Turn();
					buzzer_turn();
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第二次循迹
							Final_Speed(pid_out,22); 
					}
					if (Get_Infrared_State()==0) Set_Speed(15,15),delay_ms(4),   step=5;// Set_Speed(18,18),delay_ms(4),
					else  step=4;
				}break;				
								
				
				case 5:{   
					
					LED1_Turn();
					buzzer_turn();					
					
					while(1)
					{
						OLED_ShowSignedNum(1,1,Yaw,5);
						Set_Speed(0,0);
	

						step=999;
			
					}
				}break;	
			}				
			}	

//======================================================================================================
//======================================================================================================
						
			
			
			
			
			
			
//========================第四问=========================================================================
//=======================================================================================================

		case 20:
				delay_ms (200);
	  while(1)
	  {
			switch (step )
			{
				case 1:
				{
					
					delay_ms (300);
					while (Get_Infrared_State()==0)//
					{	
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
						
							aleft=20-pid_angle2(-38,Yaw);
							aright=20+pid_angle2(-38,Yaw);	//第一次斜线
							Limit(&aleft,&aright);
							Set_Speed(aleft,aright);	
							OLED_ShowSignedNum(1,1,Yaw,5);
					}
					if  (Get_Infrared_State()!=0) Set_Speed(0,0),delay_ms(10), step=2;				
			
				
				}break;
			
				case 2:
				{  
					
					LED1_Turn();
					buzzer_turn();				
					
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;							
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第一次循迹
							Final_Speed(pid_out,19); 
					}
					
					LED1_Turn();
					buzzer_turn();
					
					if (Get_Infrared_State()==0) Set_Speed(13,13),Delay_ms(4), step=3;
					else step=2;
				
				}break;			
				
				case 3:{   
					delay_ms (300);
					while (Get_Infrared_State()==0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;
						
							if(Yaw<0)  Yaw0_360 =Yaw +360;
							else       Yaw0_360=Yaw;
				
							
							aleft=20-pid_angle2(214,Yaw0_360);  
							aright=20+pid_angle2(214,Yaw0_360);	//第二次斜线
							Limit(&aleft,&aright);
							Set_Speed(aleft,aright);
							OLED_ShowSignedNum(1,1,Yaw,5);
						
					}
					
					if (Get_Infrared_State()!=0   )Set_Speed(0,0),delay_ms(10), step=4  ;
					else  step=3;
					}break;	
							
				
				
				case 4:{   

					LED1_Turn();
					buzzer_turn();					
					
					while (Get_Infrared_State()!=0)
					{
						while(MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw)!=0){ack=999;};
							ack=1;	
							OLED_ShowSignedNum(1,1,Yaw,5);
							find_err=Track_err();		
							pid_out=PID_out(find_err ,0); //第二次循迹
							Final_Speed(pid_out,20); 
					}
					
					LED1_Turn();
					buzzer_turn();
					
					
				if(Get_Infrared_State()==0 && count==3)  step=5;
				else	if (Get_Infrared_State()==0)  count+=1,  Set_Speed(0,0),delay_ms(10),  step=1;// Set_Speed(70,70),delay_ms(20), 
				}break;				
								
				
				case 5:{   
					while(1)
					{
						OLED_ShowSignedNum(1,1,Yaw,5);
						Set_Speed(0,0);
						step=999;
			
					}
				}break;	
			}				
			}
		}
	}
}
	

int pid_angle1(int target,int yaw)
{
	int err_last,err_intrgre,pid_a_out,err;
	err=target-yaw;
	err_intrgre+=err;
	if(err_intrgre>=15) err_intrgre=15;  
	if(err_intrgre<=-15) err_intrgre=-15;
	pid_a_out =ka1p*err+ka1i*err_intrgre+ka1d*err_last;
	err_last=err;
	return pid_a_out;
}


int pid_angle2(int target,int yaw)
{
	int err_last,err_intrgre,pid_a_out,err;
	err=target-yaw;
	err_intrgre+=err;
	if(err_intrgre>=15) err_intrgre=15;  
	if(err_intrgre<=-15) err_intrgre=-15;
	pid_a_out =ka2p*err+ka2i*err_intrgre+ka2d*err_last;
	err_last=err;
	return pid_a_out;
}





