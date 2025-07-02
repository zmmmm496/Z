#include "stm32f10x.h"                  // Device header
#include "TRACK.h" 
#include "Motor.h"
#include "stdio.h" 
#include "PWM.h"

extern uint8_t TRACK1;
extern uint8_t TRACK2;
extern uint8_t TRACK3;
extern uint8_t TRACK4;
extern uint8_t TRACK5;
extern uint8_t TRACK6;
extern uint8_t TRACK7;
extern uint8_t TRACK8;

void Track_Init(void)
{
	//A�ڵ���������
	GPIO_InitTypeDef GPIO_InitStructure;//GPIO�ṹ�嶨��
	RCC_APB2PeriphClockCmd(TRACK_PORT_CLKA ,ENABLE);//�򿪶˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = TRACK_INFRARED_PIN_1 | TRACK_INFRARED_PIN_2  |TRACK_INFRARED_PIN_3 | TRACK_INFRARED_PIN_5  |  TRACK_INFRARED_PIN_7;//���ô�������ȡ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//����Ϊ��������ģʽ
	GPIO_Init(TRACK_PORTA,&GPIO_InitStructure);//��ʼ���˿�
	
	//B�ڵ���������
	RCC_APB2PeriphClockCmd(TRACK_PORT_CLKB ,ENABLE);      //�򿪶˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = TRACK_INFRARED_PIN_4 | TRACK_INFRARED_PIN_6  |  TRACK_INFRARED_PIN_8 ;  //���ô�������ȡ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//����Ϊ��������ģʽ
	GPIO_Init(TRACK_PORTB,&GPIO_InitStructure);//��ʼ���˿�
}


 u8 Get_Infrared_State(void)
{
	TRACK1= !GPIO_ReadInputDataBit(TRACK_PORTA, TRACK_INFRARED_PIN_1 )<<7 ;
	TRACK2= !GPIO_ReadInputDataBit(TRACK_PORTA, TRACK_INFRARED_PIN_2 )<<6 ;
	TRACK3= !GPIO_ReadInputDataBit(TRACK_PORTA, TRACK_INFRARED_PIN_3 )<<5 ;
	TRACK4= !GPIO_ReadInputDataBit(TRACK_PORTB, TRACK_INFRARED_PIN_4 )<<4 ;
	TRACK5= !GPIO_ReadInputDataBit(TRACK_PORTA, TRACK_INFRARED_PIN_5 )<<3 ;
	TRACK6= !GPIO_ReadInputDataBit(TRACK_PORTB, TRACK_INFRARED_PIN_6 )<<2 ;
	TRACK7= !GPIO_ReadInputDataBit(TRACK_PORTA, TRACK_INFRARED_PIN_7 )<<1 ;
	TRACK8= !GPIO_ReadInputDataBit(TRACK_PORTB, TRACK_INFRARED_PIN_8 )<<0 ;	

	//0ɨ�����ߣ�1ûɨ��
	u8 state = 0,state1 = 0;
	
	state=(u8)(TRACK1|TRACK2|TRACK3|TRACK4|TRACK5|TRACK6|TRACK7|TRACK8);//ƴ�ӳɰ�λ���ݣ����λΪΪ�������ĵ���1�����ΪΪ����������1
	return state;
	if(TRACK1==0||TRACK2==0||TRACK3==0||TRACK4==0||TRACK5==0||TRACK6==0||TRACK7==0||TRACK8==0)//4 5ɨ��
	return state1;
	
}


//ʶ�𵽺���Ϊ1������Ϊ0

void Track_Adjust(void)//�����ٶȵ�ѭ������pid������ʹ��
{
	u8 state =  Get_Infrared_State();
	
	switch(state)
	{  

		case 0:   //0000    
		Set_Speed(20 ,20);	  break;
		case 6:   //0110---�м�Ѱ����
		Set_Speed(15 ,15);    break;
		case 4:   //0100---��ƫһ��
	    Set_Speed(15 ,25);    break;
		case 12:  //1100---��ƫ����
	    Set_Speed(10 ,25);   break;
		case 8:   //1000---��ƫ����
        Set_Speed(5 ,25);    break;
		case 2:   //0010---��ƫһ��
        Set_Speed(20 ,15);   break;
		case 3:   //0011---��ƫ����
        Set_Speed(25 ,15);   break;
		case 1:   //0001---��ƫ����
        Set_Speed(25,5);     break;	
		default: 
		Set_Speed(15 ,15);   break;
	}
}


