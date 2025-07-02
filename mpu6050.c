#include "MPU6050.h"

u8 mpu6050_write(u8 addr, u8 reg, u8 len, u8* buf)//����ֵ 0�����ɹ�  -1����ʧ��
{ 
		 unsigned char i;
     addr=addr<<1;                     //ע��dma���ַ���������һλ����Ҫ��λ
		 MPU6050_IIC_Start();              //��������
		 MPU6050_IIC_Send_Byte(addr);      //����������ַ           
		 MPU6050_IIC_Send_Byte(reg);       //���������ӵ�ַ

		 for(i=0;i<len;i++)            
			 MPU6050_IIC_Send_Byte(*buf++);  //��������
		 MPU6050_IIC_Stop();               //��������

		 return 0;
}

u8 mpu6050_read(u8 addr, u8 reg, u8 len, u8 *buf)//����ֵ 0�����ɹ�  -1����ʧ��
{
		 unsigned char i;
     addr=addr<<1;                     //ע��dma���ַ���������һλ����Ҫ��λ
		 MPU6050_IIC_Start();              //��������           
		 MPU6050_IIC_Send_Byte(addr);      //����������ַ            
		 MPU6050_IIC_Send_Byte(reg);       //���������ӵ�ַ

		 MPU6050_IIC_Start();              //������������
		 MPU6050_IIC_Send_Byte(addr+1);
		 for(i=0;i<len-1;i++)  
			 *buf++=MPU6050_IIC_Read_Byte(0);//��������
		 *buf=MPU6050_IIC_Read_Byte(1);
		 MPU6050_IIC_Stop();               //��������
	
		 return 0;
}

void mpu6050_write_reg(u8 reg, u8 dat)
{
   mpu6050_write(MPU_ADDR,reg,1,&dat);
}

u8   mpu6050_read_reg (u8 reg)
{
	 u8 dat;
   mpu6050_read(MPU_ADDR,reg,1,&dat);
	 return dat;
}

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	mpu6050_write_reg(GYRO_CONFIG,fsr<<3);//���������������̷�Χ  
	return 0;
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	mpu6050_write_reg(ACCEL_CONFIG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
	return 0;
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	mpu6050_write_reg(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
	return 0;
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	mpu6050_write_reg(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

void MPU6050_Init(void)
{ 
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	EXTI_InitTypeDef  EXIT_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//ʹ��AFIOʱ�� 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//��ʹ������IO PORTAʱ�� 
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//��ֹJTAG,�Ӷ�PA15��������ͨIOʹ��,����PA15��������ͨIO!!!	

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	 // �˿�����
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA


//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource15);  //����AFIO����ѡ����
//	
//	EXIT_InitStructure.EXTI_Line =EXTI_Line15 ;     		 //ѡ���ж���
//	EXIT_InitStructure.EXTI_LineCmd =ENABLE ;             	 //�Ƿ����ж�
//	EXIT_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;   	 //ѡ��ģʽ  �ж�/�¼�
//	EXIT_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  //ѡ�񴥷�ģʽEXTI_Trigger_Rising_Falling  EXTI_Trigger_Falling
//	
//	EXTI_Init(&EXIT_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; 
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //ѡ���ж�ʹ��/ʧ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     //��ռ���ȼ�����Ӧ���ȼ������ã��ڶ���ж���������
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStructure);
	
//	MPU_AD0_CTRL=0;			//����MPU6050��AD0��Ϊ�͵�ƽ,�ӻ���ַΪ:0X68
//	PAout(15);
//	PAin(15);
//	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
//	GPIO_SetBits(GPIOA, GPIO_Pin_15);


//	GPIO_InitTypeDef  GPIO_InitStructure;
//	EXTI_InitTypeDef  EXIT_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//��ʹ������IO PORTAʱ�� 


//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	 // �˿�����
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA


//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource3);  //����AFIO����ѡ����
//	
//	EXIT_InitStructure.EXTI_Line =EXTI_Line3 ;     		 //ѡ���ж���
//	EXIT_InitStructure.EXTI_LineCmd =ENABLE ;             	 //�Ƿ����ж�
//	EXIT_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;   	 //ѡ��ģʽ  �ж�/�¼�
//	EXIT_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  //ѡ�񴥷�ģʽEXTI_Trigger_Rising_Falling  EXTI_Trigger_Falling
//	
//	EXTI_Init(&EXIT_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; 
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //ѡ���ж�ʹ��/ʧ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     //��ռ���ȼ�����Ӧ���ȼ������ã��ڶ���ж���������
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStructure);
	
	
	u8 res; 
	MPU6050_IIC_IO_Init(); //��ʼ��IIC����
	mpu6050_write_reg(PWR_MGMT_1,0X80);	//��λMPU6050
    delay_ms(100);
	mpu6050_write_reg(PWR_MGMT_1,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(200);	//200					//���ò�����50Hz
	mpu6050_write_reg(MPU_INT_EN_REG,0X00);	//�ر������ж�
	mpu6050_write_reg(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	mpu6050_write_reg(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	mpu6050_write_reg(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=mpu6050_read_reg(MPU_DEVICE_ID_REG); 
	if(res==MPU_ADDR)//����ID��ȷ
	{
		mpu6050_write_reg(PWR_MGMT_1,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		mpu6050_write_reg(PWR_MGMT_2,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(100);	//100					//���ò�����Ϊ50Hz
 	}
}
 
//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	mpu6050_read(MPU_ADDR,TEMP_OUT_H,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=mpu6050_read(MPU_ADDR,GYRO_XOUT_H,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=mpu6050_read(MPU_ADDR,ACCEL_XOUT_H,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}
