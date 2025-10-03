#include "Rudder_control.h"
#include  "uart.h"
#include  "math.h"
uint8_t test;
// iic
void Rudder_Init(uint16_t hz)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOD_CLK_ENABLE();
	IIC_Init();//IIC��ʼ��
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);//OEʹ��
	GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	Rudder_WriteOneByte(0x0,0x11);					//��������ģʽ  ----Ϊ������Ƶ��
	Rudder_WriteOneByte(PRE_SCALE,(uint8_t)get_freq(hz));
	Rudder_WriteOneByte(0x0,0x1);					//ȡ������ģʽ  ----���õ�ַΪ0x70;
}
//��ָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void Rudder_WriteOneByte(uint8_t WriteAddr,uint8_t DataToWrite)
{
	IIC_Start();  	
	IIC_Send_Byte(Rudder<<1);    //����������ַ0x70,д���� 	 
	IIC_Wait_Ack();
  IIC_Send_Byte(WriteAddr);   //����Ŀ���ַ
	IIC_Wait_Ack();	
	IIC_Send_Byte(DataToWrite);//�����ֽ�
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//����һ��ֹͣ���� 
	delay_ms(10);	 
}
//ָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
uint8_t Rudder_ReadOneByte(uint8_t ReadAddr)
{				  
	uint8_t temp=0;		  	    																 
  IIC_Start();  
	IIC_Send_Byte(Rudder<<1);   //����������ַ0X70,д���� 	   
	IIC_Wait_Ack();
  IIC_Send_Byte(ReadAddr);   //����Ŀ���ַ
	IIC_Wait_Ack();	
	IIC_Start();  	 	   
	IIC_Send_Byte((Rudder<<1)+1);           //�������ģʽ			   
	IIC_Wait_Ack();	 
  temp=IIC_Read_Byte(0);		   
  IIC_Stop();					//����һ��ֹͣ����	    
	return temp;
}
/*****************************************************************************
��������  Rudder_control()
�������ܣ��������
�βΣ� aim---Ŀ��Ƕ�    ID�����id
��ע��  
*******************************************************************************/
void Rudder_control(uint16_t aim,uint8_t id){
		Rudder_WriteOneByte((uint8_t)(LED0_ON_L+(4*id)),0x0);
		Rudder_WriteOneByte((uint8_t)(LED0_ON_H+(4*id)),0x0);
		
		Rudder_WriteOneByte((uint8_t)(LED0_OFF_L+(4*id)),(uint8_t)(aim));
		Rudder_WriteOneByte((uint8_t)(LED0_OFF_H+(4*id)),(uint8_t)(aim>>8));
		test = Rudder_ReadOneByte((uint8_t)(LED0_OFF_L+(4*id)));
}

uint8_t get_freq(uint16_t hz){
	uint8_t prescale;
	double prescaleval;
	hz*= 0.92; 
	prescaleval = 25000000;
	prescaleval /= 4096;
	prescaleval /= hz;
	prescaleval -= 1;
	prescale =floor(prescaleval + 0.5f);
	return prescale;
}
