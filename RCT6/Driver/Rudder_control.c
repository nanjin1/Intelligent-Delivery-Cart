#include "Rudder_control.h"
#include  "uart.h"
#include  "math.h"
uint8_t test;
// iic
void Rudder_Init(uint16_t hz)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOD_CLK_ENABLE();
	IIC_Init();//IIC初始化
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);//OE使能
	GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	Rudder_WriteOneByte(0x0,0x11);					//进入休眠模式  ----为了设置频率
	Rudder_WriteOneByte(PRE_SCALE,(uint8_t)get_freq(hz));
	Rudder_WriteOneByte(0x0,0x1);					//取消休眠模式  ----设置地址为0x70;
}
//在指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void Rudder_WriteOneByte(uint8_t WriteAddr,uint8_t DataToWrite)
{
	IIC_Start();  	
	IIC_Send_Byte(Rudder<<1);    //发送器件地址0x70,写数据 	 
	IIC_Wait_Ack();
  IIC_Send_Byte(WriteAddr);   //发送目标地址
	IIC_Wait_Ack();	
	IIC_Send_Byte(DataToWrite);//发送字节
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//产生一个停止条件 
	delay_ms(10);	 
}
//指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
uint8_t Rudder_ReadOneByte(uint8_t ReadAddr)
{				  
	uint8_t temp=0;		  	    																 
  IIC_Start();  
	IIC_Send_Byte(Rudder<<1);   //发送器件地址0X70,写数据 	   
	IIC_Wait_Ack();
  IIC_Send_Byte(ReadAddr);   //发送目标地址
	IIC_Wait_Ack();	
	IIC_Start();  	 	   
	IIC_Send_Byte((Rudder<<1)+1);           //进入接收模式			   
	IIC_Wait_Ack();	 
  temp=IIC_Read_Byte(0);		   
  IIC_Stop();					//产生一个停止条件	    
	return temp;
}
/*****************************************************************************
函数名：  Rudder_control()
函数功能：舵机控制
形参： aim---目标角度    ID：舵机id
备注：  
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
