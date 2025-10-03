#include "scaner.h"
#include "math.h"
#include "turn.h"
#include "pid.h"
#include "speed_ctrl.h"
#include "motor.h"
#include "uart.h"
#include "motor_task.h"

#define LINE_SPEED_MAX 2000
#define Speed_Compensate   5
#define BLACK 0								//ѭ����
#define WHLITE 1							//ѭ����

const float line_weight[8] = {3.1,1.8,0.9,0.4,-0.4,-0.9,-1.8,-3.1};
//����

struct Scaner_Set scaner_set = {0,0};
uint8_t L_R_open = 0;  //���ݿ�������ѭ��   1�� 2�� 
SCANER Scaner;

#define Line_color WHLITE


//ѭ������
void Go_Line(float speed){

	float Fspeed;				//����PID�����Ľ��
					
	line_pid_obj.measure = Scaner.error;  							//��ǰѭ�������ڵ�λ�ã�������-7��0��0��7
	line_pid_obj.target = scaner_set.CatchsensorNum;		//Ŀ��
	
	Fspeed = positional_PID(&line_pid_obj, &line_pid_param); //����λ��PID����
	
	if (Fspeed>=LINE_SPEED_MAX) 
		Fspeed = LINE_SPEED_MAX;
	else if (Fspeed<=-LINE_SPEED_MAX) 
		Fspeed = -LINE_SPEED_MAX;
	Fspeed *= fabsf(speed)/50;
	motor_all.Lspeed = speed-Fspeed;
	motor_all.Rspeed = speed+Fspeed;
//	printf("%f    %f    %f   %f  %d\r\n",Scaner.error,motor_all.Lspeed,motor_all.Rspeed,Fspeed,Scaner.lineNum);
}


//ѭ��ɨ��
//ѭ���� 1234578 87654321
uint8_t Line_Scan(SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore)
{
	float error = 0;
	uint16_t data = 0;
	u8 linenum=0;
	u8 lednum=0;
	int8_t lednum_tmp = 0;
	data = 0X0;
	
	data|= ((uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9));
	data|= ((uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)<<1);
	data|= ((uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)<<2);
	data|= ((uint8_t)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)<<3);
	data|= ((uint8_t)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)<<4);
	data|= ((uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)<<5);
	data|= ((uint8_t)HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)<<6);
	data|= ((uint8_t)HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)<<7);
//	printf("%d %d %d %d %d %d %d %d\r\n",(int)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9),
//		(int)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8),
//			(int)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5),
//				(int)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15),
//					(int)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14),
//						(int)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4),
//							(int)HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4),
//								(int)HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5));
	Scaner.detail = data;							//��ö�����Ѳ��ֵ
	for(uint8_t i=0;i<sensorNum;i++) 		//��С�������������������������������
	{								//linenum������¼�ж������ߣ�line������¼�ڼ����ߡ�
		
		if((scaner->detail&(0x1<<i))) 
		{
			lednum++;
			if(!(scaner->detail&(1<<(i+1)))) 
				++linenum;			//�ȶ�ȡ��������������������⵽��1��Ϊ0��Ϊһ����
		}
	}
	
	scaner->lineNum = linenum;			
	scaner->ledNum=lednum;
	
	for(uint8_t i= edge_ignore; i<sensorNum - edge_ignore; i++) {
						lednum_tmp += (scaner->detail>>(sensorNum-1-i))&0X01;
						error += ((scaner->detail>>(sensorNum-1-i))&0X01) * line_weight[i];
	}	
	if(lednum_tmp==0){  //�޵�
		scaner->detail=0;		//0
		Scaner.error = 0;
		Scaner.lineNum = 0;
		Scaner.ledNum = 0;
		return 1;
	}	
	if(find_line.sleep_dargon!=1){
	error/=(float)lednum_tmp;		//ȡƽ��
	}
	Scaner.error = error;
	return 0;
}
void scaner_init(void){
	   GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOB_CLK_ENABLE();   //ʹ��GPIOCʱ��
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
	
    //PC11,12��ʼ������
    GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_8|GPIO_PIN_5|GPIO_PIN_4;
    GPIO_Initure.Mode=GPIO_MODE_INPUT ;  
    GPIO_Initure.Pull=GPIO_PULLDOWN;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
		GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15;
    GPIO_Initure.Mode=GPIO_MODE_INPUT ;  
    GPIO_Initure.Pull=GPIO_PULLDOWN;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
	
		GPIO_Initure.Pin=GPIO_PIN_4|GPIO_PIN_5;
    GPIO_Initure.Mode=GPIO_MODE_INPUT ;  
    GPIO_Initure.Pull=GPIO_PULLDOWN;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}



