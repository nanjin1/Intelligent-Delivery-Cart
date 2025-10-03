#include "speed_ctrl.h"
#include "motor_task.h"
#include "pid.h"
#define need_gradual_cal 0
volatile struct Motors motor_all = {
	.Lspeed = 0,
	.Rspeed = 0,
	.GyroG_speedMax = 800,
	.GyroT_speedMax = 800,
	.SPEED_MAX = 300,
	.Diretion = 0,
	
	.Cincrement = 150,	//ѭ�����ٶ�
	.Gincrement = 150,		//��ѭ�����ٶ�
	.F_encoder_avg = 0,
	.R_encoder_avg = 0,
	.F_Distance = 0,
	.R_Distance = 0,
	
	.is_UP = false,
	.is_DOWM = false,
};
//TC_speed:Ѳ���ٶ�       TG_speed : ��ƽ���ٶ�
struct Gradual TC_speed = {0,0,0},TG_speed = {0,0,0},CG_speed = {0,0,0};


void CarBrake(void)
{
	pid_mode_switch(is_No);
	motor_L0.target = 0;
	motor_R0.target = 0;
	motor_L1.target = 0;
	motor_R1.target = 0;
	motor_pid_clear();
}

//��һ�κ����������ٻ��߻���ֹͣ
//�β� ���ٶȾ��  Ŀ���ٶ�  ���ٶ�
void gradual_cal(struct Gradual *gradual, float target, float increment)  
{
	#if need_gradual_cal
	uint8_t direction = 0;
	
	if(target - gradual->Now < 0) 
		direction = 0;			//������� 
	else
		direction = 1;			//�������
		
	if(gradual->Now != target)	
	{
		if (direction)     
			gradual->Now += increment;
		else							
			gradual->Now -= increment;
	}
	else 
	{	
		return;				//�ﵽĿ���ٶ�
	}
	
	//���ٴ���
	if(direction == 1)
	{
		if(gradual->Now > target) 		
		{
			gradual->Now = target;
		}
	}
	else if(direction == 0)			
	{
		if(gradual->Now < target)
		{
			gradual->Now = target;
		}
	}
	#else
		gradual->Now = target;
	#endif
}

