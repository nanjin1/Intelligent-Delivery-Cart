#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include "sys.h"
#include "stdbool.h"

struct Gradual
{
	float Last;			//�ı�ʱ����һ�ε�ֵ
	float Now;			//��ǰֵ
	float D_value;		//�ı�ʱ�̵�Ŀ��ֵ�뵱ǰֵ�Ĳ�
};

struct Motors
{
	float Lspeed,Rspeed;	//�ٶ�
	float Cspeed;					//Ѱ���ٶ�
	float Gspeed;					//��ƽ���ٶ�
	float CG_speed;				//������ѭ�������ٶ�
	
	float GyroT_speedMax;	//ת������ٶ�
	float GyroG_speedMax;	//��ƽ������ٶ�
	float SPEED_MAX;
	float Diretion;
	
	float F_encoder_avg;	//���������� 
	float R_encoder_avg;	//���������� 
	
	float F_Distance;		//ǰ��·��
	float R_Distance;		//����·�� 
	
	float Cincrement;	//ѭ�����ٶ�
	float Gincrement;	//��ѭ�����ٶ�  
	
	bool is_UP;
	bool is_DOWM;
};

extern volatile struct Motors motor_all;

extern struct Gradual TC_speed, TG_speed,CG_speed;

void gradual_cal(struct Gradual *gradual, float target, float increment);
void CarBrake(void);

#endif
