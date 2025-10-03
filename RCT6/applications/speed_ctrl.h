#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include "sys.h"
#include "stdbool.h"

struct Gradual
{
	float Last;			//改变时的上一次的值
	float Now;			//当前值
	float D_value;		//改变时刻的目标值与当前值的差
};

struct Motors
{
	float Lspeed,Rspeed;	//速度
	float Cspeed;					//寻迹速度
	float Gspeed;					//自平衡速度
	float CG_speed;				//陀螺仪循迹矫正速度
	
	float GyroT_speedMax;	//转弯最大速度
	float GyroG_speedMax;	//自平衡最大速度
	float SPEED_MAX;
	float Diretion;
	
	float F_encoder_avg;	//编码器读数 
	float R_encoder_avg;	//编码器读数 
	
	float F_Distance;		//前进路程
	float R_Distance;		//右移路程 
	
	float Cincrement;	//循迹加速度
	float Gincrement;	//非循迹加速度  
	
	bool is_UP;
	bool is_DOWM;
};

extern volatile struct Motors motor_all;

extern struct Gradual TC_speed, TG_speed,CG_speed;

void gradual_cal(struct Gradual *gradual, float target, float increment);
void CarBrake(void);

#endif
