#include "turn.h"
#include "motor_task.h"
#include "math.h"
#include "pid.h"
#include "uart.h"
#include "speed_ctrl.h"
#include "light.h"

float x, y, w,chassis_w = 0;
volatile struct Angle angle = {0, 0,0};
float Radius_[5] = {0,
                    6.9,
                    6.9,
                    6.9,
                    6.9};
uint8_t distance_model =0;
float F_SPEED,R_SPEED;
uint8_t x_dir=0;
uint8_t y_dir=1;		
uint8_t K=0;										
//输出当前角度与目标角度的最小夹角 
float need2turn(float nowangle,float targetangle)
{			
	float need2Turn;		

	need2Turn=targetangle-nowangle;		//实际所需转的角度
	if(need2Turn>180)	need2Turn -= 360;
  else if(need2Turn<-180)	need2Turn += 360;
	
  return need2Turn;		
}



//陀螺仪软校准   //转弯角     //传感器角度     //参考角度	 得到当前角度和参考角度的差距
void mpuZreset(float sensorangle ,float referangle)
{	
	imu.compensateZ=need2turn(sensorangle,referangle);
}

//获取Z角度					
float getAngleZ(void) 
{
	float targetangle;
	targetangle = imu.yaw + imu.compensateZ;	

	if(targetangle>180)	targetangle -= 360;
	else if(targetangle<-180)	targetangle += 360;
	
	return targetangle;
}

/*****************************************************************************
函数编写人及更新日期：陈梓华，7-19
函数功能：陀螺仪Z轴转角度，转相对角度
输入：要转的角度 ，速度
输出：无
*****************************************************************************/
void Turn_Angle_Relative(float Angle1)//左180到右-180,速度必须是正的，
{
	float Turn_Angle_Before = 0, Turn_Angle_Targe = 0;
	
	Turn_Angle_Before = getAngleZ();//读取当前的角度//@@@@@
	Turn_Angle_Targe = Turn_Angle_Before+Angle1;//目标角度设为绝对坐标
	/*******************如果存在临界状态，把目标角度转化为绝对坐标******180 0 -180*************/
    if(Turn_Angle_Targe>180){				
    	Turn_Angle_Targe=Turn_Angle_Targe-360;
    }
    else if(Turn_Angle_Targe<-180){
    	Turn_Angle_Targe=Turn_Angle_Targe+360;
    }
	
	angle.AngleT = Turn_Angle_Targe;
	pid_mode_switch(is_Turn);  //进入转弯
}

/*****************************************************************************
函+数功能：陀螺仪Z轴结合PID原地转角度
					填要转到的角度(绝对角度)
					用法：Target = ?,PIDMode = is_Turn,while(fabs(getAngleZ()-Target)<1);
*****************************************************************************/
uint8_t Turn_Angle(float Angle)	
{
	float GTspeed, now_angle;
	
	//临界处理
	if(Angle>180)	Angle -= 360;
	else if(Angle<-180)	Angle += 360;
	
	now_angle = getAngleZ();
	if (fabsf(Angle-now_angle) < 2)
	{
			motor_L0.target = 0;
			motor_L1.target = 0;
			motor_R0.target = 0;
			motor_R1.target = 0;
		gyroT_pid.integral = 0;
		gyroT_pid.output = 0;
		return 1;
	}
	
	gyroT_pid.measure = need2turn(now_angle, Angle);
	gyroT_pid.target = 0;

	GTspeed = positional_PID(&gyroT_pid, &gyroT_pid_param);
	
	if(GTspeed >= motor_all.GyroT_speedMax) 
		GTspeed = motor_all.GyroT_speedMax;
	else if(GTspeed <= -motor_all.GyroT_speedMax) 
		GTspeed = -motor_all.GyroT_speedMax;

	motor_L0.target = GTspeed;
	motor_L1.target = -GTspeed;
	motor_R0.target = GTspeed;
	motor_R1.target = -GTspeed;
	
	return 0;
}
//平台转向
uint8_t Stage_turn_Angle(float Angle)	
{
	float GTspeed, now_angle;
	
	//临界处理
	if(Angle>180)	Angle -= 360;
	else if(Angle<-180)	Angle += 360;
	
	now_angle = getAngleZ();
	if (fabsf(Angle-now_angle) < 1)
	{
		motor_all.Lspeed = motor_all.Rspeed = 0;
		gyroT_pid.integral = 0;
		gyroT_pid.output = 0;
		return 1;
	}
	
	gyroT_pid.measure = need2turn(now_angle, Angle);
	gyroT_pid.target = 0;

	GTspeed = positional_PID(&gyroT_pid, &gyroT_pid_param);
	
	if(GTspeed >= motor_all.GyroT_speedMax) 
		GTspeed = motor_all.GyroT_speedMax;
	else if(GTspeed <= -motor_all.GyroT_speedMax) 
		GTspeed = -motor_all.GyroT_speedMax;
	
		motor_all.Lspeed = 1.35f*GTspeed; 
		motor_all.Rspeed = -GTspeed;
	return 0;
}
/*****************************************************************************
函数功能：陀螺仪Z轴自平衡行走 diretion=0 前进  diretion=1  左移
*****************************************************************************/
uint8_t runWithAngle(float angle_want,float speed)
{
	float GGspeed, now_angle;
	
	now_angle = getAngleZ();
	
	gyroG_pid.measure = need2turn(now_angle, angle_want);  
	gyroG_pid.target = 0;
	
	GGspeed = positional_PID(&gyroG_pid, &gyroG_pid_param);
	
	if(GGspeed >= motor_all.GyroG_speedMax) 
		GGspeed = motor_all.GyroG_speedMax;
	else if(GGspeed <= -motor_all.GyroG_speedMax) 
		GGspeed = -motor_all.GyroG_speedMax;					//限制最大值

	if(K==1)
	{
		w=0;
	}
	else
	{
    w = chassis_w + GGspeed;
	}
	motor_L0.target =  (y +  x + Radius_[1] * w);
	motor_L1.target = -(y -  x + Radius_[1] * w);		
	motor_R0.target = -(y -  x - Radius_[1] * w);	
	motor_R1.target =  (y +  x - Radius_[2] * w);
	
	return 1;
}

/*****************************************************************************
函数功能：陀螺仪位置式自平衡行走
*****************************************************************************/
uint8_t Distance_run(float angle_want)
{
	float GGspeed, now_angle;
	
	now_angle = getAngleZ();
	
	gyroG_pid.measure = need2turn(now_angle, angle_want);  
	gyroG_pid.target = 0;
	
	GGspeed = positional_PID(&gyroG_pid, &gyroG_pid_param);
	
	F_Distance_pid_obj.measure = distances.b_distance;//超声波
	R_Distance_pid_obj.measure = distances.l_distance;//激光
	//激光 超声波同时矫正前进  
	if(distance_model==0){
		F_SPEED = -positional_PID(&F_Distance_pid_obj, &Distance_pid_param);
		R_SPEED = -positional_PID(&R_Distance_pid_obj, &Distance_pid_param);
	}
	else if(distance_model==1||distance_model==3){
		F_SPEED = -positional_PID(&F_Distance_pid_obj, &Distance_pid_param);

	}
	else if(distance_model==2||distance_model==4){
		R_SPEED = -positional_PID(&R_Distance_pid_obj, &Distance_pid_param);
	}
	
//	if(fabs(F_SPEED)<5)
//		F_SPEED = 0;
	if(fabs(R_SPEED)<5)
		R_SPEED = 0;
//	if(fabs(GGspeed)<2)
//		GGspeed = 0;
//	
	if(GGspeed >= motor_all.GyroG_speedMax) 
		GGspeed = motor_all.GyroG_speedMax;
	else if(GGspeed <= -motor_all.GyroG_speedMax) 
		GGspeed = -motor_all.GyroG_speedMax;
	
	w = chassis_w + GGspeed; //补偿w速度
	
	if(F_SPEED >= motor_all.SPEED_MAX) 
		F_SPEED = motor_all.SPEED_MAX;
	else if(F_SPEED <= -motor_all.SPEED_MAX) 
		F_SPEED = -motor_all.SPEED_MAX;
	
	if(R_SPEED >= motor_all.SPEED_MAX) 
		R_SPEED = motor_all.SPEED_MAX;
	else if(R_SPEED <= -motor_all.SPEED_MAX) 
		R_SPEED = -motor_all.SPEED_MAX;
	
	if(distance_model==0){
		
		motor_L0.target =  (F_SPEED +  R_SPEED + Radius_[1] * w);
		motor_L1.target = -(F_SPEED -  R_SPEED + Radius_[1] * w);		
		motor_R0.target = -(F_SPEED -  R_SPEED - Radius_[1] * w);	
		motor_R1.target =  (F_SPEED +  R_SPEED - Radius_[2] * w);
		
	}
	else if(distance_model==1){
		
		motor_L0.target =  (F_SPEED +  R_SPEED + Radius_[1] * w);
		motor_L1.target = -(F_SPEED -  R_SPEED + Radius_[1] * w);		
		motor_R0.target = -(F_SPEED -  R_SPEED - Radius_[1] * w);	
		motor_R1.target =  (F_SPEED +  R_SPEED - Radius_[2] * w);
		
	}
	else if(distance_model==2){
		
		motor_L0.target =  (F_SPEED +  R_SPEED + Radius_[1] * w);
		motor_L1.target = -(F_SPEED -  R_SPEED + Radius_[1] * w);		
		motor_R0.target = -(F_SPEED -  R_SPEED - Radius_[1] * w);	
		motor_R1.target =  (F_SPEED +  R_SPEED - Radius_[2] * w);
		
	}
	else if(distance_model==3||distance_model==4){
		
		motor_L0.target =  (F_SPEED +  R_SPEED + Radius_[1] * w);
		motor_L1.target = -(F_SPEED -  R_SPEED + Radius_[1] * w);		
		motor_R0.target = -(F_SPEED -  R_SPEED - Radius_[1] * w);	
		motor_R1.target =  (F_SPEED +  R_SPEED - Radius_[2] * w);
		
	}
	return 1;
}
/*****************************************************************************
函数功能：差速转（最好还是用上pid吧）
*****************************************************************************/
void AdCircle(float speed, float radius) 
{
	motor_all.Lspeed = speed - radius;
	motor_all.Rspeed = speed + radius;	
}

void Zhuang(int dir,int speed,int time)
{
			F_SPEED = 0;
			R_SPEED = 0;				//速度都清零
	    K=1;
			switch(dir)
			{
				case 0:
					x=speed;y=0;break;
				case 1:
					x= 0; y=speed;break;    //往墙角撞
			}
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
			vTaskDelay(time);

			speed_clear();			//速度都清零
			CarBrake(); //停车
	    K=0;
}
