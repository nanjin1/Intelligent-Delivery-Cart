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
//�����ǰ�Ƕ���Ŀ��Ƕȵ���С�н� 
float need2turn(float nowangle,float targetangle)
{			
	float need2Turn;		

	need2Turn=targetangle-nowangle;		//ʵ������ת�ĽǶ�
	if(need2Turn>180)	need2Turn -= 360;
  else if(need2Turn<-180)	need2Turn += 360;
	
  return need2Turn;		
}



//��������У׼   //ת���     //�������Ƕ�     //�ο��Ƕ�	 �õ���ǰ�ǶȺͲο��ǶȵĲ��
void mpuZreset(float sensorangle ,float referangle)
{	
	imu.compensateZ=need2turn(sensorangle,referangle);
}

//��ȡZ�Ƕ�					
float getAngleZ(void) 
{
	float targetangle;
	targetangle = imu.yaw + imu.compensateZ;	

	if(targetangle>180)	targetangle -= 360;
	else if(targetangle<-180)	targetangle += 360;
	
	return targetangle;
}

/*****************************************************************************
������д�˼��������ڣ���������7-19
�������ܣ�������Z��ת�Ƕȣ�ת��ԽǶ�
���룺Ҫת�ĽǶ� ���ٶ�
�������
*****************************************************************************/
void Turn_Angle_Relative(float Angle1)//��180����-180,�ٶȱ��������ģ�
{
	float Turn_Angle_Before = 0, Turn_Angle_Targe = 0;
	
	Turn_Angle_Before = getAngleZ();//��ȡ��ǰ�ĽǶ�//@@@@@
	Turn_Angle_Targe = Turn_Angle_Before+Angle1;//Ŀ��Ƕ���Ϊ��������
	/*******************��������ٽ�״̬����Ŀ��Ƕ�ת��Ϊ��������******180 0 -180*************/
    if(Turn_Angle_Targe>180){				
    	Turn_Angle_Targe=Turn_Angle_Targe-360;
    }
    else if(Turn_Angle_Targe<-180){
    	Turn_Angle_Targe=Turn_Angle_Targe+360;
    }
	
	angle.AngleT = Turn_Angle_Targe;
	pid_mode_switch(is_Turn);  //����ת��
}

/*****************************************************************************
��+�����ܣ�������Z����PIDԭ��ת�Ƕ�
					��Ҫת���ĽǶ�(���ԽǶ�)
					�÷���Target = ?,PIDMode = is_Turn,while(fabs(getAngleZ()-Target)<1);
*****************************************************************************/
uint8_t Turn_Angle(float Angle)	
{
	float GTspeed, now_angle;
	
	//�ٽ紦��
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
//ƽ̨ת��
uint8_t Stage_turn_Angle(float Angle)	
{
	float GTspeed, now_angle;
	
	//�ٽ紦��
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
�������ܣ�������Z����ƽ������ diretion=0 ǰ��  diretion=1  ����
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
		GGspeed = -motor_all.GyroG_speedMax;					//�������ֵ

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
�������ܣ�������λ��ʽ��ƽ������
*****************************************************************************/
uint8_t Distance_run(float angle_want)
{
	float GGspeed, now_angle;
	
	now_angle = getAngleZ();
	
	gyroG_pid.measure = need2turn(now_angle, angle_want);  
	gyroG_pid.target = 0;
	
	GGspeed = positional_PID(&gyroG_pid, &gyroG_pid_param);
	
	F_Distance_pid_obj.measure = distances.b_distance;//������
	R_Distance_pid_obj.measure = distances.l_distance;//����
	//���� ������ͬʱ����ǰ��  
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
	
	w = chassis_w + GGspeed; //����w�ٶ�
	
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
�������ܣ�����ת����û�������pid�ɣ�
*****************************************************************************/
void AdCircle(float speed, float radius) 
{
	motor_all.Lspeed = speed - radius;
	motor_all.Rspeed = speed + radius;	
}

void Zhuang(int dir,int speed,int time)
{
			F_SPEED = 0;
			R_SPEED = 0;				//�ٶȶ�����
	    K=1;
			switch(dir)
			{
				case 0:
					x=speed;y=0;break;
				case 1:
					x= 0; y=speed;break;    //��ǽ��ײ
			}
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
			vTaskDelay(time);

			speed_clear();			//�ٶȶ�����
			CarBrake(); //ͣ��
	    K=0;
}
