#include "motor_task.h"
#include  "encoder.h"
#include 	"motor.h"
#include 	"uart.h"
#include "Start_task.h"
#include "speed_ctrl.h"
#include <string.h>
#include <stdlib.h>
#include "scaner.h"
#include "gyro.h"
#include "turn.h"
#include "math.h"
#define PI 3.1415926535f
extern float x, y, w,chassis_w;
float back_compensate = 0;  //���ֲ���
TaskHandle_t motor_handler;
int dirct[4] = {1,-1,1,-1};    //����
volatile uint8_t PIDMode;
uint8_t line_gyro_switch = 0;  
struct Find_line find_line = {0,0,0};
struct Infrared infrared = {0,0,0,0};

void motor_task(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //��ȡϵͳ����
	while(1){
			GET_MOTOR();
            infrared.open=1; 
			if(infrared.open){
				
			infrared.right = (uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9);//����ߺ��⣬ײ�ұ�
			infrared.left = (uint8_t)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14);
			infrared.back = (uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
			}
//			motor_all.F_encoder_avg = (float)((float)(pulse_num[0]*dirct[0] + pulse_out[0]*MAX_pulse) + (float)(pulse_num[1]*dirct[1] + pulse_out[1]*MAX_pulse)) / 2.0f;
//			motor_all.R_encoder_avg = (float)((float)(pulse_num[2]*dirct[2] + pulse_out[2]*MAX_pulse) + (float)(pulse_num[3]*dirct[3] + pulse_out[3]*MAX_pulse)) / 2.0f;
//			//�־�8.0cm
//			motor_all.F_Distance = (float)(motor_all.F_encoder_avg * 8.0f * PI)/620.0f;
//			motor_all.R_Distance = (float)(motor_all.R_encoder_avg * 8.0f * PI)/620.0f;
			
			/*  ---------�л�ģʽ��Ҫ����   -----------------*/
			//��������ƽ��->ѭ��
			if (line_gyro_switch == 1)
			{
				line_pid_obj = gyroG_pid;
				TC_speed = TG_speed;
				gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
				TG_speed = (struct Gradual){0,0,0};
				line_gyro_switch = 0;
			}
			//ѭ��->��������ƽ��
			else if (line_gyro_switch == 2)
			{
				gyroG_pid = line_pid_obj;
				TG_speed = TC_speed;
				line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
				TC_speed = (struct Gradual){0,0,0};
				line_gyro_switch = 0;
			}
			
			//ѭ������
			if (PIDMode == is_Line)
			{						
					  if(Line_Scan(&Scaner,LFB_SENSOR_NUM,scaner_set.EdgeIgnore)){
							if(find_line.id==1){
								  mpuZreset(imu.yaw,0);
									angle.AngleT = 10;
									find_line.help_open=1;
									pid_mode_switch(is_Turn);
							}
							else if(find_line.id==2){
								  mpuZreset(imu.yaw,0);
									angle.AngleT = -10;
									find_line.help_open=1;
									pid_mode_switch(is_Turn);								
							}
						}
						else{
							gradual_cal(&TC_speed, motor_all.Cspeed, motor_all.Cincrement);
							Go_Line(TC_speed.Now);
						}
			}
			else
				motor_all.Cspeed = 0;
				
			//ת��PID����      //ԭ��ת��
			if (PIDMode == is_Turn)	
			{
					if (Turn_Angle(angle.AngleT)){
						gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0};
					}
						
			}
			
		 //��ƽ��PID����
		 if (PIDMode == is_Gyro)	
			    { 
					gradual_cal(&TG_speed, motor_all.Gspeed, motor_all.Gincrement);			
					runWithAngle(angle.AngleG, TG_speed.Now);
				}
			else
					motor_all.Gspeed = 0;
		 if  (PIDMode == is_Distance)
		{
         Distance_run(angle.AngleG);
		 }
			if(PIDMode != is_Free){
											
				incremental_PID(&motor_L0, &L0_param);
				incremental_PID(&motor_L1, &L1_param);
				incremental_PID(&motor_R0, &R0_param);
				incremental_PID(&motor_R1, &R1_param);
				
				motor_set_pwm(1, (int32_t)motor_L0.output);
				motor_set_pwm(2, (int32_t)motor_L1.output);
				motor_set_pwm(3, (int32_t)motor_R0.output);
				motor_set_pwm(4, (int32_t)motor_R1.output);
			
//				motor_set_pwm(1, (int32_t)-5000);
//				motor_set_pwm(2, (int32_t)-5000);
//				motor_set_pwm(3, (int32_t)-5000);
//				motor_set_pwm(4, (int32_t)-5000);

			}
			else{
					motor_set_pwm(1, (int32_t)motor_L0.output);
					motor_set_pwm(2, (int32_t)motor_L1.output);
					motor_set_pwm(3, (int32_t)motor_R0.output);
					motor_set_pwm(4, (int32_t)motor_R1.output);
			}

//			printf("yaw:  %f   pitch: %f    %f\r\n",getAngleZ(),get_pitch(),imu.pitch);
//			printf("%f    %f   %f   %f \r\n",motor_L0.target,motor_L1.target,motor_R0.target,getAngleZ());
//			printf("%f    %f   %f\r\n",motor_all.Lspeed,motor_all.Rspeed,motor_all.Distance);
 //			  printf("%5f,%5f\r\n",motor_L0.measure,motor_L0.target);
			//printf("%d,%d,%d,%d\r\n",(int32_t)motor_L0.output,(int32_t)motor_L1.output,(int32_t)motor_R0.output,(int32_t)motor_R1.output);
			//Line_Scan(&Scaner,LFB_SENSOR_NUM,scaner_set.EdgeIgnore);
		// printf("%f    %f    %f   %d  %d\r\n",Scaner.error,motor_all.Lspeed,motor_all.Rspeed,Scaner.detail,Scaner.lineNum);
			//printf("%d\r\n",TIM7->CNT);
//		printf("%d,%d,%d,%d\r\n",(int)motor_L0.measure,(int)motor_L1.measure,(int)motor_R0.measure,(int)motor_R1.measure);
//			printf("%d,%d\r\n",(int)motor_R0.measure,(int)motor_R0.target);
			
//			printf("%5d  %5d %5d %5d\r\n",(int)pulse_num[0],(int)pulse_num[1],(int)pulse_num[2],(int)pulse_num[3]);
			//printf("%5d  %5d\r\n",(int)pulse_out[0],(int)pulse_num[0]);
			//printf("%5f\r\n",motor_all.encoder_avg);
//			printf("roll=%f,pitch=%f,yaw=%f \r\n",imu.roll,imu.pitch,imu.yaw);
//		   printf("%d   %d\r\n",nodesr.nowNode.nodenum,nodesr.nextNode.nodenum);
//			printf("%f  %f\r\n",nodesr.nowNode.speed,nodesr.nextNode.angle);
			vTaskDelayUntil(&xLastWakeTime, (5/portTICK_RATE_MS));//��������5ms // INCLUDE_vTaskDelayUntil 1
	}
}


//PID���񴴽�
void motor_task_create(void){
	  xTaskCreate((TaskFunction_t ) motor_task,//������
	               (const char *)"motor_task",	  //��������
								 (uint32_t) motor_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) motor_task_priority, //��������ȼ�
								(TaskHandle_t *)&motor_handler ); //������
							 }


void pid_mode_switch(uint8_t target_mode)
{
	switch (target_mode)
	{
		case is_Turn: {				//ת��ģʽ
			gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
			TG_speed = (struct Gradual){0,0,0};
			TC_speed = (struct Gradual){0,0,0};
			break;
		}
		
		case is_Line: {
			if (PIDMode == is_Gyro)  //����ƽ���л���ѭ��
			{
				line_gyro_switch = 1;
			}
			else
			{
				gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			}
			break;
		}
		
		case is_Gyro: {
			if (PIDMode == is_Line)  //��ѭ���л�����ƽ��
			{
				line_gyro_switch = 2;
			}
			else
			{
				gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			}
			break;
		}
		
		case is_Free: {
			break;
		}
		
		case is_No: {
			line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
			gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			TC_speed = (struct Gradual){0,0,0};
			TG_speed = (struct Gradual){0,0,0};
			break;
		}
	}
	PIDMode = target_mode;		//��ס��ǰ�л���ģʽ
}
//��ȡ���ӵ�ת��
void GET_MOTOR(void){
		if(high_time[0]==0)
			motor_L0.measure=0;
		else
			motor_L0.measure = (float)(100000/high_time[0]) * dirct[0] * direction[0] ;
		if(high_time[1]==0)
			motor_L1.measure=0;
		else
			motor_L1.measure = (float)(100000/high_time[1]) * dirct[1] * direction[1] ;
		if(high_time[2]==0)
			motor_R0.measure=0;
		else
			motor_R0.measure = (float)(100000/high_time[2]) * dirct[2] * direction[2] ;
		if(high_time[3]==0)
			motor_R1.measure=0;
		else
			motor_R1.measure = (float)(100000/high_time[3]) * dirct[3] * direction[3] ;
}

void speed_clear(void)
{
			F_SPEED = 0;
			R_SPEED = 0;				//�ٶȶ�����
			y = 0;x = 0;
}