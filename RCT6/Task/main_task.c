#include "main_task.h"
#include "buzzer.h"
#include "Start_task.h"
#include "motor.h"
#include "turn.h"
#include "motor_task.h"
#include "math.h"
#include "speed_ctrl.h"
#include "uart.h"
#include "usart.h"
#include "encoder.h"
#include "scaner.h"
#include "uart.h"
#include "delay.h"
#include "math.h"
#include "turn.h"
#include "voice.h"
#include "C8T6.h"
#include "Rudder_control.h"
#include "light.h"
#define CT86_UART UART2_Handler
float aim_distance[4] = {0,30.0f,8.0f,8};  // 0---��ʼ���  1 ������������    2--�����������ң�   3 ---���ϲ�߾���
uint8_t aim[2] = {0,0};
uint8_t temp=0;
float a  = 0;
#define small_z_speed 40;
int z_speed=80;
#define Normal_speed 270;
#define Slow_speed 100;
TaskHandle_t main_task_handler;
int time_side = 1000;
int time_middle = 1100;
int task_mode = 0;


void main_task(void *pvParameters){
	GYRO_Start();	 	 //�����ǳ�ʼ��
	motor_task_create();   //���ӿ�ʼ�˶�
	encoder_clear();
	int time=0;

	extern float x, y, w,chassis_w;
//	

//		
	// ǰ�� ---------- 24���� 13������ 0 both
		CarBrake(); 
		while((distances.b_distance==0) || (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)) )
		{
		  vTaskDelay(3);
		}
		max_fliter=1;
		
		angle.AngleG = getAngleZ();
		F_SPEED = small_z_speed;
		a = aim_distance[0];
		R_Distance_pid_obj.target=aim_distance[0] ;//���ּ���ԭʼ����
		distance_model = 2;//����
		pid_mode_switch(is_Distance);
		Distance_pid_param.kp = 4;
		vTaskDelay(1000);//�ȹ����������ȶ���
		F_SPEED = Normal_speed;
		while(distances.b_distance > 70)//�Ѿ����ά��100���ײ����
		{
			vTaskDelay(3);
		}
		//-------���ά�뵲��
		F_SPEED = 0;R_SPEED = 0;
		F_Distance_pid_obj.target = aim_distance[1];//��ߵ�
		distance_model = 3;//both
		pid_mode_switch(is_Distance);

		
		
		
		//-----ѡ��·��
		while((QR_information[1]==0||QR_information[3]==0 ) || (fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
		{
			vTaskDelay(2);
		}
		
		
		aim[0] = QR_information[1];
		aim[1] = QR_information[3];
		max_fliter=0;
		
		if(task_mode == 0)
		{
		if(aim[0] == 0x01)  //���
		{
			F_Distance_pid_obj.target = aim_distance[1];//��ߵ�
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1))
			{
			vTaskDelay(2);
			}
			
			
	    speed_clear();
			buzzer_on();

			x = -small_z_speed;
			y = 0;    
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//С�ٶ�����
			x = -Normal_speed;
			y = 0;
			
			infrared.open=1;
			while(infrared.left)
			{
				vTaskDelay(3);
			}
			buzzer_off();
			infrared.open=0;

			Zhuang(y_dir,z_speed,time_side+800);
			angle.AngleG = getAngleZ();
			printf("����1���ջ��㣬��ȡ�����ﲢ�رղ���");
			
		}
		else if(aim[0] == 0x02)//�ұ�
		{
			F_Distance_pid_obj.target = aim_distance[2];//�ұߵ�
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
			{
			vTaskDelay(2);
			}
			
			speed_clear();
			
			R_SPEED = Normal_speed;
			F_Distance_pid_obj.target=aim_distance[2] ;//���ֳ�����ԭʼ����
			distance_model = 3;//������
			pid_mode_switch(is_Distance);
			infrared.open=1;
			while(infrared.right)
			{
				vTaskDelay(3);
			}//ײ�ұ�
			infrared.open=0;
			
			Zhuang(y_dir,z_speed,time_side);
			angle.AngleG = getAngleZ();
			printf("����2���ջ��㣬��ȡ�����ﲢ�رղ���");
			
		}
		
	
		}	
		
		
		
		
		
		//�ϰ������֮���ģʽ

		else if(task_mode == 1)
		{
		
		if(aim[0] == 0x01)  //���
		{
			
			F_Distance_pid_obj.target = aim_distance[2];//
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
			{
			vTaskDelay(2);
			}
			
			speed_clear();
			
			R_SPEED = -Normal_speed;
			F_Distance_pid_obj.target=aim_distance[2] ;//���ֳ�����ԭʼ����
			distance_model = 3;//������
			pid_mode_switch(is_Distance);
			while(infrared.left)
			{
				vTaskDelay(3);
			}//ײ�ұ�
			
      Zhuang(y_dir,z_speed,time_side);
			angle.AngleG = getAngleZ();
			printf("����1���ջ��㣬��ȡ�����ﲢ�رղ���");
			
			
		}
		else if(aim[0] == 0x02)//�ұ�
		{
			F_Distance_pid_obj.target = aim_distance[1]-8;//��ߵ�
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
			{
			vTaskDelay(2);
			}
			
			
			speed_clear();
			buzzer_on();

			x = small_z_speed;
			y = 0;    //��ǽ��ײ
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//С�ٶ�����
			x = Normal_speed;
			y = 0;
			
			while(infrared.right)
			{
				vTaskDelay(3);
			}
			buzzer_off();
			
			
			Zhuang(y_dir,z_speed,time_side+800);
			angle.AngleG = getAngleZ();
			printf("����2���ջ��㣬��ȡ�����ﲢ�رղ���");
			
			
		}
		


		
		}
		
		

		mpuZreset(angle.AngleG,0);
		vTaskDelay(100);
		temp=0x01;
		print_mode =1;
		printf("%c",temp);//����c8t6��ʾ����
		print_mode =0;
		
		CarBrake();
		//ִ�п��Ų���
		while(QR_information[0]!=0)
		{
			time++;
			if(time>=1000)
				break;
			vTaskDelay(2);
		}//ȡ����

		
		Distance_pid_param.kp = 4;
		
		//----����ѡ��·����
		if(task_mode == 0)
		{
		if((aim[0] == 0x01) && (aim[1] == 0x01))//��� ���-->���� ��ͷ
		{
//		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
			printf("�����ջ����ȡ������");
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//����c8t6��ʾ����
			print_mode =0;
			//ִ�п��Ų���
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//ȡ����
		}
		
		else if((aim[0] == 0x01) && (aim[1] == 0x02))//��� �ұ�-->���� ��ͷ
		{
			//����
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(100);//С�ٶ�����
			
			speed_clear();
			F_Distance_pid_obj.target = aim_distance[1]-5;
			distance_model = 3;//������
			pid_mode_switch(is_Distance);
			//�˳�������50cm�ľ���
			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>2.0f))
			{
				vTaskDelay(3);
			}//���˽���
			
			
			speed_clear();

			x = small_z_speed;
			//angle.AngleG = getAngleZ();//���渳ֵ��
			pid_mode_switch(is_Gyro);
			vTaskDelay(200);
			
			x = Normal_speed;
		  vTaskDelay(1000);//����
			
			
			
//			R_SPEED = 0;
//			F_SPEED = 0;
//			F_Distance_pid_obj.target = aim_distance[2]+1;
//			distance_model = 3;//������
//			pid_mode_switch(is_Distance);
//			
//			//�˳���һ��
//			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
//			{
//				vTaskDelay(3);
//			}//���˽���
//			
//			speed_clear();

			x = Normal_speed; y = 70;
			pid_mode_switch(is_Gyro);
			vTaskDelay(800);
			y = 0;
			x = Normal_speed;
			pid_mode_switch(is_Gyro);
		  while(infrared.right)
			{
				vTaskDelay(3);
			}
			
      Zhuang(y_dir,z_speed,time_side);
			
			printf("����2���ջ����ȡ������");
			angle.AngleG = getAngleZ();  //ײƽ ��ȡ�½Ƕ�
		  mpuZreset(angle.AngleG,0);	
			vTaskDelay(100);
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//����c8t6��ʾ����
			print_mode =0;
			
			//ִ�п��Ų���
			CarBrake();
			time=0;
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//ȡ����
				time=0;
			
			
			
			//ȡ������֮��ؼ�
			//������
//			speed_clear();
//			F_Distance_pid_obj.target = aim_distance[2];
//			distance_model = 3;//csb
//			pid_mode_switch(is_Distance);
//			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
//			{
//				vTaskDelay(2);

//			}

			//ZUO��
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(200);//С�ٶ�����
			speed_clear();
			R_SPEED = -40;
			vTaskDelay(100);
		  R_SPEED = -Normal_speed;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[2]+2;
			distance_model = 3;//
			pid_mode_switch(is_Distance);
			vTaskDelay(1350);


			//��ǰײ
			Zhuang(y_dir,z_speed,time_middle);
			angle.AngleG = getAngleZ();  //ײƽ ��ȡ�½Ƕ�
		 
		  //���ⶨλ
			buzzer_on();
			R_SPEED = 0;
			F_SPEED = 0;
			R_Distance_pid_obj.target = aim_distance[0]+4;
			distance_model = 4;
			pid_mode_switch(is_Distance);
			while((fabs(R_Distance_pid_obj.target - R_Distance_pid_obj.measure)>1.0f))
			{
				vTaskDelay(2);
			}
			
			
			//��ǰײ
			Zhuang(y_dir,z_speed,500);
			angle.AngleG = getAngleZ();  //ײƽ ��ȡ�½Ƕ�
		  mpuZreset(angle.AngleG,0);		
			vTaskDelay(100);
			buzzer_off();
		}
		
		else if((aim[0] == 0x02) && (aim[1] == 0x01))//�ұ� ���-->���� ��ͷ
		{
//			speed_clear();
//			F_Distance_pid_obj.target = aim_distance[2];
//			distance_model = 3;//������
//			pid_mode_switch(is_Distance);
//			//�˳�������50cm�ľ���
//			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
//			{
//				vTaskDelay(3);
//			}//���˽���
//			
//			speed_clear();
			//����
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(200);//С�ٶ�����
			speed_clear();
			
			
			R_SPEED = -Normal_speed;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[2]+ 4;
			distance_model = 3;//
			pid_mode_switch(is_Distance);
			
			
			vTaskDelay(1500);
			CarBrake();
			vTaskDelay(500);
			
			speed_clear();
			
			F_Distance_pid_obj.target = aim_distance[1]-4;
			distance_model = 3;//csb
			pid_mode_switch(is_Distance);
			buzzer_on();

			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
		{
			vTaskDelay(2);

		}

			buzzer_off();
		
		
			F_SPEED = 0;
			R_SPEED = 0;	
			x = -small_z_speed; 
		  y = 0;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//�ȶ�����
		
			x = -Normal_speed;
		  y = 0;
			while(infrared.left)
			{
				vTaskDelay(2);
			}
			
			
      Zhuang(y_dir,z_speed,time_side+800);
			printf("����1���ջ����ȡ������");
			angle.AngleG = getAngleZ();  //ײƽ ��ȡ�½Ƕ�
	   	    mpuZreset(angle.AngleG,0);
			vTaskDelay(500);
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//����c8t6��ʾ����
			print_mode =0;

			//ִ�п��Ų���
			CarBrake();
			time=0;
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//ȡ����
			time=0;

			//ȡ������֮��ؼ�
//			speed_clear();
//			Distance_pid_param.kp = 4;
//			buzzer_on();
//			x = 0;
//			y = -40;   
//			angle.AngleG = getAngleZ();
//			pid_mode_switch(is_Gyro);
//    	vTaskDelay(500);//С�ٶ�����
//			speed_clear();
			R_SPEED = 0;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[1]-6;
		
			distance_model = 3;//������
			pid_mode_switch(is_Distance);
			//�˳�������50cm�ľ���
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
			{
				vTaskDelay(2);
			}
			
			
			speed_clear();

			x = small_z_speed;
			//angle.AngleG = getAngleZ();//���渳ֵ��
			pid_mode_switch(is_Gyro);
			vTaskDelay(500);
			

			x = Normal_speed;
			vTaskDelay(1240);//����
		
			//��ǰײ,�ܿ��ϰ���֮�󼤹ⶨλ
      Zhuang(y_dir,70,800);
//			angle.AngleG = getAngleZ();  //ײƽ ��ȡ�½Ƕ�
//		  mpuZreset(angle.AngleG,0);		
			buzzer_on();
//			vTaskDelay(300);
		  //���ⶨλ	
			R_SPEED = 0;
			F_SPEED = 0;
			R_Distance_pid_obj.target = aim_distance[0]+4;
			distance_model = 4;
			pid_mode_switch(is_Distance);
			//���˳���
			while((fabs(R_Distance_pid_obj.target - R_Distance_pid_obj.measure)>1.0f))
			{
				vTaskDelay(2);
			}
		  buzzer_off();
			//��ǰײ
			Zhuang(y_dir,80,1000);
			angle.AngleG = getAngleZ();  //ײƽ ��ȡ�½Ƕ�
		  mpuZreset(angle.AngleG,0);		
			vTaskDelay(500);
			buzzer_on();
			
		}
		
		else if((aim[0] == 0x02) && (aim[1] == 0x02))//�ұ� �ұ�-->���� ��ͷ
		{
			printf("����2���ջ����ȡ������");
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//����c8t6��ʾ����
			print_mode =0;
			//ִ�п��Ų���
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//ȡ����
		}
	}
		
	
	
	
	
	
	
	else if(task_mode == 1)
	{
		if((aim[0] == 0x01) && (aim[1] == 0x01))//��� ���-->���� ��ͷ
		{
//		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
			printf("�����ջ����ȡ������");
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//����c8t6��ʾ����
			print_mode =0;
			//ִ�п��Ų���
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//ȡ����
		}
		
		else if((aim[0] == 0x01) && (aim[1] == 0x02))//��� �ұ�-->���� ��ͷ
		{
		speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//С�ٶ�����
			speed_clear();
//			F_Distance_pid_obj.target = aim_distance[2];
//			distance_model = 3;//������
//			pid_mode_switch(is_Distance);
//			//�˳�������
//			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
//			{
//				vTaskDelay(3);
//			}//���˽���
//			
//			speed_clear();
			
			//����
		
			
			
			R_SPEED = Normal_speed;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[2];
			distance_model = 3;//
			pid_mode_switch(is_Distance);
			vTaskDelay(1350);
			CarBrake();
			vTaskDelay(500);
			
			speed_clear();
			
			F_Distance_pid_obj.target = aim_distance[1]-4;
			distance_model = 3;//csb
			pid_mode_switch(is_Distance);
			buzzer_on();

			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
		{
			vTaskDelay(2);

		}

			buzzer_off();
		
		
			F_SPEED = 0;
			R_SPEED = 0;	
			x = small_z_speed; 
		  y = 0;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//�ȶ�����
		
			x = Normal_speed;
		  y = 0;
			while(infrared.right)
			{
				vTaskDelay(3);
			}
			
			
      Zhuang(y_dir,z_speed,time_side+800);
			
			printf("����2���ջ����ȡ������");
			angle.AngleG = getAngleZ();  //ײƽ ��ȡ�½Ƕ�
		  mpuZreset(angle.AngleG,0);
			vTaskDelay(500);
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//����c8t6��ʾ����
			print_mode =0;

			//ִ�п��Ų���
			CarBrake();
			time=0;
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//ȡ����
			time=0;

			//ȡ������֮��ؼ�
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//С�ٶ�����
			speed_clear();
			R_SPEED = 0;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[1]-3;
		
			distance_model = 3;//������
			pid_mode_switch(is_Distance);
			//�˳�������50cm�ľ���
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
			{
				vTaskDelay(2);
			}
			
			
			speed_clear();

			x = -small_z_speed;
			//angle.AngleG = getAngleZ();//���渳ֵ��
			pid_mode_switch(is_Gyro);
			vTaskDelay(500);
			x = -Normal_speed;
			
			vTaskDelay(1000);//����

		
			

		  //���ⶨλ	
			R_SPEED = 0;
			F_SPEED = 0;
			R_Distance_pid_obj.target = aim_distance[0];
			distance_model = 4;
			pid_mode_switch(is_Distance);
			//���˳���
			while((fabs(R_Distance_pid_obj.target - R_Distance_pid_obj.measure)>1.0f))
			{
				vTaskDelay(2);
			}
		
			//��ǰײ
      Zhuang(y_dir,100,1800);
			angle.AngleG = getAngleZ();  //ײƽ ��ȡ�½Ƕ�
		  mpuZreset(angle.AngleG,0);		
			vTaskDelay(500);
		}
		
		else if((aim[0] == 0x02) && (aim[1] == 0x01))//you->zuo
		{
			//����
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//С�ٶ�����
			speed_clear();
			speed_clear();
			F_Distance_pid_obj.target = aim_distance[1]-4;
			distance_model = 3;//������
			pid_mode_switch(is_Distance);
			//�˳�������50cm�ľ���
			buzzer_on();
//			CarBrake();
//			vTaskDelay(500);
			pid_mode_switch(is_Distance);
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.0f))
			{
				vTaskDelay(2);
			}//���˽���
			buzzer_off();

			
			speed_clear();

			x = -small_z_speed;
			//angle.AngleG = getAngleZ();//���渳ֵ��
			pid_mode_switch(is_Gyro);
			
			x = -Normal_speed;
		  vTaskDelay(1400);//����
			
			
			
			R_SPEED = 0;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[2]+1;
			distance_model = 3;//������
			pid_mode_switch(is_Distance);
			
			//�˳���һ��
			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
			{
				vTaskDelay(2);
			}//���˽���
			
			speed_clear();

			x = -Normal_speed;
			pid_mode_switch(is_Gyro);
			
//			for(int i = 0; i<2 ; i++)
//			{
//					vTaskDelay(910);//����
//			}
		  while(infrared.left)
			{
				vTaskDelay(3);
			}
			
      Zhuang(y_dir,z_speed,time_side);
			printf("����1���ջ����ȡ������");
			angle.AngleG = getAngleZ();  //ײƽ ��ȡ�½Ƕ�
		  mpuZreset(angle.AngleG,0);	
			vTaskDelay(500);
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//����c8t6��ʾ����
			print_mode =0;
			
			//ִ�п��Ų���
			CarBrake();
			time=0;
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//ȡ����
				time=0;
			
			
			
			//ȡ������֮��ؼ�
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//С�ٶ�����
			speed_clear();
			//������
			speed_clear();
			F_Distance_pid_obj.target = aim_distance[2];
			distance_model = 3;//csb
			pid_mode_switch(is_Distance);
			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
			{
				vTaskDelay(2);

			}

			//����
		  R_SPEED = Normal_speed;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[2];
			distance_model = 3;//
			pid_mode_switch(is_Distance);
			vTaskDelay(1400);

			//����ײ
      Zhuang(y_dir,-z_speed,600);
	
		  //���ⶨλ
			buzzer_on();
			R_SPEED = 0;
			F_SPEED = 0;
			R_Distance_pid_obj.target = aim_distance[0];
			distance_model = 4;
			pid_mode_switch(is_Distance);
			while((fabs(R_Distance_pid_obj.target - R_Distance_pid_obj.measure)>1.0f))
			{
				vTaskDelay(2);
			}
			
			
			//��ǰײ
      {
					F_SPEED = 0;
					R_SPEED = 0;				//�ٶȶ�����
					K=1;
					x = 0; y = 100;   //��ǽ��ײ
					angle.AngleG = getAngleZ();
					pid_mode_switch(is_Gyro);
					vTaskDelay(1800);

					speed_clear();			//�ٶȶ�����
					CarBrake(); //ͣ��
					K=0;
			}
			angle.AngleG = getAngleZ();  //ײƽ ��ȡ�½Ƕ�
		  mpuZreset(angle.AngleG,0);		
			vTaskDelay(500);
			buzzer_off();
			
		}
		
		else if((aim[0] == 0x02) && (aim[1] == 0x02))//�ұ� �ұ�-->���� ��ͷ
		{
			printf("����2���ջ����ȡ������");
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//����c8t6��ʾ����
			print_mode =0;
			//ִ�п��Ų���
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//ȡ����
		}
	
	
	
	
	}
		
		
		
		
	
buzzer_off();
		//ͳһ��ֱ��
		 	speed_clear();
			Distance_pid_param.kp = 4;
			x = 0;
			y = -30;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//С�ٶ�����
			speed_clear();
			x = 0; y = -100;
			pid_mode_switch(is_Gyro);
			vTaskDelay(1400);
			x  = 0; y = 0;
			CarBrake(); //ͣ��
			vTaskDelay(300);
			max_fliter=1;
			F_SPEED = -150;
			Distance_pid_param.kp = 6;
			R_Distance_pid_obj.target=aim_distance[0]+4;//���ּ���ԭʼ����
			distance_model = 4;//����
			pid_mode_switch(is_Distance);
			vTaskDelay(500);
			
			F_SPEED = -250;
			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4))//�����
			{
				vTaskDelay(2);
			}
//			while(infrared.right)//�����
//			{
//				vTaskDelay(2);
//			}
			F_SPEED = -100;
			Distance_pid_param.kp = 4;
			R_Distance_pid_obj.target=aim_distance[0];//���ּ���ԭʼ����
			distance_model = 4;//����
			pid_mode_switch(is_Distance);
			vTaskDelay(300);
			while(1)
			{
			
				CarBrake();
			}
	}



//�����񴴽�
void main_task_create(void){
	  xTaskCreate((TaskFunction_t ) main_task,//������
	               (const char *)"main_task",	  //��������
								 (uint32_t) main_task_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) main_task_priority, //��������ȼ�
								(TaskHandle_t *)&main_task_handler ); //������
							 }
