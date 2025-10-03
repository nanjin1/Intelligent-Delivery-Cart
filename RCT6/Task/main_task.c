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
float aim_distance[4] = {0,30.0f,8.0f,8};  // 0---初始侧边  1 超声波（向左）    2--超声波（向右）   3 ---物料侧边距离
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
	GYRO_Start();	 	 //陀螺仪初始化
	motor_task_create();   //轮子开始运动
	encoder_clear();
	int time=0;

	extern float x, y, w,chassis_w;
//	

//		
	// 前进 ---------- 24激光 13超声波 0 both
		CarBrake(); 
		while((distances.b_distance==0) || (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)) )
		{
		  vTaskDelay(3);
		}
		max_fliter=1;
		
		angle.AngleG = getAngleZ();
		F_SPEED = small_z_speed;
		a = aim_distance[0];
		R_Distance_pid_obj.target=aim_distance[0] ;//保持激光原始距离
		distance_model = 2;//激光
		pid_mode_switch(is_Distance);
		Distance_pid_param.kp = 4;
		vTaskDelay(1000);//度过超声波不稳定期
		F_SPEED = Normal_speed;
		while(distances.b_distance > 70)//已经离二维码100厘米差不多了
		{
			vTaskDelay(3);
		}
		//-------离二维码挡板
		F_SPEED = 0;R_SPEED = 0;
		F_Distance_pid_obj.target = aim_distance[1];//左边的
		distance_model = 3;//both
		pid_mode_switch(is_Distance);

		
		
		
		//-----选择路径
		while((QR_information[1]==0||QR_information[3]==0 ) || (fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
		{
			vTaskDelay(2);
		}
		
		
		aim[0] = QR_information[1];
		aim[1] = QR_information[3];
		max_fliter=0;
		
		if(task_mode == 0)
		{
		if(aim[0] == 0x01)  //左边
		{
			F_Distance_pid_obj.target = aim_distance[1];//左边的
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
    	vTaskDelay(500);//小速度启动
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
			printf("到达1号收货点，请取出货物并关闭仓门");
			
		}
		else if(aim[0] == 0x02)//右边
		{
			F_Distance_pid_obj.target = aim_distance[2];//右边的
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
			{
			vTaskDelay(2);
			}
			
			speed_clear();
			
			R_SPEED = Normal_speed;
			F_Distance_pid_obj.target=aim_distance[2] ;//保持超声波原始距离
			distance_model = 3;//超声波
			pid_mode_switch(is_Distance);
			infrared.open=1;
			while(infrared.right)
			{
				vTaskDelay(3);
			}//撞右边
			infrared.open=0;
			
			Zhuang(y_dir,z_speed,time_side);
			angle.AngleG = getAngleZ();
			printf("到达2号收货点，请取出货物并关闭仓门");
			
		}
		
	
		}	
		
		
		
		
		
		//障碍物更换之后的模式

		else if(task_mode == 1)
		{
		
		if(aim[0] == 0x01)  //左边
		{
			
			F_Distance_pid_obj.target = aim_distance[2];//
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
			{
			vTaskDelay(2);
			}
			
			speed_clear();
			
			R_SPEED = -Normal_speed;
			F_Distance_pid_obj.target=aim_distance[2] ;//保持超声波原始距离
			distance_model = 3;//超声波
			pid_mode_switch(is_Distance);
			while(infrared.left)
			{
				vTaskDelay(3);
			}//撞右边
			
      Zhuang(y_dir,z_speed,time_side);
			angle.AngleG = getAngleZ();
			printf("到达1号收货点，请取出货物并关闭仓门");
			
			
		}
		else if(aim[0] == 0x02)//右边
		{
			F_Distance_pid_obj.target = aim_distance[1]-8;//左边的
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
			{
			vTaskDelay(2);
			}
			
			
			speed_clear();
			buzzer_on();

			x = small_z_speed;
			y = 0;    //往墙角撞
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//小速度启动
			x = Normal_speed;
			y = 0;
			
			while(infrared.right)
			{
				vTaskDelay(3);
			}
			buzzer_off();
			
			
			Zhuang(y_dir,z_speed,time_side+800);
			angle.AngleG = getAngleZ();
			printf("到达2号收货点，请取出货物并关闭仓门");
			
			
		}
		


		
		}
		
		

		mpuZreset(angle.AngleG,0);
		vTaskDelay(100);
		temp=0x01;
		print_mode =1;
		printf("%c",temp);//发给c8t6表示到了
		print_mode =0;
		
		CarBrake();
		//执行开门操作
		while(QR_information[0]!=0)
		{
			time++;
			if(time>=1000)
				break;
			vTaskDelay(2);
		}//取出了

		
		Distance_pid_param.kp = 4;
		
		//----二次选择路径·
		if(task_mode == 0)
		{
		if((aim[0] == 0x01) && (aim[1] == 0x01))//左边 左边-->不动 回头
		{
//		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
			printf("到达收获点请取出物料");
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//发给c8t6表示到了
			print_mode =0;
			//执行开门操作
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//取出了
		}
		
		else if((aim[0] == 0x01) && (aim[1] == 0x02))//左边 右边-->右走 回头
		{
			//后退
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(100);//小速度启动
			
			speed_clear();
			F_Distance_pid_obj.target = aim_distance[1]-5;
			distance_model = 3;//超声波
			pid_mode_switch(is_Distance);
			//退出来保持50cm的距离
			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>2.0f))
			{
				vTaskDelay(3);
			}//后退结束
			
			
			speed_clear();

			x = small_z_speed;
			//angle.AngleG = getAngleZ();//上面赋值了
			pid_mode_switch(is_Gyro);
			vTaskDelay(200);
			
			x = Normal_speed;
		  vTaskDelay(1000);//右移
			
			
			
//			R_SPEED = 0;
//			F_SPEED = 0;
//			F_Distance_pid_obj.target = aim_distance[2]+1;
//			distance_model = 3;//超声波
//			pid_mode_switch(is_Distance);
//			
//			//退出来一点
//			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
//			{
//				vTaskDelay(3);
//			}//后退结束
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
			
			printf("到达2号收获点请取出物料");
			angle.AngleG = getAngleZ();  //撞平 获取新角度
		  mpuZreset(angle.AngleG,0);	
			vTaskDelay(100);
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//发给c8t6表示到了
			print_mode =0;
			
			//执行开门操作
			CarBrake();
			time=0;
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//取出了
				time=0;
			
			
			
			//取出物料之后回家
			//纯后退
//			speed_clear();
//			F_Distance_pid_obj.target = aim_distance[2];
//			distance_model = 3;//csb
//			pid_mode_switch(is_Distance);
//			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
//			{
//				vTaskDelay(2);

//			}

			//ZUO移
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(200);//小速度启动
			speed_clear();
			R_SPEED = -40;
			vTaskDelay(100);
		  R_SPEED = -Normal_speed;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[2]+2;
			distance_model = 3;//
			pid_mode_switch(is_Distance);
			vTaskDelay(1350);


			//往前撞
			Zhuang(y_dir,z_speed,time_middle);
			angle.AngleG = getAngleZ();  //撞平 获取新角度
		 
		  //激光定位
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
			
			
			//往前撞
			Zhuang(y_dir,z_speed,500);
			angle.AngleG = getAngleZ();  //撞平 获取新角度
		  mpuZreset(angle.AngleG,0);		
			vTaskDelay(100);
			buzzer_off();
		}
		
		else if((aim[0] == 0x02) && (aim[1] == 0x01))//右边 左边-->左走 回头
		{
//			speed_clear();
//			F_Distance_pid_obj.target = aim_distance[2];
//			distance_model = 3;//超声波
//			pid_mode_switch(is_Distance);
//			//退出来保持50cm的距离
//			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
//			{
//				vTaskDelay(3);
//			}//后退结束
//			
//			speed_clear();
			//左走
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(200);//小速度启动
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
    	vTaskDelay(500);//稳定启动
		
			x = -Normal_speed;
		  y = 0;
			while(infrared.left)
			{
				vTaskDelay(2);
			}
			
			
      Zhuang(y_dir,z_speed,time_side+800);
			printf("到达1号收获点请取出物料");
			angle.AngleG = getAngleZ();  //撞平 获取新角度
	   	    mpuZreset(angle.AngleG,0);
			vTaskDelay(500);
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//发给c8t6表示到了
			print_mode =0;

			//执行开门操作
			CarBrake();
			time=0;
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//取出了
			time=0;

			//取出物料之后回家
//			speed_clear();
//			Distance_pid_param.kp = 4;
//			buzzer_on();
//			x = 0;
//			y = -40;   
//			angle.AngleG = getAngleZ();
//			pid_mode_switch(is_Gyro);
//    	vTaskDelay(500);//小速度启动
//			speed_clear();
			R_SPEED = 0;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[1]-6;
		
			distance_model = 3;//超声波
			pid_mode_switch(is_Distance);
			//退出来保持50cm的距离
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
			{
				vTaskDelay(2);
			}
			
			
			speed_clear();

			x = small_z_speed;
			//angle.AngleG = getAngleZ();//上面赋值了
			pid_mode_switch(is_Gyro);
			vTaskDelay(500);
			

			x = Normal_speed;
			vTaskDelay(1240);//右移
		
			//往前撞,避开障碍物之后激光定位
      Zhuang(y_dir,70,800);
//			angle.AngleG = getAngleZ();  //撞平 获取新角度
//		  mpuZreset(angle.AngleG,0);		
			buzzer_on();
//			vTaskDelay(300);
		  //激光定位	
			R_SPEED = 0;
			F_SPEED = 0;
			R_Distance_pid_obj.target = aim_distance[0]+4;
			distance_model = 4;
			pid_mode_switch(is_Distance);
			//不退出来
			while((fabs(R_Distance_pid_obj.target - R_Distance_pid_obj.measure)>1.0f))
			{
				vTaskDelay(2);
			}
		  buzzer_off();
			//往前撞
			Zhuang(y_dir,80,1000);
			angle.AngleG = getAngleZ();  //撞平 获取新角度
		  mpuZreset(angle.AngleG,0);		
			vTaskDelay(500);
			buzzer_on();
			
		}
		
		else if((aim[0] == 0x02) && (aim[1] == 0x02))//右边 右边-->不动 回头
		{
			printf("到达2号收获点请取出物料");
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//发给c8t6表示到了
			print_mode =0;
			//执行开门操作
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//取出了
		}
	}
		
	
	
	
	
	
	
	else if(task_mode == 1)
	{
		if((aim[0] == 0x01) && (aim[1] == 0x01))//左边 左边-->不动 回头
		{
//		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
			printf("到达收获点请取出物料");
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//发给c8t6表示到了
			print_mode =0;
			//执行开门操作
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//取出了
		}
		
		else if((aim[0] == 0x01) && (aim[1] == 0x02))//左边 右边-->右走 回头
		{
		speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//小速度启动
			speed_clear();
//			F_Distance_pid_obj.target = aim_distance[2];
//			distance_model = 3;//超声波
//			pid_mode_switch(is_Distance);
//			//退出来保持
//			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
//			{
//				vTaskDelay(3);
//			}//后退结束
//			
//			speed_clear();
			
			//左走
		
			
			
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
    	vTaskDelay(500);//稳定启动
		
			x = Normal_speed;
		  y = 0;
			while(infrared.right)
			{
				vTaskDelay(3);
			}
			
			
      Zhuang(y_dir,z_speed,time_side+800);
			
			printf("到达2号收获点请取出物料");
			angle.AngleG = getAngleZ();  //撞平 获取新角度
		  mpuZreset(angle.AngleG,0);
			vTaskDelay(500);
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//发给c8t6表示到了
			print_mode =0;

			//执行开门操作
			CarBrake();
			time=0;
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//取出了
			time=0;

			//取出物料之后回家
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//小速度启动
			speed_clear();
			R_SPEED = 0;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[1]-3;
		
			distance_model = 3;//超声波
			pid_mode_switch(is_Distance);
			//退出来保持50cm的距离
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.5f))
			{
				vTaskDelay(2);
			}
			
			
			speed_clear();

			x = -small_z_speed;
			//angle.AngleG = getAngleZ();//上面赋值了
			pid_mode_switch(is_Gyro);
			vTaskDelay(500);
			x = -Normal_speed;
			
			vTaskDelay(1000);//右移

		
			

		  //激光定位	
			R_SPEED = 0;
			F_SPEED = 0;
			R_Distance_pid_obj.target = aim_distance[0];
			distance_model = 4;
			pid_mode_switch(is_Distance);
			//不退出来
			while((fabs(R_Distance_pid_obj.target - R_Distance_pid_obj.measure)>1.0f))
			{
				vTaskDelay(2);
			}
		
			//往前撞
      Zhuang(y_dir,100,1800);
			angle.AngleG = getAngleZ();  //撞平 获取新角度
		  mpuZreset(angle.AngleG,0);		
			vTaskDelay(500);
		}
		
		else if((aim[0] == 0x02) && (aim[1] == 0x01))//you->zuo
		{
			//后退
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//小速度启动
			speed_clear();
			speed_clear();
			F_Distance_pid_obj.target = aim_distance[1]-4;
			distance_model = 3;//超声波
			pid_mode_switch(is_Distance);
			//退出来保持50cm的距离
			buzzer_on();
//			CarBrake();
//			vTaskDelay(500);
			pid_mode_switch(is_Distance);
			while((fabs(F_Distance_pid_obj.target - distances.b_distance)>1.0f))
			{
				vTaskDelay(2);
			}//后退结束
			buzzer_off();

			
			speed_clear();

			x = -small_z_speed;
			//angle.AngleG = getAngleZ();//上面赋值了
			pid_mode_switch(is_Gyro);
			
			x = -Normal_speed;
		  vTaskDelay(1400);//右移
			
			
			
			R_SPEED = 0;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[2]+1;
			distance_model = 3;//超声波
			pid_mode_switch(is_Distance);
			
			//退出来一点
			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
			{
				vTaskDelay(2);
			}//后退结束
			
			speed_clear();

			x = -Normal_speed;
			pid_mode_switch(is_Gyro);
			
//			for(int i = 0; i<2 ; i++)
//			{
//					vTaskDelay(910);//右移
//			}
		  while(infrared.left)
			{
				vTaskDelay(3);
			}
			
      Zhuang(y_dir,z_speed,time_side);
			printf("到达1号收获点请取出物料");
			angle.AngleG = getAngleZ();  //撞平 获取新角度
		  mpuZreset(angle.AngleG,0);	
			vTaskDelay(500);
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//发给c8t6表示到了
			print_mode =0;
			
			//执行开门操作
			CarBrake();
			time=0;
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//取出了
				time=0;
			
			
			
			//取出物料之后回家
			speed_clear();
			Distance_pid_param.kp = 4;
			buzzer_on();
			x = 0;
			y = -40;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//小速度启动
			speed_clear();
			//纯后退
			speed_clear();
			F_Distance_pid_obj.target = aim_distance[2];
			distance_model = 3;//csb
			pid_mode_switch(is_Distance);
			while((fabs(F_Distance_pid_obj.target - F_Distance_pid_obj.measure)>1.0f))
			{
				vTaskDelay(2);

			}

			//右移
		  R_SPEED = Normal_speed;
			F_SPEED = 0;
			F_Distance_pid_obj.target = aim_distance[2];
			distance_model = 3;//
			pid_mode_switch(is_Distance);
			vTaskDelay(1400);

			//往后撞
      Zhuang(y_dir,-z_speed,600);
	
		  //激光定位
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
			
			
			//往前撞
      {
					F_SPEED = 0;
					R_SPEED = 0;				//速度都清零
					K=1;
					x = 0; y = 100;   //往墙角撞
					angle.AngleG = getAngleZ();
					pid_mode_switch(is_Gyro);
					vTaskDelay(1800);

					speed_clear();			//速度都清零
					CarBrake(); //停车
					K=0;
			}
			angle.AngleG = getAngleZ();  //撞平 获取新角度
		  mpuZreset(angle.AngleG,0);		
			vTaskDelay(500);
			buzzer_off();
			
		}
		
		else if((aim[0] == 0x02) && (aim[1] == 0x02))//右边 右边-->不动 回头
		{
			printf("到达2号收获点请取出物料");
			temp=0x02;
			print_mode =1;
			printf("%c",temp);//发给c8t6表示到了
			print_mode =0;
			//执行开门操作
			while(QR_information[2]!=0)
			{
				time++;
				if(time>=1000)
					break;
				vTaskDelay(2);
			}//取出了
		}
	
	
	
	
	}
		
		
		
		
	
buzzer_off();
		//统一回直走
		 	speed_clear();
			Distance_pid_param.kp = 4;
			x = 0;
			y = -30;   
			angle.AngleG = getAngleZ();
			pid_mode_switch(is_Gyro);
    	vTaskDelay(500);//小速度启动
			speed_clear();
			x = 0; y = -100;
			pid_mode_switch(is_Gyro);
			vTaskDelay(1400);
			x  = 0; y = 0;
			CarBrake(); //停车
			vTaskDelay(300);
			max_fliter=1;
			F_SPEED = -150;
			Distance_pid_param.kp = 6;
			R_Distance_pid_obj.target=aim_distance[0]+4;//保持激光原始距离
			distance_model = 4;//激光
			pid_mode_switch(is_Distance);
			vTaskDelay(500);
			
			F_SPEED = -250;
			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4))//后红外
			{
				vTaskDelay(2);
			}
//			while(infrared.right)//后红外
//			{
//				vTaskDelay(2);
//			}
			F_SPEED = -100;
			Distance_pid_param.kp = 4;
			R_Distance_pid_obj.target=aim_distance[0];//保持激光原始距离
			distance_model = 4;//激光
			pid_mode_switch(is_Distance);
			vTaskDelay(300);
			while(1)
			{
			
				CarBrake();
			}
	}



//主任务创建
void main_task_create(void){
	  xTaskCreate((TaskFunction_t ) main_task,//任务函数
	               (const char *)"main_task",	  //任务名字
								 (uint32_t) main_task_size,    //任务堆栈大小
								 (void* )NULL,                  //传递给任务参数的指针参数
								 (UBaseType_t) main_task_priority, //任务的优先级
								(TaskHandle_t *)&main_task_handler ); //任务句柄
							 }
