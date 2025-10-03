#include "Start_task.h"
#include "encoder.h"
#include "Rudder_control.h"
#include "uart.h"
#include "delay.h"
#include  "motor.h"
#include "usmart.h"
#include "motor_task.h"
#include "buzzer.h"
#include "gyro.h"
#include "main_task.h"
#include "sin_generate.h"
#include "usmart_task.h"
#include "turn.h"
#include "scaner.h"
#include "voice.h"
#include "light.h"
#include "c8t6.h"
TaskHandle_t Start_handler ; //定义开始任务句柄

void Start_task(void *pvParameters){
			taskENTER_CRITICAL();           //进入临界区
	    #if sin_work == 1
			sin_task_create();
			#endif
			#if main_task_open == 1
			main_task_create();
			#endif
		  #if usmart_task_open==1
			usmart_task_create();
			#endif
			user_init();                 //用户初始化
			vTaskDelete(Start_handler); //删除开始任务
	  	taskEXIT_CRITICAL();            //退出临界区
}

/*****************************************************************************
函数名： GET_free_RAM
函数功能：获得任务的剩余堆栈大小 并且打印  
形参：该任务的句柄      若传回NULL，则为该任务的堆栈 
注意：INCLUDE_uxTaskGetStackHighWaterMark 1    才能使用
*******************************************************************************/
void GET_free_RAM(TaskHandle_t xTask){
//		printf("RAM = %d\r\n",(int32_t)uxTaskGetStackHighWaterMark(xTask));
		vTaskDelay(500);
}

/*****************************************************************************
函数名： user_init
函数功能：初始化外设，结构体等 
*******************************************************************************/
void user_init(void){	
	
	uart_init(115200);  		//初始化重定向串口
	usmart_dev.init(72);	  //初始化USMART	
	delay_init();        //初始化延时函数
	Rudder_Init(50);        	//舵机初始化 
	Encoder_init();     		//初始化编码器
	LED_init();							//初始化LED
	buzzer_init();					//初始化蜂鸣器
	gyro_init();			     //陀螺仪初始化
	motor_init();          //初始化轮子以及PID参数
	scaner_init();
	light_init();
	C8T6_init();
}

void Start_task_create(void){
	  xTaskCreate((TaskFunction_t ) Start_task,//任务函数
	               (const char *)"Start_task",	  //任务名字
								 (uint32_t) Start_size,    //任务堆栈大小
								 (void* )NULL,                  //传递给任务参数的指针参数
								 (UBaseType_t) Start_task_priority, //任务的优先级
								(TaskHandle_t *)&Start_handler ); //任务句柄
}

void LED_init(void){
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	 __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void LED_twinkle(void){
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}

void  GYRO_Start(void){
	float mpuZ_reset_val = 0;
	float temp_compensatePitch = 0;
	float temp_distance = 0;
	vTaskDelay(2000);
	voice_init();
	for (int i = 0; i<10; i++)
	{
		vTaskDelay(20);
		mpuZ_reset_val += imu.yaw;
		temp_compensatePitch += imu.pitch;
		temp_distance+=distances.l_distance;
	}
	mpuZ_reset_val /= 10;
	aim_distance[0] = temp_distance/10;
	
	mpuZreset(mpuZ_reset_val, 0);
	imu.compensatePitch = temp_compensatePitch/10;
}
