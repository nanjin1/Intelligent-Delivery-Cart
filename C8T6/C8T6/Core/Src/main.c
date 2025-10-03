/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD.h"
#include "pic.h"
#include <string.h>
#include "delay.h"
#include <stdio.h>
#include "QR.h"
#include "Rudder_control.h"
#include "iic.h"
#define one 0x31
#define two 0x32
/* USER CODE END Includes */
typedef struct{
	TIM_HandleTypeDef TIM;
	GPIO_TypeDef* GPIO_2;//B相
  uint32_t GPIO_PORT_2;		
	uint32_t Channel;			//A相
	uint32_t ActiveChannel;
}wheel;

wheel wheel_1;

uint8_t capture_Cnt[4] = {0};
uint32_t capture_Buf_before[4] = {0},capture_Buf_latter[4] = {0};   //存放计数时间
uint32_t temp_time[4][8] = {0};        			//高电平暂存时间
volatile uint32_t high_time[4] = {0};        			//高电平时间
uint32_t real_time[4] = {0};    					//CCR真实计时
uint8_t frist_flag[4]={0};								  //第一次收集八个数据的标志
uint8_t temp_i[4]={0};
float bo_distance = 0;
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
 {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	delay_init();
	QR_init();
	LCD_Init();//LED初始化
	LCD_Fill(0,0,160,128,WHITE);
	LCD_ShowPicture(8,0,143,128,gImage_1);
	Rudder_Init(50);
//	Rudder_control(480,0);//1号380开 480关
//	Rudder_control(250,15);//2号350开 250关

//LCD_ShowPicture(8,0,142,128,gImage_2);
  /* USER CODE END 2 */
	wheel_1.TIM = htim1;  
	wheel_1.Channel = TIM_CHANNEL_4;
	wheel_1.ActiveChannel = HAL_TIM_ACTIVE_CHANNEL_4;
	HAL_TIM_IC_Start_IT(&(wheel_1.TIM), wheel_1.Channel);	  //启动输入捕获
	__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_1.TIM, wheel_1.Channel, TIM_INPUTCHANNELPOLARITY_RISING);  //开启上升沿捕获
	
	for(uint8_t i =0;i<8;i++){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
		delay_us(20);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
		HAL_Delay(60);
	}
	bo_distance = (((float)high_time[0])*170.0f);
	uint8_t BO_DATA[4] = {0xFF,0X0,0X0,0X0};
	uint8_t QR_DATA[4] = {0X55,0X0,0X0,0x0};
    Rudder_control(480,0);
	Rudder_control(250,15);
//	Rudder_control(350,0);
//	Rudder_control(400,15);
	uint8_t time=0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
	   
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
		delay_us(20);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
		bo_distance = (((float)high_time[0])*0.17f);  //单位毫米
		BO_DATA[1] = (uint16_t)bo_distance/256;
		BO_DATA[2] = (uint16_t)bo_distance%256;
		BO_DATA[3] = (uint8_t)(BO_DATA[0]+ BO_DATA[1]+BO_DATA[2]);  
		HAL_UART_Transmit(&huart1,BO_DATA,4,0xffff);
		HAL_Delay(30);
		if(QR_flag==1){
			QR_flag = 0;
			if((QR_information[0]==one||QR_information[0]==two)&&(QR_information[1]==one||QR_information[1]==two)){
				if(QR_information[0]==one){
					QR_DATA[1] = 0X1;
				}
				else {
					QR_DATA[1] = 0X2;
				}
			
				if(QR_information[1]==one){
					QR_DATA[2] = 0X1;
				}
				else {
					QR_DATA[2] = 0X2;
				}
				QR_DATA[3] = (uint8_t)(QR_DATA[0]+QR_DATA[1]+QR_DATA[2]);
				while((USART1->SR&0X40)==0);//等待超声波发送完成
				HAL_UART_Transmit(&huart1,QR_DATA,4,0xffff);
		  }
		}
		HAL_Delay(30);
	   if(time==0)//出发时
	   {
		   if(QR_information[0]==0x31)
		   {
			   Rudder_control(350,0);//开1号门
			   
		   }
		   else if(QR_information[0]==0x32)
		   {
				Rudder_control(400,15);//开2号门
		   } 
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1))
			{
				HAL_Delay(500);
				Rudder_control(480,0);//关
				QR_information[0]=0;
				QR_DATA[1]=0;
				QR_DATA[3] = (uint8_t)(QR_DATA[0]+QR_DATA[1]+QR_DATA[2]);
				HAL_Delay(500);//等待物料放进去
				for(int i=0;i<3;i++)//多次发送避免收不到
				{
					while((USART1->SR&0X40)==0);//等待超声波发送完成
					HAL_UART_Transmit(&huart1,QR_DATA,4,0xffff);
				}
				time=1;
			}	
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0))
			{
				HAL_Delay(500);
				Rudder_control(250,15);//关
				QR_DATA[1]=0;
				QR_information[0]=0;
				QR_DATA[3] = (uint8_t)(QR_DATA[0]+QR_DATA[1]+QR_DATA[2]);
				HAL_Delay(500);
				for(int i=0;i<3;i++)
				{
					while((USART1->SR&0X40)==0);//等待超声波发送完成
					HAL_UART_Transmit(&huart1,QR_DATA,4,0xffff);
				}
				time=1;
			}
			
	   }
	    if(time==1&QR_information[0]!=0)//收货区扫到二维码
		{
			if(QR_information[0]==0x31)
			{
				QR_DATA[1] = 0X1;
				QR_DATA[3] = (uint8_t)(QR_DATA[0]+QR_DATA[1]+QR_DATA[2]);
				for(int i=0;i<10;i++)
				{
					while((USART1->SR&0X40)==0);
					HAL_UART_Transmit(&huart1,QR_DATA,4,0xffff);
				}
				Rudder_control(350,0);//开1号门
				while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1))
				{
					QR_information[0]=0;
					QR_DATA[1]=0;
					QR_DATA[3] = (uint8_t)(QR_DATA[0]+QR_DATA[1]+QR_DATA[2]);
				}//拿出物料
					
				HAL_Delay(500);
				 Rudder_control(480,0);//拿出物料关门
				 
				for(int i=0;i<3;i++)
				{
					while((USART1->SR&0X40)==0);
					HAL_UART_Transmit(&huart1,QR_DATA,4,0xffff);
				}
			}
			else if(QR_information[0]==0X32)
			{
				QR_DATA[1] = 0X1;
				QR_DATA[3] = (uint8_t)(QR_DATA[0]+QR_DATA[1]+QR_DATA[2]);
				for(int i=0;i<10;i++)
				{
					while((USART1->SR&0X40)==0);//等待超声波发送完成
					HAL_UART_Transmit(&huart1,QR_DATA,4,0xffff);
				}
				Rudder_control(400,15);//开2号门
				while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0))
				{
				   
					QR_information[0]=0;
					QR_DATA[1]=0;
					QR_DATA[3] = (uint8_t)(QR_DATA[0]+QR_DATA[1]+QR_DATA[2]);
					
				}
				 HAL_Delay(500);
				 Rudder_control(250,15);//拿出物料关门
				
				for(int i=0;i<3;i++)
				{
					while((USART1->SR&0X40)==0);//等待超声波发送完成
					HAL_UART_Transmit(&huart1,QR_DATA,4,0xffff);
				}
			}
		}	
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//左上轮
	if(htim->Channel == wheel_1.ActiveChannel)
	{
		switch(capture_Cnt[0]){
			
			case 0:   //模式1   --------   捕获到上升沿 开始计时
				capture_Buf_before[0] = HAL_TIM_ReadCapturedValue(&(wheel_1.TIM),wheel_1.Channel);//获取当前的捕获值
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_1.TIM,wheel_1.Channel,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
				capture_Cnt[0]++; //从模式一切换到模式二
				break;
			
			  case 1:   //  模式二  --------   捕获到下降沿 关闭计时 
				capture_Buf_latter[0] = HAL_TIM_ReadCapturedValue(&(wheel_1.TIM),wheel_1.Channel);//获取当前的捕获值
				HAL_TIM_IC_Stop_IT(&wheel_1.TIM,wheel_1.Channel); //停止捕获  或者: __HAL_TIM_DISABLE(&htim5);
				if(capture_Buf_latter[0]<capture_Buf_before[0]){       //溢出计算
				    real_time[0] = (65535-capture_Buf_before[0])+capture_Buf_latter[0] +1;
			   }
			 else{
						real_time[0] =  capture_Buf_latter[0]- capture_Buf_before[0] ;  //没有溢出
			  }
					if(!frist_flag[0]){    //前八次要先收集起来
						temp_time[0][temp_i[0]] = real_time[0];
						temp_i[0]++;
						if(temp_i[0]==8){
							for(uint8_t i =0; i<8;i++){
								real_time[0]+= temp_time[0][i];
							}
							high_time[0] = real_time[0]/8;
							frist_flag[0]=1;
						}
					}
					else{
						high_time[0] = 0;
						memmove(&temp_time[0], &temp_time[0][1], sizeof(uint32_t) * 7);
						temp_time[0][7] = real_time[0];
						for(uint8_t i =0; i<8;i++){
								real_time[0]+= temp_time[0][i];	
							}
						high_time[0] = real_time[0]/8;
					}
			  capture_Cnt[0] = 0;  //清空标志
					HAL_TIM_IC_Start_IT(&(wheel_1.TIM), wheel_1.Channel);	  //启动输入捕获
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_1.TIM, wheel_1.Channel, TIM_INPUTCHANNELPOLARITY_RISING);  //开启上升沿捕获
			  break; 
		 }
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
