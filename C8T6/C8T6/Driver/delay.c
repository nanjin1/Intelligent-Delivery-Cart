#include "delay.h"
struct Time sys_time;
//延时函数+电机狗
TIM_HandleTypeDef TIM4_Handler;
void delay_init(void)
{
    //定时器7
    __HAL_RCC_TIM4_CLK_ENABLE();
     
    TIM4_Handler.Instance=TIM4;                          //通用定时器7
    TIM4_Handler.Init.Prescaler=72-1;                     //分频
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM4_Handler.Init.Period=1500-1;                        //自动装载值
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&TIM4_Handler);
	  HAL_NVIC_SetPriority(TIM4_IRQn,5,0);    //设置中断优先级，抢占优先级3，子优先级3
    HAL_NVIC_EnableIRQ(TIM4_IRQn);          //开启ITM4中断  
    HAL_TIM_Base_Start_IT(&TIM4_Handler); //使能定时器7和定时器7中断 
}

//1.0ms  
void TIM4_IRQHandler(void)
{
		static uint8_t times = 0;
    if(__HAL_TIM_GET_IT_SOURCE(&TIM4_Handler,TIM_IT_UPDATE)==SET){//溢出中断
			sys_time.times++;
			if(times%2==0){
						if(times>250){
							times = 0;
						}
				}
		}
    __HAL_TIM_CLEAR_IT(&TIM4_Handler, TIM_IT_UPDATE);//清除中断标志位
}

//最多延时1500us
void delay_us(uint16_t nus)
{
   TIM4->CNT = 0;
	 while(TIM4->CNT<nus);
}

void delay_ms(uint16_t nms)
{
	for(uint16_t i=0;i<nms;i++){
		delay_us(1000);
	}
}
