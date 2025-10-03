#include "delay.h"
struct Time sys_time;
//��ʱ����+�����
TIM_HandleTypeDef TIM4_Handler;
void delay_init(void)
{
    //��ʱ��7
    __HAL_RCC_TIM4_CLK_ENABLE();
     
    TIM4_Handler.Instance=TIM4;                          //ͨ�ö�ʱ��7
    TIM4_Handler.Init.Prescaler=72-1;                     //��Ƶ
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM4_Handler.Init.Period=1500-1;                        //�Զ�װ��ֵ
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&TIM4_Handler);
	  HAL_NVIC_SetPriority(TIM4_IRQn,5,0);    //�����ж����ȼ�����ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(TIM4_IRQn);          //����ITM4�ж�  
    HAL_TIM_Base_Start_IT(&TIM4_Handler); //ʹ�ܶ�ʱ��7�Ͷ�ʱ��7�ж� 
}

//1.0ms  
void TIM4_IRQHandler(void)
{
		static uint8_t times = 0;
    if(__HAL_TIM_GET_IT_SOURCE(&TIM4_Handler,TIM_IT_UPDATE)==SET){//����ж�
			sys_time.times++;
			if(times%2==0){
						if(times>250){
							times = 0;
						}
				}
		}
    __HAL_TIM_CLEAR_IT(&TIM4_Handler, TIM_IT_UPDATE);//����жϱ�־λ
}

//�����ʱ1500us
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
