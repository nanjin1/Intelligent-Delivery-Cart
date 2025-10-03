#include "c8t6.h"
#include "light.h"
#include <string.h>
#include "math.h"
#include "main_task.h"
#define CT86_UART UART2_Handler
#define BUFFER_SIZE  20
uint8_t QR_information[4]={0,0,0,0};
uint8_t C8T6_rx_buf[BUFFER_SIZE] = {0};
uint8_t C8T6_rx_len = 0;
uint8_t OK = 0;
uint8_t max_fliter_b = 0;
void C8T6_init(void){
	__HAL_UART_ENABLE_IT(&CT86_UART, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&CT86_UART,C8T6_rx_buf,BUFFER_SIZE);
}

//激光接收中断
void USART2_IRQHandler(){
	uint32_t flag_idle = 0;
  static float last_distacne = 0.0f;
	flag_idle = __HAL_UART_GET_FLAG(&CT86_UART,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&CT86_UART);
		HAL_UART_DMAStop(&CT86_UART); 
		
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);   
		C8T6_rx_len = BUFFER_SIZE - temp; 
		
		if(C8T6_rx_buf[0] == 0x55)
		{ //二维码讯号
				QR_information[0] = C8T6_rx_buf[1];
				QR_information[1] = C8T6_rx_buf[2];
				QR_information[2] = C8T6_rx_buf[3];
				QR_information[3] = C8T6_rx_buf[4];
		}
		
		
	  else if(C8T6_rx_buf[0] == 0xFF)
	  {   //超声波
			OK = C8T6_rx_buf[4];
			uint8_t sum = 0;
			for(uint8_t i=0;i<3;i++){
				sum+=C8T6_rx_buf[i];
			}
			if(sum==C8T6_rx_buf[3])
			{ //校验正确
				uint16_t data = ((C8T6_rx_buf[1]<<8) + C8T6_rx_buf[2]);
				if(max_fliter_b)
				{
						if(fabs((((float)data)/10)-last_distacne)<8)
						{
								distances.b_distance = ((float)(data))/10.0f;			
								last_distacne = distances.b_distance;							
						}
				}
				else
				{
					distances.b_distance = ((float)(data))/10.0f;
					last_distacne = distances.b_distance;
				}

			}
		}
		else if(C8T6_rx_buf[0] == 0x55)
		{ //二维码讯号
				QR_information[0] = C8T6_rx_buf[1];
				QR_information[1] = C8T6_rx_buf[2];
				QR_information[2] = C8T6_rx_buf[3];
				QR_information[3] = C8T6_rx_buf[4];
		}
//		else if(C8T6_rx_buf[0] == 0x08)
//		{ 
//	    	OK	= C8T6_rx_buf[1];
//		}
		C8T6_rx_len = 0;
		memset(C8T6_rx_buf,0,C8T6_rx_len);
	}
	HAL_UART_Receive_DMA(&CT86_UART,C8T6_rx_buf,BUFFER_SIZE);
	HAL_UART_IRQHandler(&CT86_UART); 	
}
