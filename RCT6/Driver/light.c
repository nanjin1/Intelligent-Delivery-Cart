#include "light.h"
#include "string.h"
#include "math.h"
uint8_t max_fliter = 0;
#define light huart3
#define BUFFER_SIZE  20
struct Distance distances = {0,0};
uint8_t light_rx_buf[BUFFER_SIZE] = {0};
uint8_t light_rx_len = 0;

void light_init(void)
{	
	__HAL_UART_ENABLE_IT(&light, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&light,light_rx_buf,BUFFER_SIZE);
}

//激光接收中断
void USART3_IRQHandler(){
	uint32_t flag_idle = 0;
	static float last_distacne = 0.0f;
	flag_idle = __HAL_UART_GET_FLAG(&light,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&light);
		HAL_UART_DMAStop(&light); 
		
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);   
		light_rx_len = BUFFER_SIZE - temp; 
		
		if(light_rx_buf[0] == 0x57){
			uint8_t sum = 0;
			for (int i=0; i<15; i++)
				sum += light_rx_buf[i];
			if (sum == light_rx_buf[15]){					
					uint32_t data = ((light_rx_buf[8])+(light_rx_buf[9]<<8)+(light_rx_buf[10]<<16));
					if(max_fliter){
						if(fabs((((float)data)/10)-last_distacne)<7){
								distances.l_distance = ((float)data)/10;			
								last_distacne = distances.l_distance;							
						}
					}
					else{
							 distances.l_distance = ((float)data)/10;
							 last_distacne = distances.l_distance;
					}
			}
		}
		light_rx_len = 0;
		memset(light_rx_buf,0,light_rx_len);
	}
	HAL_UART_Receive_DMA(&light,light_rx_buf,BUFFER_SIZE);
	HAL_UART_IRQHandler(&light); 	
}
