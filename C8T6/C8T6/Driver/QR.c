#include "QR.h"
#include <string.h>
#define QR_UART huart2
#define BUFFER_SIZE  20

uint8_t QR_rx_buf[BUFFER_SIZE] = {0};
uint8_t QR_rx_len = 0;
uint8_t QR_information[2] = {0,0};
uint8_t QR_flag = 0;
void QR_init(void){
//	print_mode = 1;
//	printf("$202004-HEAD-7591");
//  printf("$201001-EE7F");
//	print_mode = 0;
	__HAL_UART_ENABLE_IT(&QR_UART, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&QR_UART,QR_rx_buf,BUFFER_SIZE);
}

//二维码接收中断
void USART2_IRQHandler(){
	uint32_t flag_idle = 0;
	
	flag_idle = __HAL_UART_GET_FLAG(&QR_UART,UART_FLAG_IDLE); 
//	if((flag_idle != RESET))
//	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&QR_UART);
		HAL_UART_DMAStop(&QR_UART); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);   
		QR_rx_len = BUFFER_SIZE - temp; 
		
		if(QR_rx_buf[0] == 0x48 &&QR_rx_buf[1] == 0x45&&QR_rx_buf[2] == 0x41&&QR_rx_buf[3] == 0x44&&QR_rx_buf[6] == 0x54){   //二维码
			QR_information[0]=  QR_rx_buf[4];
			QR_information[1]=  QR_rx_buf[5];
			QR_flag = 1;
		}
//		HAL_UART_Transmit(&huart1,QR_rx_buf,QR_rx_len,0XFFFF);
		QR_rx_len = 0;
		memset(QR_rx_buf,0,QR_rx_len);
//	}
	HAL_UART_Receive_DMA(&QR_UART,QR_rx_buf,BUFFER_SIZE);
	HAL_UART_IRQHandler(&QR_UART); 	
}
