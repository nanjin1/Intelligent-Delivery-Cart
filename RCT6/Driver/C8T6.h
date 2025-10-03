#ifndef __C8T6_h__
#define __C8T6_h__
#include "main.h"
#include "uart.h"
#include "usart.h"
#define BUFFER_SIZE  20
void C8T6_init(void);
extern uint8_t QR_information[4];
extern uint8_t OK;
extern uint8_t max_fliter_b;
extern uint8_t C8T6_rx_buf[BUFFER_SIZE];
#endif
