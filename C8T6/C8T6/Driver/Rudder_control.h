#ifndef __Rudder_control_H__
#define __Rudder_control_H__
#include "iic.h"
#include "main.h"

void Rudder_Init(uint16_t hz);

#define Rudder 0x70
#define PRE_SCALE 0xFE

#define LED0_ON_L  	0x06
#define LED0_ON_H  	0x07
#define LED0_OFF_L  0x08
#define LED0_OFF_H  0x09

#define LED1_ON_L  	0x0A
#define LED1_ON_H  	0x0B
#define LED1_OFF_L  0X0C
#define LED1_OFF_H  0X0D

#define LED2_ON_L   0x0E
#define LED2_ON_H   0x0F
#define LED2_OFF_L  0X10
#define LED2_OFF_H  0X11

#define LED3_ON_L   0x12
#define LED3_ON_H   0x13
#define LED3_OFF_L  0X14
#define LED3_OFF_H  0X15

#define LED4_ON_L   0x16
#define LED4_ON_H   0x17
#define LED4_OFF_L  0X18
#define LED4_OFF_H  0X19

#define LED10_ON_L   0x2E
#define LED10_ON_H   0x2F
#define LED10_OFF_L  0X30
#define LED10_OFF_H  0X31
void Rudder_control(uint16_t aim,uint8_t id);
void Rudder_WriteOneByte(uint8_t WriteAddr,uint8_t DataToWrite);
uint8_t Rudder_ReadOneByte(uint8_t ReadAddr);
uint8_t get_freq(uint16_t hz);
#endif
