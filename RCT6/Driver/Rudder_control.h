#ifndef __Rudder_control_H__
#define __Rudder_control_H__
#include "iic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
void Rudder_Init(uint16_t hz);

#define Rudder 0x70
#define PRE_SCALE 0xFE

#define LED0_ON_L  	0x06
#define LED0_ON_H  	0x07
#define LED0_OFF_L  0x08
#define LED0_OFF_H  0x09

void Rudder_control(uint16_t aim,uint8_t id);
void Rudder_WriteOneByte(uint8_t WriteAddr,uint8_t DataToWrite);
uint8_t Rudder_ReadOneByte(uint8_t ReadAddr);
uint8_t get_freq(uint16_t hz);
#endif
