#ifndef light_H
#define light_H
#include "main.h"
#include "usart.h"
struct Distance{
	float l_distance;
	float b_distance;
};
extern struct Distance distances;
void light_init(void);
extern uint8_t max_fliter;
#endif
