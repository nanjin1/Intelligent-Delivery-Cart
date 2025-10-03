#ifndef __SCANER_H
#define __SCANER_H
#include "sys.h"
#include "pid.h"

typedef struct scaner	
{
	volatile uint16_t detail;  //二进制灯数据
	volatile float error;			//误差
	volatile u8 ledNum;				//灯的数量
	volatile u8 lineNum;      //linenum用来记录有多少条引导线
}SCANER;

#define LFB_SENSOR_NUM 8   //循迹板传感器数量
struct Scaner_Set {
	float CatchsensorNum;   //目标位置
	int8_t EdgeIgnore;			//忽略灯
};
void scaner_init(void);
extern struct Scaner_Set scaner_set;
extern uint8_t L_R_open;  //短暂控制左右循迹   1左 2右 
extern SCANER Scaner;
extern const float line_weight[8];

void Go_Line(float speed);
uint8_t Line_Scan(SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore);
#endif
