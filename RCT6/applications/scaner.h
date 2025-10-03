#ifndef __SCANER_H
#define __SCANER_H
#include "sys.h"
#include "pid.h"

typedef struct scaner	
{
	volatile uint16_t detail;  //�����Ƶ�����
	volatile float error;			//���
	volatile u8 ledNum;				//�Ƶ�����
	volatile u8 lineNum;      //linenum������¼�ж�����������
}SCANER;

#define LFB_SENSOR_NUM 8   //ѭ���崫��������
struct Scaner_Set {
	float CatchsensorNum;   //Ŀ��λ��
	int8_t EdgeIgnore;			//���Ե�
};
void scaner_init(void);
extern struct Scaner_Set scaner_set;
extern uint8_t L_R_open;  //���ݿ�������ѭ��   1�� 2�� 
extern SCANER Scaner;
extern const float line_weight[8];

void Go_Line(float speed);
uint8_t Line_Scan(SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore);
#endif
