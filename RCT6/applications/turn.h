#ifndef __Turn_H
#define __Turn_H
#include "sys.h"
#include "gyro.h"

struct Angle {
	float AngleT;
	float AngleG;
	float AngleCG;
};
extern volatile struct Angle angle;
extern uint8_t K;
float need2turn(float nowangle,float targetangle);
void mpuZreset(float sensorangle ,float referangle);
float getAngleZ(void);
	
uint8_t Turn_Angle(float Angle);
uint8_t Stage_turn_Angle(float Angle);
void Turn_Angle_Relative(float Angle1);
uint8_t runWithAngle(float angle_want,float speed);
void AdCircle(float speed, float radius);
uint8_t Distance_run(float angle_want);
static inline float get_pitch(void)
{
	return imu.pitch;
}
extern uint8_t distance_model;
extern float F_SPEED,R_SPEED;
void Zhuang(int dir,int speed,int time);
extern uint8_t x_dir;
extern uint8_t y_dir;	
#endif
