#ifndef __FOCDRIVER_H
#define __FOCDRIVER_H
#include "main.h"
// 外部变量声明
extern float Voltage_Power_Supply;
extern float Zero_Electric_Angle;
extern float shaft_angle;
extern float open_loop_timestamp;
extern float Ualpha, Ubeta, Ua, Ub, Uc, dc_a, dc_b, dc_c;


//宏定义一个取范围函数

#define _constrain(amt,low,high) ((amt)<(low)?(low):(amt>(high)?(high):(amt)))

// 函数声明
float Electrical_Angle(int direction, int pole_pairs);
float Angle_Normalize(float angle);
void SetPWM(float Ua, float Ub, float Uc);
void SetphaseVoltage(float Uq, float Angle_el);
float velocityOpenloop(float target_velocity);


#endif
