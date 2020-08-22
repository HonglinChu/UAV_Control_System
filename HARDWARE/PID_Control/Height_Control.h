#ifndef _HEIGHT_CONTROL
#define _HEIGHT_CONTROL
#include "Includes.h"
void Height_Control(float T,float thr);
void Ultra_Out_Control(float T,float thr);
float my_deathzoom(float x,float zoom);
float my_deathzoom_2(float x,float zoom);
float Moving_Median(u8 item,u8 width_num,float in);
void Ultra_In_Control(float T,float thr,float exp_z_speed,float h_speed);
#endif
