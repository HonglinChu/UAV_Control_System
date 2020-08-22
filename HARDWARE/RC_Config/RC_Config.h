#ifndef _RC_CONFIG_H
#define _RC_CONFIG_H
#include "Includes.h"

void QUAD_Armed(u32 time);
void RC_Adjust(void);
u16  THR_Target(void);
float ROLL_Target(void); 
float PITCH_Target(void);
u16   YAW_Target(void);
float Range_Limit(float Pre,float MAX,float MIN);

#endif


