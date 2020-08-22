#ifndef _ATTITUDE_COUNT_H
#define _ATTITUDE_COUNT_H
#include "Includes.h"

float  Num_To_Dps(short int num);
void   AHRS_Update(Sensor_Data * Gyro,Sensor_Data *Acc,Sensor_Data*Mag,Angle *euler_angle);
void   Filter_Acc(Sensor_Data *Acc);
void   Filter_Gyro(Sensor_Data*Gyro);
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
double KalmanFilter(double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
float  VariableParameter(float error);
float  safe_asin(float v);

#endif

