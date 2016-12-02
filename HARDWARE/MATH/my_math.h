/***
*mymath.h
*
*       
*
*Purpose:
*       This file defined the functions and variables used by users
*       to fast computation the result of trigonometric functions and
*       the square root.
****/

#ifndef __MYMATH_H__
#define __MYMATH_H__
#include "../HARDWARE/define.h"
#include "../HARDWARE/parameter.h"
#define OFFSET_AV_NUM 	50					//校准偏移量时的平均次数。
#define FILTER_NUM 			10					//滑动平均滤波数值个数

#define MED_WIDTH_NUM 50					//中值滤波长度
#define MED_FIL_ITEM  30


#define REAL   float
#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256
#define MY_PPPIII   3.14159f
#define MY_PPPIII_HALF   1.570796f

#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))
#define LPF_1(A,B,C,D) LPF_1_((A),(B),(C),*(D));

void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
float Moving_Median(u8 item,u8 width_num,float in);
extern double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
extern fp32 LPF_1st(fp32 oldData, fp32 newData, fp32 lpf_factor);
void simple_3d_trans(xyz_f_t *ref, xyz_f_t *in, xyz_f_t *out);
double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double x_last,double p_last);

float my_abs(float f);
REAL fast_atan2(REAL y, REAL x);
float my_pow(float a);
float my_sqrt(float number);
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deathzoom(float x,float zoom);
float my_deathzoom_2(float x,float zoom);
float To_180_degrees(float x);
float my_pow_2_curve(float in,float a,float max);
float limit_mine(float x,float zoom);
float limit_mine2(float x,float min,float max);
float my_deathzoom_rc(float x,float zoom);


#endif

