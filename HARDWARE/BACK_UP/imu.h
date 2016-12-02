#ifndef _IMU_H_
#define	_IMU_H_

#include "stm32f4xx.h"

#include "parameter.h"
#include "my_math.h"
#include "math.h"

typedef struct 
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;
	
}ref_t;

extern xyz_f_t reference_v;
extern float reference_vr[3],reference_vr_fc[3];
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw);
extern float Roll,Pitch,Yaw,yaw_mag_view[4];
extern float Roll_mid_down,Pitch_mid_down,Yaw_mid_down;    				//вкл╛╫г
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) ;
extern float ref_q[4] , q_nav[4];
extern float ref_q_imd_down[4];
extern float reference_vr_imd_down[3];
#endif

