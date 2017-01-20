#ifndef _alt_ukf_h
#define _alt_ukf_h

#include "../HARDWARE/define.h"
#include "../HARDWARE/MATH/arm_math_m.h"

extern float ALT_POS_BMP_EKF,ALT_VEL_BMP_EKF;
extern float ALT_POS_BMP,ALT_VEL_BMP,accz_bmp;
extern float ALT_POS_SONAR,ALT_VEL_SONAR,ALT_POS_SONAR2,ALT_POS_SONAR3,ALT_POS_BMP_UKF_OLDX,ALT_VEL_BMP_UKF_OLDX;;
extern double X_ukf_baro[3];
extern float X_apo_height[2],x_tst[2],acc_bmp;
void ukf_baro_task1(float T);
extern float acc_body[3];
extern float acc_est;
#endif
