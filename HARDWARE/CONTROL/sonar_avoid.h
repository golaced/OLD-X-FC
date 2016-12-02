#ifndef _SONAR_AVOID_H
#define _SONAR_AVOID_H

#include "../HARDWARE/define.h"
extern int sonar_avoid[8];
extern int sonar_avoid_c[2];
extern float angle_sonar[2];
extern double DIS_IN[20];
void oldx_avoid(void);
extern float avoid_trace[3];	
extern u8 need_avoid;
#endif
