#include "OLD_X_AVOID.h"
#include "avoid.h"
#include "filter.h"
#define NUM_AVOID 8
//激光雷达避障  未使用
double DIS_IN_TEST[NUM_AVOID] ;//= { 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
double DIS_IN[20];
double A = 0.2*100;
double A_dead = 0.001*1000;
int max_try = 200;	
u8 need_avoid;
float flt_avoid=0.05;

u16 TRC_DEAD=20;
u16 DIS_MAX=1000;
u16 DIS_MIN=600;	
u8 en_trace_test=0;
float avoid_trace[3];	
double x_mid = 0; 
double y_mid = 0;
double r_mid = 0;
void oldx_avoid(void)
{u8 i;
 if(!en_trace_test)	
	for(i=0;i<NUM_AVOID;i++)
	{
	if(DIS_IN[i]>DIS_MAX)
	DIS_IN_TEST[i]=DIS_MAX;
	else if(DIS_IN[i]<DIS_MIN)
	DIS_IN_TEST[i]=DIS_MIN;
	else 
	DIS_IN_TEST[i]=DIS_IN[i];
	}
	
	OLD_X_AVOID(DIS_IN_TEST, A, A_dead,max_try, &x_mid, &y_mid, &r_mid);
  avoid_trace[0]=my_deathzoom((int)x_mid,TRC_DEAD)*flt_avoid+(1-flt_avoid)*avoid_trace[0];
	avoid_trace[1]=my_deathzoom((int)y_mid,TRC_DEAD)*flt_avoid+(1-flt_avoid)*avoid_trace[1];
	avoid_trace[2]=my_deathzoom((int)r_mid,TRC_DEAD)*flt_avoid+(1-flt_avoid)*avoid_trace[2];
	
	if((fabs(avoid_trace[0])>TRC_DEAD||fabs(avoid_trace[1])>TRC_DEAD)&&mode.en_sonar_avoid)
		need_avoid=1;
	else
		need_avoid=0; 
}  
