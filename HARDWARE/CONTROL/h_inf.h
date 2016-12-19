#ifndef _H_INF_H_
#define	_H_INF_H_

#include "../HARDWARE/define.h"
#include "../HARDWARE/HINF/W1_ert_rtw/W1.h"                        /* Model's header file */
#include "../HARDWARE/HINF/KSW2_ert_rtw/KSW2.h"                        /* Model's header file */
extern double ctrl_inf_att_out,ctrl_inf_att_out2;
float h_inf_att_inner(float set,float in,float max);//×ËÌ¬INNER
float h_inf_height_spd_out(float set,float in);
float h_inf_att_inner1(float set,float in,float max);
float h_inf_att_inner2(float set,float in,float max);
#endif

