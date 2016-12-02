#ifndef _H_INF_H_
#define	_H_INF_H_

#include "../HARDWARE/define.h"
extern double ctrl_inf_att_out,ctrl_inf_att_out2;
extern void h_inf_att_out(float set,float in);
float h_inf_height_spd_out(float set,float in);

#endif

