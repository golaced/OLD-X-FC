#ifndef _ESO_H_
#define	_ESO_H_

#include "stm32f4xx.h"

typedef struct
{ float eso_dead;
  float beta0,beta1,beta2,beta3;
	float disturb,disturb_u,disturb_u_reg;
	float alfa1,alfa2,alfa0,tao,KP,KD,KI,e;
	float z[3];
	float h,integer;
	u8 init,level;
	u8 n,out_mode,use_td;
  u8 not_use_px4;
	u8 auto_b0;
	float v1,v2,h0,r0,b0,b01,h1,r1,c,u;
}ESO;

extern ESO eso_att_outter[4],eso_att_inner[4];
extern ESO eso_att_outter_c[4],eso_att_inner_c[4],ESO_BMP;
float fst(float x1,float x2,float w,float h);
float fal(float e,float alfa,float delta);
float fst2(float x1,float x2,float w, float h);
float sign(float x);
void ESO_BMP_INIT(ESO *eso_in);
void SMOOTH_IN_ESO(ESO *eso_in,float in);
float AUTO_B0(ESO *eso_in,float v,float y,float u,float T,float MAX);
float ESO_3N(ESO *eso_in,float v,float y,float u,float T,float MAX);
float ESO_2N(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4);
float ATT_CONTRL_OUTER_ESO_3(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4);
float ATT_CONTRL_INNER_ESO_3(ESO *eso_in,float v,float y,float u,float T,float MAX);
float ATT_CONTRL_INNER_ESO_3_Y(ESO *eso_in,float v,float y,float u,float T,float MAX);
float HIGH_CONTROL_SPD_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX);
#endif

