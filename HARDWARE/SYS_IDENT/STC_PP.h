#ifndef _STC_PP_h_
#define _STC_PP_h_

#include "PPCFFRELS.h"  //需要用到渐消记忆最小二乘
#include "ControlFGR.h"  //极点配置的控制输出方式


void updateSTCPPFilterOnly(PPCFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]);

void updateSTCPP(PPCFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]);

float getControlSTCPP(PPCFFRELS_T* cffrelsIn, float CDataYr[], float CDataY[], float CDataU[]);




#endif
