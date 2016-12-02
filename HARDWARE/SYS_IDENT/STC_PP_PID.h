#ifndef _STC_PP_PID_h_
#define _STC_PP_PID_h_

#include "CFFRELS.h"  //需要用到渐消记忆最小二乘
#include "ControlFGR.h"  //极点配置的控制输出方式


void updateSTCFilterOnly(CFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]);

void updateSTCPPPID(CFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]);

float getControlSTCPPPID(CFFRELS_T* cffrelsIn, float CDataYr[], float CDataY[], float CDataU[]);




#endif
