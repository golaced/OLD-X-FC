/*********************************************************************************
*                                山猫飞控（Lynx）
*                             for LynxFly under GPLv2
*
* COPYRIGHT (C) 2012 - 2013, Lynx 84693469@qq.com
*
* Version   	: V1.0
* By        	: Lynx@sia 84693469@qq.com
*
* For       	: Stm32f405RGT6
* Mode      	: Thumb2
* Description   : 极点配置控制器输出封装
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include "ControlFGR.h"


//一些神奇的数学定义
#define NAN	 (0.0/0.0)	 //无效值NaN
#define isNAN(x)	 ((x)!=(x))	
#define INF	 (1.0/0.0)	 //无穷大
#define PINF	 INF	 //正无穷大
#define NINF	 -INF	 //负无穷大
#define isINF(x)	 (((x)==PINF)||((x)==NINF))
#define isPINF(x)	 ((x)==PINF)
#define isNINF(x)	 ((x)==NINF)


/*
 * Name										: ControlFGR
 * Description						: 根据F G R三个系统多项式函数输出控制量，F为控制器极点多项式，R为跟踪输入零点多项式，G为输出反馈零点多项式
 * Entry                  : （0为当前时刻）跟踪目标序列，被控系统输出序列，被控系统输入序列，首一的F序列，F序列长度，G序列，G序列长度，R序列，R序列长度
 * Return                 : 控制量
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/15/2013
 *
 * create.
 * ----------------------
 */
float ControlFGR(float CDataYr[], float CDataY[], float CDataU[], float F[], int lengthF, float G[], int lengthG, float R[], int lengthR) {
	
	int i = 0;  //你懂得
	
	float CSF = 0;  //三个求和的临时变量
	float CSG = 0;
	float CSR = 0;
	
	float tmpOut = 0;
	
	CSF = 0;
	for(i=1;i<lengthF;i++){  //注意是从1开始的 第一项不在这里乘
		CSF -= F[i]*CDataU[i];  //注意负号，注意起始位置
	}
	
	CSR = 0;
	for(i=0;i<lengthR;i++){  // 以后要考虑提前反应的话这里就得想办法改改了
		CSR += R[i]*CDataYr[i];  
	}
	
	CSG = 0;
	for(i=0;i<lengthG;i++){  //注意是从1开始的
		CSG -= G[i]*CDataY[i];  //注意负号
	}
	
	//return (CSF+CSG+CSR)/F[0];  //返回该通道的值 因为会产生非数而被遗弃
	tmpOut = CSF+CSG+CSR;
	if(isNAN(tmpOut)){
		tmpOut = 0;  //给一些出错提示，如果可以的话
	}
	return tmpOut;  //返回该通道的值
	
}

