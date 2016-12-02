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
* Description   : 直接法自校正极点配置PID控制器
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include "STC_PP_PID.h"


//定义存储的控制用数据的长度（历史和当前，当前的索引为0），这个值一定得大于2，因为默认U有一阶延迟
#define I_CONTROL_DATA_LENGTH 10  

//定义多组输入输出数据的存储
// float CDataUfk[I_CONTROL_DATA_LENGTH];   //注意这里与matlab程序顺序的差异，结尾带k的都是从上一时刻开始的数据，既 第一个 数yk(1)对应这里 第二个 数既yk[1]
// float CDataYfk[I_CONTROL_DATA_LENGTH];   //注意这里与matlab程序顺序的差异，结尾带k的都是从上一时刻开始的数据，既 第一个 数yk(1)对应这里 第二个 数既yk[1]

// float RELS_tmp_V1[CFFRELS_ML_A][1];  //用于矩阵运算的临时变量 向量
// float RELS_tmp_VT1[1][CFFRELS_ML_A];  //用于矩阵运算的临时变量 转置向量
// float RELS_tmp_UM1[1][1];  //用于矩阵运算的临时变量 单位矩阵



/*
 * Name										: updateSTCFilterOnly
 * Description						: 直接法极点配置自校正PID型控制器 辨识滤波器数据的更新
 * Entry                  : cffrelsIn 直接法的观测器结构体，（0为当前时刻）滤波后的输出序列，滤波后的输入序列，被控系统输出序列，被控系统输入序列
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
void updateSTCFilterOnly(CFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]){
	//几个记录中间值的临时变量
	float tmpFSumAAuf = 0;  //-AA(2:naa+1)*ufk(d+1:d+naa)
	float tmpFSumAAyf = 0;  //-AA(2:naa+1)*yfk(d+1:d+naa)
	float tmpFSumFu = 0;    //deltaF*uk(d:d+ndf)
	float tmpFSumY = 0;     //yk(d)
	
	int i = 0;   //你懂得
	
	//-AA(2:naa+1)*ufk(d+1:d+naa)
	tmpFSumAAuf = 0;
	for(i=0;i<cffrelsIn->NAA;i++){
		tmpFSumAAuf -= cffrelsIn->AA[0][i+1]*CDataUFK[cffrelsIn->D+i+1]; //注意负号 //NND不加1  不纠结了 要杀要剐随便了
	}
	//deltaF*uk(d:d+ndf)
	tmpFSumFu = 0;
	tmpFSumFu += 1*CDataU[cffrelsIn->D];  //NND不加1  不纠结了 要杀要剐随便了
	tmpFSumFu += -1*CDataU[cffrelsIn->D+1];
	
	//-AA(2:naa+1)*yfk(d+1:d+naa)
	tmpFSumAAyf = 0;
	for(i=0;i<cffrelsIn->NAA;i++){
		tmpFSumAAyf -= cffrelsIn->AA[0][i+1]*CDataYFK[cffrelsIn->D+i+1]; //注意负号 //NND不加1  不纠结了 要杀要剐随便了
	}
	//yk(d)
	tmpFSumY = CDataYE[cffrelsIn->D];   //yk(1)对应YE[1]
	
	CDataUFK[cffrelsIn->D] = tmpFSumAAuf+tmpFSumFu;  //ufk(d)=-AA(2:naa+1)*ufk(d+1:d+naa)+deltaF*uk(d:d+ndf); %滤波输入输出 这里已经考虑△
	CDataYFK[cffrelsIn->D] = tmpFSumAAyf+tmpFSumY;   //yfk(d)=-AA(2:naa+1)*yfk(d+1:d+naa)+yk(d);
}




/*
 * Name										: updateSTCPPPID
 * Description						: 直接法极点配置自校正PID型控制器 辨识滤波器数据的更新并观测得到控制器参数
 * Entry                  : cffrelsIn 直接法的观测器结构体，（0为当前时刻）滤波后的输出序列，滤波后的输入序列，被控系统输入序列
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
void updateSTCPPPID(CFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]){
	//先更新滤波器
	updateSTCFilterOnly(cffrelsIn, CDataYFK, CDataUFK, CDataYE, CDataU);
	//进行观测
	CFFRELS_Update(cffrelsIn, CDataYFK, CDataUFK, 0.995);   //这里取λ=1
}




/*
 * Name										: getControlSTCPPPID
 * Description						: 直接法极点配置自校正PID型控制器 获取控制器输出的控制量
 * Entry                  : cffrelsIn 直接法的观测器结构体，（0为当前时刻）跟踪目标序列，被控系统输出序列，被控系统输入序列
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
 * Rev										: 0.00
 * Date										: 06/18/2013
 *
 * cffrelsIn->NR        ->         cffrelsIn->NR+1
 * ----------------------
 */
float getControlSTCPPPID(CFFRELS_T* cffrelsIn, float CDataYr[], float CDataY[], float CDataU[]) {
	
	float tmpOut = 0;
	
	tmpOut = ControlFGR(CDataYr, CDataY, CDataU, (float*)cffrelsIn->FE, cffrelsIn->NF+1, (float*)cffrelsIn->GE, cffrelsIn->NG+1, (float*)cffrelsIn->R, cffrelsIn->NR+1);
	
	return tmpOut;  //返回该通道的值
	
}







