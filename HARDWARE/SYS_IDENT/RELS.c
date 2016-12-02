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
* Description   : 最小二乘辨识封装
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include "RELS.h"
#include "LibMatrix.h"   //矩阵库


/*
 * Name										: RELS_init
 * Description						: 在使用辨识之前需要先用该函数初始化，并指定4个必要的变量，注意这里辨识的初值没有让用户指定
 * Entry                  : RELS_T的结构体指针，a的阶数减一，b的阶数减一，c的阶数减一，控制延迟d
 * Return                 : void
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 05/31/2013
 *
 * create.
 * ----------------------
 */
void RELS_init(RELS_T* relsIn, int na, int nb, int nc, int d)
{
	int RELS_i = 0;  //循环用的变量
	float RELS_tmp_M1[RELS_ML_A][RELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	
	if(na<=0 && nb<=0 && nc<=0 && d<=0){
		relsIn->NA = 2;  //如果未定义四个关键参数则使用默认参数
		relsIn->NB = 1;
		relsIn->NC = 1;
		relsIn->D = 3;
	}else{
		relsIn->NA = na;  //给结构体赋值
		relsIn->NB = nb;
		relsIn->NC = nc;
		relsIn->D = d;
	}
	//计算ML
	relsIn->ML = relsIn->NA+relsIn->NB+1+relsIn->NC;
	//RELS初始化
	relsIn->xie = 0;
	//thetae_1=0.001*ones(na+nb+1+nc,1);%非常小的正数（此处不能为0）
	for(RELS_i=0;RELS_i<relsIn->ML;RELS_i++){   
		relsIn->thetae_1[RELS_i][0] = 0.001;  //0.001*1
	}
	//因为把thetae的更新提前了，这里就对一个初始化防止thetae变0
	matrix_copy((float*)relsIn->thetae_1, relsIn->ML, 1, (float*)relsIn->thetae);
	//P=10^6*eye(na+nb+1+nc);  初始化P
	matrix_eye((float*)RELS_tmp_M1, relsIn->ML);
	matrix_multiply_k((float*)RELS_tmp_M1, 1000000.0, relsIn->ML, relsIn->ML, (float*)relsIn->P); 
}

/*
 * Name										: RELS_Update
 * Description						: 根据输入输出数据更新递推增广最小二乘的辨识向量并输出当初观测到的噪声值
 * Entry                  : RELS_T的结构体指针，系统输出的时间序列（当先为0越大越过去），系统输入的时间序列（当先为0越大越过去），系统噪声的时间序列（当先为0越大越过去）
 * Return                 : 根据本次结果观测到的系统噪声
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 05/31/2013
 *
 * create.
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/15/2013
 * CDataU[1+relsIn->D+RELS_i];  //后推了一个       ->         CDataU[relsIn->D+RELS_i];  //U多一个不首一不后推
 * create.
 * ----------------------
 */
float RELS_Update(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[])
{
	//这里先都按照最长情况分配地址，计算时只计算限制的范围
	float RELS_tmp_V1[RELS_ML_A][1];  //用于矩阵运算的临时变量 向量
	float RELS_tmp_VT1[1][RELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_VT2[1][RELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_M1[RELS_ML_A][RELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M2[RELS_ML_A][RELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M3[RELS_ML_A][RELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M4[RELS_ML_A][RELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_UM1[1][1];  //用于矩阵运算的临时变量 单位矩阵
	float RELS_tmp_U1 = 0;  //用于矩阵运算的临时变量 中间变量
	int RELS_i = 0;  //循环用的变量
	
	//----------------------------------------------------------------------------------------
	//thetae_1=thetae(:,k);  这句被从最后移到了这里，为了保证代码的集中，方便移植
	matrix_copy((float*)relsIn->thetae, relsIn->ML, 1, (float*)relsIn->thetae_1);
	//首先构造观测向量ψ   phie=[-yk;uk(d:d+nb);xiek];
	//注意这里实际后推了一个，因为递推要用过去的值来预测本次的输出，所以请确保提供的数据长度足够
	for(RELS_i=0;RELS_i<relsIn->NA;RELS_i++){   //-yk
		relsIn->phie[RELS_i][0] = -CDataYE[1+RELS_i];  //后推了一个
	}
	for(RELS_i=0;RELS_i<(relsIn->NB+1);RELS_i++){   //uk(d:d+nb)
		relsIn->phie[relsIn->NA+RELS_i][0] = CDataU[relsIn->D+RELS_i];  //U多一个不首一不后推
	}
	for(RELS_i=0;RELS_i<relsIn->NC;RELS_i++){   //xiek
		relsIn->phie[relsIn->NA+(relsIn->NB+1)+RELS_i][0] = CDataXiek[1+RELS_i];  //后推了一个
	}
	//K=P*phie/(1+phie'*P*phie);
	matrix_transpose((float*)relsIn->phie, relsIn->ML, 1, (float*)RELS_tmp_VT1);  //phie'
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->P, 1, relsIn->ML, relsIn->ML, (float*)RELS_tmp_VT2); //phie'*P
	matrix_multiply((float*)RELS_tmp_VT2, (float*)relsIn->phie, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1); //phie'*P*phie
	RELS_tmp_U1 = 1/(1+RELS_tmp_UM1[0][0]);  ///(1+phie'*P*phie)
	matrix_multiply((float*)relsIn->P, (float*)relsIn->phie, relsIn->ML, relsIn->ML, 1, (float*)RELS_tmp_V1); //P*phie
	matrix_multiply_k((float*)RELS_tmp_V1, RELS_tmp_U1, relsIn->ML, 1, (float*)relsIn->K);  //K=P*phie/(1+phie'*P*phie);
	//thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->thetae_1, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1); //phie'*thetae_1  前面已经算过phie'了，这里直接用,前面注意保留
	RELS_tmp_U1 = CDataYE[0]-RELS_tmp_UM1[0][0];  //y(k)-phie'*thetae_1
	matrix_multiply_k((float*)relsIn->K, RELS_tmp_U1, relsIn->ML, 1, (float*)RELS_tmp_V1);  //K*(y(k)-phie'*thetae_1)
	matrix_addition((float*)relsIn->thetae_1, (float*)RELS_tmp_V1, relsIn->ML, 1, (float*)relsIn->thetae);  //thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
	//P=(eye(na+nb+1+nc)-K*phie')*P;
	matrix_multiply((float*)relsIn->K, (float*)RELS_tmp_VT1, relsIn->ML, 1, relsIn->ML, (float*)RELS_tmp_M1); //K*phie' 前面已经算过phie'了，这里直接用,前面注意保留
	matrix_eye((float*)RELS_tmp_M2, relsIn->ML);  //eye(na+nb+1+nc)
	matrix_minus((float*)RELS_tmp_M2, (float*)RELS_tmp_M1, relsIn->ML, relsIn->ML, (float*)RELS_tmp_M3);   //(eye(na+nb+1+nc)-K*phie')
	matrix_multiply((float*)RELS_tmp_M3, (float*)relsIn->P, relsIn->ML, relsIn->ML, relsIn->ML, (float*)RELS_tmp_M4);  //(eye(na+nb+1+nc)-K*phie')*P
	matrix_copy((float*)RELS_tmp_M4, relsIn->ML, relsIn->ML, (float*)relsIn->P);  //P=(eye(na+nb+1+nc)-K*phie')*P;
	//xie=y(k)-phie'*thetae(:,k);
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->thetae, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1);  //phie'*thetae(:,k)  前面已经算过phie'了，这里直接用,前面注意保留
	relsIn->xie = CDataYE[0] - RELS_tmp_UM1[0][0];  //xie=y(k)-phie'*thetae(:,k);

// 					%递推增广最小二乘法
// 					phie=[-yk;uk(d:d+nb);xiek];
// 					K=P*phie/(1+phie'*P*phie);
// 					thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
// 					P=(eye(na+nb+1+nc)-K*phie')*P;
// 					
// 					xie=y(k)-phie'*thetae(:,k);%白噪声的估计值
// 					
// 					%提取辨识参数
// 					ae=[1 thetae(1:na,k)']; be=thetae(na+1:na+nb+1,k)'; ce=[1 thetae(na+nb+2:na+nb+1+nc,k)'];
// 					if abs(ce(2))>0.9
// 							ce(2)=sign(ce(2))*0.9;
// 					end
	
	return relsIn->xie;  //这里返回值是观测到的本次的噪声
	
}

/*
 * Name										: RELS_Observ
 * Description						: 仅仅输出根据结构体之前辨识得到的系统的噪声观测数据
 * Entry                  : RELS_T的结构体指针，系统输出的时间序列（当先为0越大越过去），系统输入的时间序列（当先为0越大越过去），系统噪声的时间序列（当先为0越大越过去）
 * Return                 : 根据最后一次辨识结果的系统观测得到的噪声数据
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 05/31/2013
 *
 * create.
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/15/2013
 * CDataU[1+relsIn->D+RELS_i];  //后推了一个       ->         CDataU[relsIn->D+RELS_i];  //U多一个不首一不后推
 * create.
 * ----------------------
 */
float RELS_Observ(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[])
{
	//这里先都按照最长情况分配地址，计算时只计算限制的范围
	float RELS_tmp_VT1[1][RELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_UM1[1][1];  //用于矩阵运算的临时变量 单位矩阵
	int RELS_i = 0;  //循环用的变量
	
	//首先构造观测向量ψ   phie=[-yk;uk(d:d+nb);xiek];
	//注意这里实际后推了一个，因为递推要用过去的值来预测本次的输出，所以请确保提供的数据长度足够
	for(RELS_i=0;RELS_i<relsIn->NA;RELS_i++){   //-yk
		relsIn->phie[RELS_i][0] = -CDataYE[1+RELS_i];  //后推了一个
	}
	for(RELS_i=0;RELS_i<(relsIn->NB+1);RELS_i++){   //uk(d:d+nb)
		relsIn->phie[relsIn->NA+RELS_i][0] = CDataU[relsIn->D+RELS_i];  //U多一个不首一不后推
	}
	for(RELS_i=0;RELS_i<relsIn->NC;RELS_i++){   //xiek
		relsIn->phie[relsIn->NA+(relsIn->NB+1)+RELS_i][0] = CDataXiek[1+RELS_i];  //后推了一个
	}
	
	
	matrix_transpose((float*)relsIn->phie, relsIn->ML, 1, (float*)RELS_tmp_VT1);  //phie'
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->thetae, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1);  //phie'*thetae(:,k)  前面已经算过phie'了，这里直接用,前面注意保留
	relsIn->xie = CDataYE[0] - RELS_tmp_UM1[0][0];  //xie=y(k)-phie'*thetae(:,k);

	
	return relsIn->xie;  //这里返回值是观测到的本次的噪声
}

/*
 * Name										: FFRELS_Update
 * Description						: 渐消记忆递推最小二乘辨识，根据输入输出数据更新递推增广最小二乘的辨识向量并输出当初观测到的噪声值（只增加了遗忘因子，其余部分与RELS通用）
 * Entry                  : RELS_T的结构体指针，系统输出的时间序列（当先为0越大越过去），系统输入的时间序列（当先为0越大越过去），系统噪声的时间序列（当先为0越大越过去），遗忘因子（0.9-1）
 * Return                 : 根据本次结果观测到的系统噪声
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 05/31/2013
 *
 * create.
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/15/2013
 * CDataU[1+relsIn->D+RELS_i];  //后推了一个       ->         CDataU[relsIn->D+RELS_i];  //U多一个不首一不后推
 * create.
 * ----------------------
 */
float FFRELS_Update(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[], float lambda)
{
	//这里先都按照最长情况分配地址，计算时只计算限制的范围
	float RELS_tmp_V1[RELS_ML_A][1];  //用于矩阵运算的临时变量 向量
	float RELS_tmp_VT1[1][RELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_VT2[1][RELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_M1[RELS_ML_A][RELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M2[RELS_ML_A][RELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M3[RELS_ML_A][RELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M4[RELS_ML_A][RELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_UM1[1][1];  //用于矩阵运算的临时变量 单位矩阵
	float RELS_tmp_U1 = 0;  //用于矩阵运算的临时变量 中间变量
	int RELS_i = 0;  //循环用的变量
	
	//规范化数据
	if(lambda>1){
		lambda = 1;
	}else if(lambda<0.9){
		lambda = 0.9;
	}
	
	//----------------------------------------------------------------------------------------
	//thetae_1=thetae(:,k);  这句被从最后移到了这里，为了保证代码的集中，方便移植
	matrix_copy((float*)relsIn->thetae, relsIn->ML, 1, (float*)relsIn->thetae_1);
	//首先构造观测向量ψ   phie=[-yk;uk(d:d+nb);xiek];
	//注意这里实际后推了一个，因为递推要用过去的值来预测本次的输出，所以请确保提供的数据长度足够
	for(RELS_i=0;RELS_i<relsIn->NA;RELS_i++){   //-yk
		relsIn->phie[RELS_i][0] = -CDataYE[1+RELS_i];  //后推了一个
	}
	for(RELS_i=0;RELS_i<(relsIn->NB+1);RELS_i++){   //uk(d:d+nb)
		relsIn->phie[relsIn->NA+RELS_i][0] = CDataU[relsIn->D+RELS_i];  //U多一个不首一不后推
	}
	for(RELS_i=0;RELS_i<relsIn->NC;RELS_i++){   //xiek
		relsIn->phie[relsIn->NA+(relsIn->NB+1)+RELS_i][0] = CDataXiek[1+RELS_i];  //后推了一个
	}
	//K=P*phie/(lambda+phie'*P*phie);
	matrix_transpose((float*)relsIn->phie, relsIn->ML, 1, (float*)RELS_tmp_VT1);  //phie'
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->P, 1, relsIn->ML, relsIn->ML, (float*)RELS_tmp_VT2); //phie'*P
	matrix_multiply((float*)RELS_tmp_VT2, (float*)relsIn->phie, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1); //phie'*P*phie
	RELS_tmp_U1 = 1.0/(lambda+RELS_tmp_UM1[0][0]);  ///(lambda+phie'*P*phie)
	matrix_multiply((float*)relsIn->P, (float*)relsIn->phie, relsIn->ML, relsIn->ML, 1, (float*)RELS_tmp_V1); //P*phie
	matrix_multiply_k((float*)RELS_tmp_V1, RELS_tmp_U1, relsIn->ML, 1, (float*)relsIn->K);  //K=P*phie/(lambda+phie'*P*phie);
	//thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->thetae_1, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1); //phie'*thetae_1  前面已经算过phie'了，这里直接用,前面注意保留
	RELS_tmp_U1 = CDataYE[0]-RELS_tmp_UM1[0][0];  //y(k)-phie'*thetae_1
	matrix_multiply_k((float*)relsIn->K, RELS_tmp_U1, relsIn->ML, 1, (float*)RELS_tmp_V1);  //K*(y(k)-phie'*thetae_1)
	matrix_addition((float*)relsIn->thetae_1, (float*)RELS_tmp_V1, relsIn->ML, 1, (float*)relsIn->thetae);  //thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
	//P=(eye(nf+ng+2)-K*phie')*P/lambda;
	matrix_multiply((float*)relsIn->K, (float*)RELS_tmp_VT1, relsIn->ML, 1, relsIn->ML, (float*)RELS_tmp_M1); //K*phie' 前面已经算过phie'了，这里直接用,前面注意保留
	matrix_eye((float*)RELS_tmp_M2, relsIn->ML);  //eye(na+nb+1+nc)
	matrix_minus((float*)RELS_tmp_M2, (float*)RELS_tmp_M1, relsIn->ML, relsIn->ML, (float*)RELS_tmp_M3);   //(eye(na+nb+1+nc)-K*phie')
	matrix_multiply((float*)RELS_tmp_M3, (float*)relsIn->P, relsIn->ML, relsIn->ML, relsIn->ML, (float*)RELS_tmp_M4);  //(eye(na+nb+1+nc)-K*phie')*P
	matrix_multiply_k((float*)RELS_tmp_M4, 1.0/lambda, relsIn->ML, relsIn->ML, (float*)relsIn->P);  //P=(eye(nf+ng+2)-K*phie')*P/lambda;
	//xie=y(k)-phie'*thetae(:,k);
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->thetae, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1);  //phie'*thetae(:,k)  前面已经算过phie'了，这里直接用,前面注意保留
	relsIn->xie = CDataYE[0] - RELS_tmp_UM1[0][0];  //xie=y(k)-phie'*thetae(:,k);

// 					%递推增广最小二乘法
// 					phie=[ufk(d:d+nf);yfk(d:d+ng)];
// 					K=P*phie/(lambda+phie'*P*phie);
// 					thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
// 					P=(eye(nf+ng+2)-K*phie')*P/lambda;
// 					
// 					xie=y(k)-phie'*thetae(:,k);%白噪声的估计值
// 					
// 					%提取辨识参数
// 					ae=[1 thetae(1:na,k)']; be=thetae(na+1:na+nb+1,k)'; ce=[1 thetae(na+nb+2:na+nb+1+nc,k)'];
// 					if abs(ce(2))>0.9
// 							ce(2)=sign(ce(2))*0.9;
// 					end
	
	return relsIn->xie;  //这里返回值是观测到的本次的噪声
	
}

