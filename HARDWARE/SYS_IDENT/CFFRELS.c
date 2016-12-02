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
* Description   : 闭环渐消记忆最小二乘辨识封装（用于直接法自校正极点配置PID控制器）
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include "CFFRELS.h"
#include "LibMatrix.h"   //矩阵库
#include "LibMyMath.h"   //其他数学库


/*
 * Name										: CFFRELS_init
 * Description						: 在使用辨识之前需要先用该函数初始化，并指定3个必要的变量，注意这里辨识的初值没有让用户指定
 * Entry                  : CFFRELS_T的结构体指针，nf1（不包括积分项）的阶数，ng的阶数，控制延迟d
 * Return                 : void
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/14/2013
 *
 * create.
 * ----------------------
 */
void CFFRELS_init(CFFRELS_T* relsIn, int na, int nb, int d, float am1, float am2, float am3)
{
	int RELS_i = 0;  //循环用的变量
	float RELS_tmp_M1[CFFRELS_ML_A][CFFRELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_VT1[1][CFFRELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_VT2[1][CFFRELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	
	if(na<=0 && nb<=0 && d<=0 && am1<=0 && am2<=0 && am3<=0){
		relsIn->NA = 2;  //如果未定义三个关键参数则使用默认参数
		relsIn->NB = 1;
		relsIn->D = 3;
		matrix_init0((float*)relsIn->AM, 1, CFFRELS_ML_A);  //初始化为0
		relsIn->AM[0][0] = 1.0;
		relsIn->AM[0][1] = -1.6;
		relsIn->AM[0][2] = 0.66;
		relsIn->NAM = 2;
	}else{
		relsIn->NA = na;  //给结构体赋值
		relsIn->NB = nb;
		relsIn->D = d;
		matrix_init0((float*)relsIn->AM, 1, CFFRELS_ML_A);  //初始化为0
		relsIn->AM[0][0] = am1;
		relsIn->AM[0][1] = am2;
		relsIn->AM[0][2] = am3;
		relsIn->NAM = 2;
	}
	
	//计算一大堆长度参数
	relsIn->NF = relsIn->NB+relsIn->D;
	relsIn->NG = relsIn->NA;
	relsIn->NF1 = relsIn->NF-1;  //少了积分极点就少了一个长度
	relsIn->NA0=2*relsIn->NA-relsIn->NAM-relsIn->NB-1;  //na0=2*na-nam-nb-1; %观测器最低阶次
	
	//计算ML
	relsIn->ML = relsIn->NF1+1+relsIn->NG+1;
	
	//计算A0
	matrix_init0((float*)relsIn->A0, 1, CFFRELS_ML_A);  //初始化为0
	matrix_init0((float*)RELS_tmp_VT1, 1, CFFRELS_ML_A);  //初始化为0
	matrix_init0((float*)RELS_tmp_VT2, 1, CFFRELS_ML_A);  //初始化为0
	relsIn->A0[0][0] = 1;   //给A0赋初值
	RELS_tmp_VT1[0][0] = 1;   //[1 0.3-i*0.1]
	RELS_tmp_VT1[0][1] = 0.3; 
	for(RELS_i=0;RELS_i<relsIn->NA0;RELS_i++){
		RELS_tmp_VT1[0][1] -= 0.1;   //A0=conv(A0,[1 0.3-i*0.1]);%生成观测器
		fconv((float*)relsIn->A0, 1+RELS_i, (float*)RELS_tmp_VT1, 2, (float*)RELS_tmp_VT2);
		matrix_copy((float*)RELS_tmp_VT2, 1, CFFRELS_ML_A, (float*)relsIn->A0);
		matrix_init0((float*)RELS_tmp_VT2, 1, CFFRELS_ML_A);  //初始化为0
	}
	matrix_init0((float*)relsIn->AA, 1, CFFRELS_ML_A);  //初始化为0
	fconv((float*)relsIn->A0, relsIn->NA0+1, (float*)relsIn->AM, relsIn->NAM+1, (float*)relsIn->AA);   //AA=conv(A0,Am); naa=na0+nam; %A0*Am
	relsIn->NAA = relsIn->NA0+relsIn->NAM;
	
	relsIn->NR = relsIn->NA0;
	
	
	//RELS初始化
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
 * Name										: CFFRELS_initAsSteady
 * Description						: 用稳态的参数初始化，在使用辨识之前需要先用该函数初始化，并指定3个必要的变量，注意这里辨识的初值没有让用户指定
 * Entry                  : CFFRELS_T的结构体指针，nf1（不包括积分项）的阶数，ng的阶数，控制延迟d
 * Return                 : void
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/17/2013
 *
 * create.
 * ----------------------
 */
void CFFRELS_initAsSteady(CFFRELS_T* relsIn, int na, int nb, int d, float am1, float am2, float am3)
{
	int RELS_i = 0;  //循环用的变量
	
	const int MaxConstLength = 7;
	const float Steady_thetae_1[7][1] = {{0.00534},{0.0116},{0.0168},{0.00504},{2.456},{-4.337},{1.941}};
	const float Steady_P[7][7] =  {{6.761e-05,0.0001,0.0001,3.453e-05,0.01785,-0.03252,0.01467},{
    0.0001,0.0002,0.0003,7.7576e-05,0.03783,-0.06835,0.0305},{
    0.0001,0.0003,0.0004,0.0001,0.0523,-0.0939,0.04158},{
    3.4532e-05,7.758e-05,0.0001,5.880e-05,0.01454,-0.02524,0.01070},{
    0.01785,0.0378,0.05234,0.01454,7.3881,-13.34,5.9559},{
    -0.03252,-0.06835,-0.09392,-0.02524,-13.34,24.26,-10.92},{
    0.01467,0.0305,0.04158,0.01070,5.9559,-10.92,4.964}};
	
	
	CFFRELS_init(relsIn, na, nb, d, am1, am2, am3);  //为了减少冗余代码，先正常初始化
	
	
	//RELS初始化
	//按照给定的值初始化
	for(RELS_i=0;RELS_i<relsIn->ML && RELS_i<MaxConstLength;RELS_i++){   
		relsIn->thetae_1[RELS_i][0] = Steady_thetae_1[RELS_i][0];  //把预先设定的值给
	}
	//因为把thetae的更新提前了，这里就对一个初始化防止thetae变0
	matrix_copy((float*)relsIn->thetae_1, relsIn->ML, 1, (float*)relsIn->thetae);
	//按照给定的值初始化
	if(relsIn->ML > MaxConstLength){
		matrix_copy((float*)Steady_P, MaxConstLength, MaxConstLength, (float*)relsIn->P);
	}else{
		matrix_copy((float*)Steady_P, relsIn->ML, relsIn->ML, (float*)relsIn->P);
	}
	
	//根据这个计算出一个初始的控制器
	CFFRELS_ClcFGR(relsIn);
}


/*
 * Name										: CFFRELS_Update
 * Description						: 渐消记忆递推最小二乘辨识，这里是闭环的系统辨识，输入的都是经过滤波以后的序列
 * Entry                  : CFFRELS_T的结构体指针，经滤波后的输出时间序列（当先为0越大越过去），经滤波后的输入时间序列（当先为0越大越过去），遗忘因子（0.9-1）
 * Return                 : void
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/14/2013
 *
 * create.
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/17/2013
 *
 * 将FGR的运算独立了
 * ----------------------
 */
void CFFRELS_Update(CFFRELS_T* relsIn, float CDataYFK[], float CDataUFK[], float lambda)
{
	//这里先都按照最长情况分配地址，计算时只计算限制的范围
	float RELS_tmp_V1[CFFRELS_ML_A][1];  //用于矩阵运算的临时变量 向量
	float RELS_tmp_VT1[1][CFFRELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_VT2[1][CFFRELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_M1[CFFRELS_ML_A][CFFRELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M2[CFFRELS_ML_A][CFFRELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M3[CFFRELS_ML_A][CFFRELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M4[CFFRELS_ML_A][CFFRELS_ML_A];  //用于矩阵运算的临时变量 矩阵
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
	//首先构造观测向量ψ   phie=[ufk(d:d+nf1);yfk(d:d+ng)];   %这里观测的是除掉△的F
	//注意这里没有后推，因为有D的存在干扰了
	for(RELS_i=0;RELS_i<(relsIn->NF1+1);RELS_i++){   //ufk(d:d+nf1)
		relsIn->phie[RELS_i][0] = CDataUFK[relsIn->D+RELS_i];  //这里要注意有D的项不要额外+1 D=1时从过去的第一个也就是现在的第二个也就是[1]开始
	}
	for(RELS_i=0;RELS_i<(relsIn->NG+1);RELS_i++){   //yfk(d:d+ng)
		relsIn->phie[relsIn->NF1+1+RELS_i][0] = CDataYFK[relsIn->D+RELS_i];  //这里要注意有D的项不要额外+1
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
	RELS_tmp_U1 = CDataYFK[0]-RELS_tmp_UM1[0][0];  //y(k)-phie'*thetae_1
	matrix_multiply_k((float*)relsIn->K, RELS_tmp_U1, relsIn->ML, 1, (float*)RELS_tmp_V1);  //K*(y(k)-phie'*thetae_1)
	matrix_addition((float*)relsIn->thetae_1, (float*)RELS_tmp_V1, relsIn->ML, 1, (float*)relsIn->thetae);  //thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
	//P=(eye(nf+ng+2)-K*phie')*P/lambda;
	matrix_multiply((float*)relsIn->K, (float*)RELS_tmp_VT1, relsIn->ML, 1, relsIn->ML, (float*)RELS_tmp_M1); //K*phie' 前面已经算过phie'了，这里直接用,前面注意保留
	matrix_eye((float*)RELS_tmp_M2, relsIn->ML);  //eye(na+nb+1+nc)
	matrix_minus((float*)RELS_tmp_M2, (float*)RELS_tmp_M1, relsIn->ML, relsIn->ML, (float*)RELS_tmp_M3);   //(eye(na+nb+1+nc)-K*phie')
	matrix_multiply((float*)RELS_tmp_M3, (float*)relsIn->P, relsIn->ML, relsIn->ML, relsIn->ML, (float*)RELS_tmp_M4);  //(eye(na+nb+1+nc)-K*phie')*P
	matrix_multiply_k((float*)RELS_tmp_M4, 1.0/lambda, relsIn->ML, relsIn->ML, (float*)relsIn->P);  //P=(eye(nf+ng+2)-K*phie')*P/lambda;

//		%递推最小二乘法
//     phie=[ufk(d:d+nf1);yfk(d:d+ng)];   %这里观测的是除掉△的F
//     K=P*phie/(lambda+phie'*P*phie);
//     thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
//     P=(eye(nf1+ng+2)-K*phie')*P/lambda;   %这里的长度也变了

// 	//计算观测得到的be0 Fe Ge R
// 	relsIn->BE0 = relsIn->thetae[0][0];   //be0=thetae(1,k); 
// 	matrix_multiply_k((float*)relsIn->thetae, 1.0/relsIn->BE0, 1, relsIn->ML, (float*)RELS_tmp_V1);   // thetaeb(:,k)=thetae(:,k)/be0;
// 	for(RELS_i=0;RELS_i<(relsIn->NF1+1);RELS_i++){   //F1e=thetaeb(1:nf1+1,k)';
// 		relsIn->F1E[0][RELS_i] = RELS_tmp_V1[RELS_i][0];  
// 	}
// 	for(RELS_i=0;RELS_i<relsIn->NG+1;RELS_i++){   //Ge=thetaeb(nf1+2:nf1+ng+2,k)'; 
// 		relsIn->GE[0][RELS_i] = RELS_tmp_V1[relsIn->NF1+1+RELS_i][0];  
// 	}
// 	
// 	//Fe=conv(F1e,deltaF);  %求解带积分项的F 既△F
// 	matrix_init0((float*)RELS_tmp_VT2, 1, CFFRELS_ML_A);  //初始化为0
// 	RELS_tmp_VT2[0][0] = 1;   //[1 -1]
// 	RELS_tmp_VT2[0][1] = -1;
// 	matrix_init0((float*)relsIn->FE, 1, CFFRELS_ML_A);  //初始化为0
// 	fconv((float*)relsIn->F1E, relsIn->NF1+1, (float*)RELS_tmp_VT2, 2, (float*)relsIn->FE);   //Fe=conv(F1e,deltaF);
// 	
// 	
// 	matrix_init0((float*)relsIn->R, 1, CFFRELS_ML_A);  //初始化为0
// 	//Bm1=sum(Am)/be0; %Bm'
// 	RELS_tmp_U1 = 0;
// 	for(RELS_i=0;RELS_i<relsIn->NAM+1;RELS_i++){   //sum(Am)
// 		RELS_tmp_U1 += relsIn->AM[0][RELS_i];
// 	}
// 	
// 	
// 	//饮鸩止渴的办法，能缓解非数情况一会儿，但是系统发散的话它是没有任何办法的
// 	if(relsIn->BE0 < 0.00001 && relsIn->BE0 > -0.00001){ //这里做个强制越界判断，这个数是我随便取得，别太大了，因为b0本来就很小
// 		relsIn->BE0 = 0.00001;
// 	}
// //这个可以应付非数……但效果依旧很差
// // 	if((int)(relsIn->BE0*1000) == 0){ //这里做个强制越界判断，这个数是我随便取得，别太大了，因为b0本来就很小
// // 		relsIn->BE0 = 0.00001;
// // 	}
// 	RELS_tmp_U1 /= relsIn->BE0;  //Bm1=sum(Am)/be0;
// 	
// 	//R=Bm1*A0;
// 	matrix_multiply_k((float*)relsIn->A0, RELS_tmp_U1, 1, CFFRELS_ML_A, (float*)relsIn->R);


	//一切的一切计算FGR的计算都被移动到这里了
	CFFRELS_ClcFGR(relsIn);
}



/*
 * Name										: CFFRELS_ClcFGR
 * Description						: 根据辨识得到的theta来计算FGR的参数
 * Entry                  : CFFRELS_T的结构体指针
 * Return                 : void
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/17/2013
 *
 * create.从update函数中提取而来
 * ----------------------
 */
void CFFRELS_ClcFGR(CFFRELS_T* relsIn)
{
	//这里先都按照最长情况分配地址，计算时只计算限制的范围
	float RELS_tmp_V1[CFFRELS_ML_A][1];  //用于矩阵运算的临时变量 向量
	float RELS_tmp_VT2[1][CFFRELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_U1 = 0;  //用于矩阵运算的临时变量 中间变量
	int RELS_i = 0;  //循环用的变量

	//计算观测得到的be0 Fe Ge R
	relsIn->BE0 = relsIn->thetae[0][0];   //be0=thetae(1,k); 
	matrix_multiply_k((float*)relsIn->thetae, 1.0/relsIn->BE0, 1, relsIn->ML, (float*)RELS_tmp_V1);   // thetaeb(:,k)=thetae(:,k)/be0;
	for(RELS_i=0;RELS_i<(relsIn->NF1+1);RELS_i++){   //F1e=thetaeb(1:nf1+1,k)';
		relsIn->F1E[0][RELS_i] = RELS_tmp_V1[RELS_i][0];  
	}
	for(RELS_i=0;RELS_i<relsIn->NG+1;RELS_i++){   //Ge=thetaeb(nf1+2:nf1+ng+2,k)'; 
		relsIn->GE[0][RELS_i] = RELS_tmp_V1[relsIn->NF1+1+RELS_i][0];  
	}
	
	//Fe=conv(F1e,deltaF);  %求解带积分项的F 既△F
	matrix_init0((float*)RELS_tmp_VT2, 1, CFFRELS_ML_A);  //初始化为0
	RELS_tmp_VT2[0][0] = 1;   //[1 -1]
	RELS_tmp_VT2[0][1] = -1;
	matrix_init0((float*)relsIn->FE, 1, CFFRELS_ML_A);  //初始化为0
	fconv((float*)relsIn->F1E, relsIn->NF1+1, (float*)RELS_tmp_VT2, 2, (float*)relsIn->FE);   //Fe=conv(F1e,deltaF);
	
	
	matrix_init0((float*)relsIn->R, 1, CFFRELS_ML_A);  //初始化为0
	//Bm1=sum(Am)/be0; %Bm'
	RELS_tmp_U1 = 0;
	for(RELS_i=0;RELS_i<relsIn->NAM+1;RELS_i++){   //sum(Am)
		RELS_tmp_U1 += relsIn->AM[0][RELS_i];
	}
	
	
	//饮鸩止渴的办法，能缓解非数情况一会儿，但是系统发散的话它是没有任何办法的
	if(relsIn->BE0 < 0.00001 && relsIn->BE0 > -0.00001){ //这里做个强制越界判断，这个数是我随便取得，别太大了，因为b0本来就很小
		relsIn->BE0 = 0.00001;
	}
//这个可以应付非数……但效果依旧很差
// 	if((int)(relsIn->BE0*1000) == 0){ //这里做个强制越界判断，这个数是我随便取得，别太大了，因为b0本来就很小
// 		relsIn->BE0 = 0.00001;
// 	}
	RELS_tmp_U1 /= relsIn->BE0;  //Bm1=sum(Am)/be0;
	
	//R=Bm1*A0;
	matrix_multiply_k((float*)relsIn->A0, RELS_tmp_U1, 1, CFFRELS_ML_A, (float*)relsIn->R);
}

