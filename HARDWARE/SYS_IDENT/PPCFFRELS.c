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
* Description   : 闭环渐消记忆最小二乘辨识封装（用于直接法自校正极点配置控制器）
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include "PPCFFRELS.h"
#include "LibMatrix.h"   //矩阵库
#include "LibMyMath.h"   //其他数学库


/*
 * Name										: PPCFFRELS_init
 * Description						: 在使用辨识之前需要先用该函数初始化，并指定3个必要的变量，注意这里辨识的初值没有让用户指定
 * Entry                  : PPCFFRELS_T的结构体指针，nf1（不包括积分项）的阶数，ng的阶数，控制延迟d
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
void PPCFFRELS_init(PPCFFRELS_T* relsIn, int na, int nb, int d, float am1, float am2, float am3)
{
	int RELS_i = 0;  //循环用的变量
	float RELS_tmp_M1[PPCFFRELS_ML_A][PPCFFRELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_VT1[1][PPCFFRELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_VT2[1][PPCFFRELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	
	if(na<=0 && nb<=0 && d<=0 && am1<=0 && am2<=0 && am3<=0){
		relsIn->NA = 2;  //如果未定义三个关键参数则使用默认参数
		relsIn->NB = 1;
		relsIn->D = 3;
		matrix_init0((float*)relsIn->AM, 1, PPCFFRELS_ML_A);  //初始化为0
		relsIn->AM[0][0] = 1.0;
		relsIn->AM[0][1] = -1.3;
		relsIn->AM[0][2] = 0.48;
		relsIn->NAM = 2;
	}else{
		relsIn->NA = na;  //给结构体赋值
		relsIn->NB = nb;
		relsIn->D = d;
		matrix_init0((float*)relsIn->AM, 1, PPCFFRELS_ML_A);  //初始化为0
		relsIn->AM[0][0] = am1;
		relsIn->AM[0][1] = am2;
		relsIn->AM[0][2] = am3;
		relsIn->NAM = 2;
	}
	
	//计算一大堆长度参数
	relsIn->NF = relsIn->NB+relsIn->D-1;
	relsIn->NG = relsIn->NA-1;
	relsIn->NA0=2*relsIn->NA-relsIn->NAM-relsIn->NB-1;  //na0=2*na-nam-nb-1; %观测器最低阶次
	
	//计算ML
	relsIn->ML = relsIn->NF+1+relsIn->NG+1;
	
	//计算A0
	matrix_init0((float*)relsIn->A0, 1, PPCFFRELS_ML_A);  //初始化为0
	matrix_init0((float*)RELS_tmp_VT1, 1, PPCFFRELS_ML_A);  //初始化为0
	matrix_init0((float*)RELS_tmp_VT2, 1, PPCFFRELS_ML_A);  //初始化为0
	relsIn->A0[0][0] = 1;   //给A0赋初值
	RELS_tmp_VT1[0][0] = 1;   //[1 0.3-i*0.1]
	RELS_tmp_VT1[0][1] = 0.3; 
	for(RELS_i=0;RELS_i<relsIn->NA0;RELS_i++){
		RELS_tmp_VT1[0][1] -= 0.1;   //A0=conv(A0,[1 0.3-i*0.1]);%生成观测器
		fconv((float*)relsIn->A0, 1+RELS_i, (float*)RELS_tmp_VT1, 2, (float*)RELS_tmp_VT2);
		matrix_copy((float*)RELS_tmp_VT2, 1, PPCFFRELS_ML_A, (float*)relsIn->A0);
		matrix_init0((float*)RELS_tmp_VT2, 1, PPCFFRELS_ML_A);  //初始化为0
	}
	matrix_init0((float*)relsIn->AA, 1, PPCFFRELS_ML_A);  //初始化为0
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
 * Name										: PPCFFRELS_initAsSteady
 * Description						: 用稳态的参数初始化，在使用辨识之前需要先用该函数初始化，并指定3个必要的变量，注意这里辨识的初值没有让用户指定
 * Entry                  : PPCFFRELS_T的结构体指针，nf1（不包括积分项）的阶数，ng的阶数，控制延迟d
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
void PPCFFRELS_initAsSteady(PPCFFRELS_T* relsIn, int na, int nb, int d, float am1, float am2, float am3)
{
	int RELS_i = 0;  //循环用的变量
	
	const int MaxConstLength = 6;
	const float Steady_thetae_1[6][1] = {{0.00506070970223235},{0.00635502256773037},{0.00494823142213283},{0.000918467319518595},{0.491331722635412},{-0.483337742203960}};
	const float Steady_P[6][6] =  {{7.25801162530924e-06,-7.70027366576231e-06,1.21214865112869e-06,6.02023100038719e-07,-5.71430472170938e-06,-6.23557582535170e-06},{
    -7.70027366575120e-06,1.54079156168219e-05,-9.03773372725222e-06,1.02078711838165e-06,1.97846289919320e-05,-1.69472474860069e-05},{
    1.21214865106959e-06,-9.03773372733294e-06,1.54822818505554e-05,-8.13508172041828e-06,2.26339537588858e-05,-1.80521927029820e-05},{
    6.02023100076411e-07,1.02078711837705e-06,-8.13508172005746e-06,8.24563127892771e-06,-5.87241585034854e-05,4.33848944347747e-05},{
    -5.71430472058451e-06,1.97846289907165e-05,2.26339537754104e-05,-5.87241585080893e-05,0.00365387551157519,-0.00344564862994308},{
    -6.23557582636281e-06,-1.69472474839917e-05,-1.80521927234402e-05,4.33848944426555e-05,-0.00344564862984164,0.00344611855273718}};
	
	
	PPCFFRELS_init(relsIn, na, nb, d, am1, am2, am3);  //为了减少冗余代码，先正常初始化
	
	
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
	PPCFFRELS_ClcFGR(relsIn);
}


/*
 * Name										: PPCFFRELS_Update
 * Description						: 渐消记忆递推最小二乘辨识，这里是闭环的系统辨识，输入的都是经过滤波以后的序列
 * Entry                  : PPCFFRELS_T的结构体指针，经滤波后的输出时间序列（当先为0越大越过去），经滤波后的输入时间序列（当先为0越大越过去），遗忘因子（0.9-1）
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
void PPCFFRELS_Update(PPCFFRELS_T* relsIn, float CDataYFK[], float CDataUFK[], float lambda)
{
	//这里先都按照最长情况分配地址，计算时只计算限制的范围
	float RELS_tmp_V1[PPCFFRELS_ML_A][1];  //用于矩阵运算的临时变量 向量
	float RELS_tmp_VT1[1][PPCFFRELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_VT2[1][PPCFFRELS_ML_A];  //用于矩阵运算的临时变量 转置向量
	float RELS_tmp_M1[PPCFFRELS_ML_A][PPCFFRELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M2[PPCFFRELS_ML_A][PPCFFRELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M3[PPCFFRELS_ML_A][PPCFFRELS_ML_A];  //用于矩阵运算的临时变量 矩阵
	float RELS_tmp_M4[PPCFFRELS_ML_A][PPCFFRELS_ML_A];  //用于矩阵运算的临时变量 矩阵
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
	for(RELS_i=0;RELS_i<(relsIn->NF+1);RELS_i++){   //ufk(d:d+nf1)
		relsIn->phie[RELS_i][0] = CDataUFK[relsIn->D+RELS_i];  //这里要注意有D的项不要额外+1 D=1时从过去的第一个也就是现在的第二个也就是[1]开始
	}
	for(RELS_i=0;RELS_i<(relsIn->NG+1);RELS_i++){   //yfk(d:d+ng)
		relsIn->phie[relsIn->NF+1+RELS_i][0] = CDataYFK[relsIn->D+RELS_i];  //这里要注意有D的项不要额外+1
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
// 	matrix_init0((float*)RELS_tmp_VT2, 1, PPCFFRELS_ML_A);  //初始化为0
// 	RELS_tmp_VT2[0][0] = 1;   //[1 -1]
// 	RELS_tmp_VT2[0][1] = -1;
// 	matrix_init0((float*)relsIn->FE, 1, PPCFFRELS_ML_A);  //初始化为0
// 	fconv((float*)relsIn->F1E, relsIn->NF1+1, (float*)RELS_tmp_VT2, 2, (float*)relsIn->FE);   //Fe=conv(F1e,deltaF);
// 	
// 	
// 	matrix_init0((float*)relsIn->R, 1, PPCFFRELS_ML_A);  //初始化为0
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
// 	matrix_multiply_k((float*)relsIn->A0, RELS_tmp_U1, 1, PPCFFRELS_ML_A, (float*)relsIn->R);


	//一切的一切计算FGR的计算都被移动到这里了
	PPCFFRELS_ClcFGR(relsIn);
}



/*
 * Name										: PPCFFRELS_ClcFGR
 * Description						: 根据辨识得到的theta来计算FGR的参数
 * Entry                  : PPCFFRELS_T的结构体指针
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
void PPCFFRELS_ClcFGR(PPCFFRELS_T* relsIn)
{
	//这里先都按照最长情况分配地址，计算时只计算限制的范围
	float RELS_tmp_V1[PPCFFRELS_ML_A][1];  //用于矩阵运算的临时变量 向量
	float RELS_tmp_U1 = 0;  //用于矩阵运算的临时变量 中间变量
	int RELS_i = 0;  //循环用的变量

	//计算观测得到的be0 Fe Ge R
	relsIn->BE0 = relsIn->thetae[0][0];   //be0=thetae(1,k); 
	matrix_multiply_k((float*)relsIn->thetae, 1.0/relsIn->BE0, 1, relsIn->ML, (float*)RELS_tmp_V1);   // thetaeb(:,k)=thetae(:,k)/be0;
	for(RELS_i=0;RELS_i<(relsIn->NF+1);RELS_i++){   //F1e=thetaeb(1:nf1+1,k)';
		relsIn->FE[0][RELS_i] = RELS_tmp_V1[RELS_i][0];  
	}
	for(RELS_i=0;RELS_i<relsIn->NG+1;RELS_i++){   //Ge=thetaeb(nf1+2:nf1+ng+2,k)'; 
		relsIn->GE[0][RELS_i] = RELS_tmp_V1[relsIn->NF+1+RELS_i][0];  
	}	
	
	matrix_init0((float*)relsIn->R, 1, PPCFFRELS_ML_A);  //初始化为0
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
	matrix_multiply_k((float*)relsIn->A0, RELS_tmp_U1, 1, PPCFFRELS_ML_A, (float*)relsIn->R);
}

