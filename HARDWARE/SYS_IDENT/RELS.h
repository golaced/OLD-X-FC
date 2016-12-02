#ifndef _RELS_h_
#define _RELS_h_

//递推增广最小二乘估计的相关定义

#define  RELS_ML_A 6  //定义辨识用 全局初始化的 向量和矩阵的最大维度，请确保这个值大于每一个RELS结构体内的ML



//----------------Data struct---------------------------//

typedef struct {
//varitable
	
	//这几个数初始化以后就千万不要再乱改了，不好，不好
	int NA;      //a的阶数减一
	int NB;      //b的阶数减一
	int NC;      //c的阶数减一
	int D;       //d延迟
	int ML;      // ML=NA+NB+1+NC 定义辨识用向量和矩阵的最大维度
	
	float phie[RELS_ML_A][1];  //ψ
	float P[RELS_ML_A][RELS_ML_A];  //单位阵P
	float thetae_1[RELS_ML_A][1];  //θ的前一次的值，初始化为初值
	float thetae[RELS_ML_A][1];  //θ的初值
	float K[RELS_ML_A][1];  //K
	float xie;   //噪声的估计值
	

//function none
}RELS_T;



void RELS_init(RELS_T* relsIn, int na, int nb, int nc, int d);
float RELS_Update(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[]);
float RELS_Observ(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[]);
float FFRELS_Update(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[], float lambda);












#endif
