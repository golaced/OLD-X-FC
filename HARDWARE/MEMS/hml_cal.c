#include "../HARDWARE/include.h" 
#include "../HARDWARE/MEMS/hml_sample.h"
#include "../HARDWARE/MEMS/hml_cal.h"
#define NUM 6 //线性方程组大小
HML_CAL cycle_hml_cal;
float matrix_Q[NUM1][6][3] = {0};//6组电子罗盘数据，顺序X、Y、Z

double matrix_A[NUM1][NUM][NUM] = {0};//A矩阵由后面的PP函数产生
double matrix_b[NUM1][NUM] = {0,0,0,0,0,0};//同上
double matrix_x[NUM1][NUM] = {0, 0, 0, 0, 0, 0};
double matrix_M[NUM1][7] = {0};        

void PP(u8 state)
{int j ,i;
        double matrix_temp1[NUM][NUM] = {0};

        for ( i = 0; i < NUM; i++)
        {
                matrix_temp1[i][0] = matrix_Q[state][i][0]*matrix_Q[state][i][0];
                matrix_temp1[i][1] = matrix_Q[state][i][1]*matrix_Q[state][i][1];
                matrix_temp1[i][2] = matrix_Q[state][i][2]*matrix_Q[state][i][2];
                matrix_temp1[i][3] = matrix_Q[state][i][0];
                matrix_temp1[i][4] = matrix_Q[state][i][1];
                matrix_temp1[i][5] = matrix_Q[state][i][2];
        }
        for ( j = 0; j < NUM; j++)
        {
                for ( i = 0; i < NUM; i++)
                {
                        matrix_A[state][j][0] += matrix_temp1[i][j]*matrix_temp1[i][0];
                        matrix_A[state][j][1] += matrix_temp1[i][j]*matrix_temp1[i][1];
                        matrix_A[state][j][2] += matrix_temp1[i][j]*matrix_temp1[i][2];
                        matrix_A[state][j][3] += matrix_temp1[i][j]*matrix_temp1[i][3];
                        matrix_A[state][j][4] += matrix_temp1[i][j]*matrix_temp1[i][4];
                        matrix_A[state][j][5] += matrix_temp1[i][j]*matrix_temp1[i][5];
                }
        }
       
        for ( i = 0; i < NUM; i++)
        {
                for ( j = 0; j < NUM; j++)
                {
                        matrix_b[state][i] += -matrix_temp1[j][i];
                }
               // cout << "matrix_b[" << i <<"]" << matrix_b[i] << endl;
             ;  // cout << "*********************************************" << endl;
        }
        for ( i = 0; i < NUM; i++)
        {
                for( j = 0; j < NUM; j++)
                {
                      ; // cout << "matrix_A[" << i <<"]" << "["<< j << "]" << matrix_A[i][j] << endl;
                }
        }
         
}


/******************************************************************************
函数  ： SOR()
功能  ： SOR迭代法解线性方程组
调用  ： 外部
说明  ： 松弛因子0~1 比如0.9    精度自己决定 比如0.000001
返回值： 有
******************************************************************************/
int SOR(u8 state)
{
        int i = 0, j = 0,k = 0;
        double w = 0, precision = 0, temp = 0, x[NUM] = {0, 0, 0, 0, 0, 0};//w:松弛因子   precision：精度
        w =0.9;//cout << "请输入松弛因子w： "<< endl;
        precision=0.001;// cout << "请输入精度： "<< endl;

        for(k = 0; k < 1000; k++)//迭代次数最多不超过1000
        {
                for(i = 0; i < NUM; i++)  
                {
                        temp = 0;
                        for(j = 0; j < NUM; j++)
                        {
                                if(j != i)
                                temp += matrix_A[state][i][j]*matrix_x[state][j];        
                        }
                        matrix_x[state][i] = (1-w)*x[i]+w*(matrix_b[state][i]-temp)/matrix_A[state][i][i];
                      //  cout << matrix_x[i]<< "        ";
                        if(i == 3)
													;
                       // cout << endl;        
                }
                for(i = 0; i < NUM; i++)
                {
                        if(fabs(matrix_x[state][i]-x[i]) < precision)
                        {
                                if(i == 3)
                                {
                                      /*  cout << endl;
                                        cout << "您输入的松弛因子w = " << w << endl;
                                        cout << "  输入的精度precision = " << precision << endl;
                                        cout << "迭代次数： " << k << endl;
                                        cout << "迭代结果："<< endl;
                                        for (int i = 0; i < NUM; i++)
                                        {
                                           cout << "x" << i << ":" << matrix_x[i] << endl;
                                        }
                                        cout <<"x1 = " << matrix_x[0] << "    x2 = " << matrix_x[1];
                                        cout <<       "x3 = " << matrix_x[2] << "    x4 = " << matrix_x[3];
                                        cout <<       "x5 = " << matrix_x[4] << "    x6 = " << matrix_x[5] << endl;*/
                                        return 1;
                                }
                        }
                        else
                        {break;}
                }

                for(i = 0; i < NUM; i++)
                x[i] = matrix_x[state][i];
        }
				
				return 0;
}


void OO(float R,u8 state)//40000=pow X+y+z
{int i;
        matrix_M[state][0] = matrix_x[state][3]/2/matrix_x[state][0];
        matrix_M[state][1] = matrix_x[state][4]/2/matrix_x[state][1];
        matrix_M[state][2] = matrix_x[state][5]/2/matrix_x[state][2];
        matrix_M[state][3] = R/(matrix_x[state][3]*matrix_x[state][3]/matrix_x[state][0]/4
                          +matrix_x[state][4]*matrix_x[state][4]/matrix_x[state][1]/4
                          +matrix_x[state][5]*matrix_x[state][5]/matrix_x[state][2]/4
                                                              -1.0);
		    matrix_M[state][4] = sqrtl( fabs ( (matrix_M[state][3])*matrix_x[state][0] )   );
        matrix_M[state][5] = sqrtl( fabs ( (matrix_M[state][3])*matrix_x[state][1] )   );
        matrix_M[state][6] = sqrtl( fabs ( (matrix_M[state][3])*matrix_x[state][2] )   );
        for( i = 0; i < 7; i++)
        {
                /*cout << endl;
                cout << "******************" << endl;
                cout << "matrix_M[" << i << "]:" << matrix_M[i] << endl;*/
        }
}
	/*
		[1]	12.137754276725149	double
		[2]	23.386814547006360	double
		[3]	-39122.863717754750	double//拟合球体半径
		[4]	0.98711960330191384	double
		[5]	0.96399112693995659	double
		[6]	1.1079369359034139	double
	说明：测量值[Xm,Ym,Zm]      校正后[Xc,Yc,Zc]     平移参数[Ox,Oy,Oz]      缩放参数[gx,gy,gz]   
	   关系：Xc=(Xm+Ox)*gx  
           Yc=(Ym+Oy)*gy 
           Zc=(Zm+Oz)*gz    
  PP(); OO();函数不解释。
  matrix_M[0]对应Ox；matrix_M[1]对应Oy；matrix_M[2]对应Oz；
  matrix_M[4]对应gx； matrix_M[5]对应gy； matrix_M[6]对应gz；*/

u8 HMC_SLOVE(u8 state)
{ 
	PP(state);
  if(SOR(state)){
   OO(440.,state);
		
	cycle_hml_cal.ox[state]=-matrix_M[state][0];cycle_hml_cal.oy[state]=-matrix_M[state][1];cycle_hml_cal.oz[state]=-matrix_M[state][2];
  cycle_hml_cal.kx[state]=matrix_M[state][4];cycle_hml_cal.ky[state]=matrix_M[state][5];cycle_hml_cal.kz[state]=matrix_M[state][6];
	 return 1;	
	}
	
	return 0;
}


float d_yaw=90/NUM1;
float d_yaw2=180/NUM1;
float dead_gz=0.5;
u8 SEL_HML[5];
u8 HML_SAMPLE(u8 en,int hx,int hy,int hz,float pit ,float rol,float gx,float gy,float gz,float T)
{
static float yaw1;	
static u8 state;
u8 i;
	switch(state)
	{
		case 0:
			if(en)
			{	
			SEL_HML[0]=SEL_HML[1]=0;	
			state=1;
			yaw1=0;
			cycle_hml_cal.state=1;	
			matrix_Q[SEL_HML[0]][SEL_HML[1]][0]=hx;matrix_Q[SEL_HML[0]][SEL_HML[1]][1]=hy;matrix_Q[SEL_HML[0]][SEL_HML[1]][2]=hz;		
			SEL_HML[0]++;
				
			}
		break;
	
	  case 1:
			
		 if(SEL_HML[1]>4-1)
		 {yaw1=0;
			 
//		  for(i=0;i<NUM1;i++)		
//			{matrix_Q[i][4][0]=-21;matrix_Q[i][4][1]=-155;matrix_Q[i][4][2]=37;	
//			 matrix_Q[i][5][0]=-57;matrix_Q[i][5][1]=-142;matrix_Q[i][5][2]=-40;	} 
//			for(i=0;i<NUM1;i++)	 
//		  HMC_SLOVE(i);	
			if(fabs(pit)>70){
		 cycle_hml_cal.state=state=2; 
				
			}
		 }else	
		 {
			yaw1+=T*my_deathzoom_2(gz,dead_gz);	
	   if(fabs(yaw1)>d_yaw&&fabs(pit)<15&&fabs(rol)<15)
		 {yaw1=0;
		 	matrix_Q[SEL_HML[0]][SEL_HML[1]][0]=hx;matrix_Q[SEL_HML[0]][SEL_HML[1]][1]=hy;matrix_Q[SEL_HML[0]][SEL_HML[1]][2]=hz;			
			 SEL_HML[0]++;
		 }
		 if(SEL_HML[0]>NUM1-1)
		 {
		 SEL_HML[0]=0;
		 SEL_HML[1]++;	 
		 }
	  }
		 break;
	  
		case 2:
			if(fabs(pit)>70)
	   	yaw1+=T*my_deathzoom_2(gx,dead_gz);
	   if(fabs(yaw1)>d_yaw2&&fabs(pit)>70)
		 {yaw1=0;
		 	matrix_Q[SEL_HML[0]][SEL_HML[1]][0]=hx;matrix_Q[SEL_HML[0]][SEL_HML[1]][1]=hy;matrix_Q[SEL_HML[0]][SEL_HML[1]][2]=hz;			
			 SEL_HML[0]++;
		 }
		 if(SEL_HML[0]>NUM1-1)
		 {
		 SEL_HML[0]=0;
		 SEL_HML[1]++;	 
		 }
		 if(SEL_HML[1]>6-1)
		 {
		 cycle_hml_cal.state=state=3; 
		 }	
		 break;
;
		 
		case 3:

		if(!en)	{
		
	  for(i=0;i<NUM1;i++)		
		HMC_SLOVE(i);	
		cycle_hml_cal.state=state=0; 
			
		}
		break;
	
	}
	

}




u8 HML_SAMPLE_ONLINE(u8 en,int hx,int hy,int hz,float pit ,float rol,float gz,float T)
{


}