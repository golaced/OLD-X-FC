#include "../HARDWARE/include.h"
#include "../HARDWARE/CONTROL/h_inf.h"
//鲁棒控制 未使用

/*
W1d =
 
  100 (z-0.99）
  ------------
     (z-1）
 
Sample time: 0.01 seconds
Discrete-time zero/pole/gain model.


Ks_W2_jd =
 
  -37.5 z^3 + 97.99 z^2 - 85 z + 24.49
  ------------------------------------
  z^3 - 1.669 z^2 + 0.7534 z - 0.04375

% u  num  0----->i
% -
% y  den
*/
double ctrl_inf_att_out,ctrl_inf_att_out2;
#define N_W1_num 2
#define N_W1_den 2
float num_w1[N_W1_num] = {450.0000 ,-112.5000}; // 100.0000  -99.9750};//
float den_w1[N_W1_den] = { 1   , -1};

#define N_K_W2_num 5
#define N_K_W2_den 5
float K_kw2=1;
double num_kw2[N_K_W2_num] = { -11.7911304193333  ,        39.5035680351943     ,    -48.5761905582863    ,      25.7778761120505 , -4.91440406546654};//-28.0892  , 90.8207 ,-106.3194 ,  52.5141  , -8.9262}; //-37.9501 , 150.0452, -222.4559  ,146.5762  ,-36.2156};//
double den_kw2[N_K_W2_den] = {    1    ,     -2.87077504678393      ,    2.97145244756849     ,    -1.30804105872186, 0.207644580710823};//1.0000  , -2.3191   , 1.6869 ,  -0.3748,    0.0071};//	1.0000  , -3.6390  ,  4.9303  , -2.9434  ,  0.6521};//

float ks_w2_0=  -0.413024457588263;
float test1;
float Out_k=1;//0.26
u16 inf_cnt;
float d_inf;
float kd=1;
float ero_inf;
float h_inf_att_inner(float set,float in,float max)//姿态INNER
{
  double ero,temp=0;
	u8 i;
	static double y1w2[10],u1w2[10];
	static double y2[10],u2[10];
 //Ks_W2_j
	  inf_cnt++;
	  static int a;
    u1w2[0]=in*0.0173;//ctrl_inf_att_out2=10*sin(a++*DEG_RAD);
    for(i=1;i<N_K_W2_den;i++)
        {temp=temp-den_kw2[i]*y1w2[i];}
    
    for(i=0;i<N_K_W2_num;i++)
        {temp=temp+K_kw2*num_kw2[i]*u1w2[i];}
    
//更新   
	      if(Thr_Low)   
				  for(i=0;i<=8;i++)
					  u1w2[i]=y1w2[i]=0;
				else
        y1w2[0]=temp;
        //ctrl_inf_att_out=y1w2[0];
				y1w2[8]=y1w2[7];
        y1w2[7]=y1w2[6];
        y1w2[6]=y1w2[5];
        y1w2[5]=y1w2[4];
        y1w2[4]=y1w2[3];
        y1w2[3]=y1w2[2];
        y1w2[2]=y1w2[1];
        y1w2[1]=y1w2[0];
				u1w2[8]=u1w2[7];
        u1w2[7]=u1w2[6];
        u1w2[6]=u1w2[5];
        u1w2[5]=u1w2[4];
        u1w2[4]=u1w2[3];
        u1w2[3]=u1w2[2];
        u1w2[2]=u1w2[1];
        u1w2[1]=u1w2[0];
    

	ero_inf=-set*0.0173*ks_w2_0+y1w2[0];
 
//W1
	  temp=0;
    u2[0]=ero_inf;
    for(i=1;i<N_W1_den;i++)
        temp=temp-den_w1[i]*y2[i];
    
    for(i=0;i<N_W1_num;i++)
        temp=temp+num_w1[i]*u2[i];
    
//更新    
				if(Thr_Low)   
				  for(i=0;i<=8;i++)
					  u2[i]=y2[i]=0;
				else				
        y2[0]=temp;
       
        y2[6]=y2[5];
        y2[5]=y2[4];
        y2[4]=y2[3];
        y2[3]=y2[2];
        y2[2]=y2[1];
        y2[1]=y2[0];
        u2[6]=u2[5];
        u2[5]=u2[4];
        u2[4]=u2[3];
        u2[3]=u2[2];
        u2[2]=u2[1];
        u2[1]=u2[0];
//  if(mode.en_h_inf)  
//	 if(SPID.YI!=0)
//	Out_k=(float)SPID.YI/1000;
	 float d,ero_d;
	 static float ero_r;
	  ero_d=set-in;
	d_inf= 10 *kd *(ero_d - ero_r) *( 0.005f/0.01 ) ;
		ero_r=ero_d;
 return ctrl_inf_att_out=LIMIT(y2[0]*Out_k,-max,max);
}


float thr_inf;
float h_inf_height_spd_out(float set,float in)//高度
{
  double ero,temp=0;
	u8 i;
	static double y1w2[10],u1w2[10];
	static double y2[10],u2[10];
 //Ks_W2_j
	  inf_cnt++;
	  static int a;
    u1w2[0]=in;//ctrl_inf_att_out2=10*sin(a++*DEG_RAD);
    for(i=1;i<N_K_W2_den;i++)
        {temp=temp-den_kw2[i]*y1w2[i];}
    
    for(i=0;i<N_K_W2_num;i++)
        {temp=temp+K_kw2*num_kw2[i]*u1w2[i];}
    
//更新   
	      if(Thr_Low)   
				  for(i=0;i<=8;i++)
					  u1w2[i]=y1w2[i]=0;
				else
        y1w2[0]=temp;
        //ctrl_inf_att_out=y1w2[0];
				y1w2[8]=y1w2[7];
        y1w2[7]=y1w2[6];
        y1w2[6]=y1w2[5];
        y1w2[5]=y1w2[4];
        y1w2[4]=y1w2[3];
        y1w2[3]=y1w2[2];
        y1w2[2]=y1w2[1];
        y1w2[1]=y1w2[0];
				u1w2[8]=u1w2[7];
        u1w2[7]=u1w2[6];
        u1w2[6]=u1w2[5];
        u1w2[5]=u1w2[4];
        u1w2[4]=u1w2[3];
        u1w2[3]=u1w2[2];
        u1w2[2]=u1w2[1];
        u1w2[1]=u1w2[0];
    

	ero_inf=-set*ks_w2_0+y1w2[0];
 
//W1
	  temp=0;
    u2[0]=ero_inf;
    for(i=1;i<N_W1_den;i++)
        temp=temp-den_w1[i]*y2[i];
    
    for(i=0;i<N_W1_num;i++)
        temp=temp+num_w1[i]*u2[i];
    
//更新    
				if(Thr_Low)   
				  for(i=0;i<=8;i++)
					  u2[i]=y2[i]=0;
				else				
        y2[0]=temp;
       
        y2[6]=y2[5];
        y2[5]=y2[4];
        y2[4]=y2[3];
        y2[3]=y2[2];
        y2[2]=y2[1];
        y2[1]=y2[0];
        u2[6]=u2[5];
        u2[5]=u2[4];
        u2[4]=u2[3];
        u2[3]=u2[2];
        u2[2]=u2[1];
        u2[1]=u2[0];
//  if(mode.en_h_inf)  
//	 if(SPID.YI!=0)
//	Out_k=(float)SPID.YI/1000;
	 float d,ero_d;
	 static float ero_r;
	  ero_d=set-in;
	d_inf= 10 *kd *(ero_d - ero_r) *( 0.005f/0.01 ) ;
		ero_r=ero_d;
		if(SPID.YD!=0&&SPID.YD!=800)Out_k=(float)SPID.YD/1000.;
		
 return thr_inf=LIMIT(y2[0]*Out_k,-400,400);
}

float PWM_OFF=0;
float h_inf_att_inner1(float set,float in,float max)
{	static u8 init;
	if(!init){init=1;
		 W1_initialize();
		 KSW2_initialize();
	}
  double ero,temp=0;
	u8 i;
	static double y1w2[10],u1w2[10];
	static double y2[10],u2[10];
	if(!fly_ready||Thr_Low)
	{
	 W1_initialize();
	 KSW2_initialize();
	}
 //Ks_W2_j
	  inf_cnt++;
	  static int a;
	  KSW2_U.In1 =in*0.0173;
	  KSW2_step();
	  ero_inf=-set*0.0173*ks_w2_0+KSW2_Y.Out1;
 
//W1
		W1_U.In1 =ero_inf;
	  W1_step();
 return thr_inf=LIMIT(W1_Y.Out1*Out_k-PWM_OFF,-max,max);
}

float h_inf_att_inner2(float set,float in,float max)
{	static u8 init;
	if(!init){init=1;
		 W1_initialize();
		 KSW2_initialize();
	}
  double ero,temp=0;
	u8 i;
	static double y1w2[10],u1w2[10];
	static double y2[10],u2[10];
	if(!fly_ready||Thr_Low)
	{
	 W1_initialize();
	 KSW2_initialize();
	}
 //Ks_W2_j
	  inf_cnt++;
	  static int a;
	  KSW2_U.In1 =(-set+in)*0.0173;
	  KSW2_step();
	  ero_inf=KSW2_Y.Out1;
 
//W1
		W1_U.In1 =ero_inf;
	  W1_step();
 return thr_inf=LIMIT(W1_Y.Out1*Out_k-PWM_OFF,-max,max);
}
