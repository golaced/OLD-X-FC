#include "../HARDWARE/CONTROL/sonar_avoid.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/include.h"
#include "../HARDWARE/CONTROL/eso.h"
#include "../HARDWARE/MATH/my_math.h"
#include "../HARDWARE/CONTROL/height_ctrl.h"
ESO ESO_BMP;
ESO eso_att_outter[4],eso_att_inner[4];
ESO eso_att_outter_c[4],eso_att_inner_c[4];
#define ESO_PARA_USE_REAL_TIME 1
float fst2(float x1,float x2,float w, float h)
{
	float td_y=0;
	float a0=0,a1,a2;
	float a=0;
	float fhan=0;
	float d=0;
	float d0=0;//dead
	float sy,sa;
	//float flag_y=0;
	//float flag_a=0;
	
	d=w*h*h;
	a0=h*x2;
	td_y=x1+a0;
	a1=my_sqrt(d*(d+8*fabs(td_y)));
	a2=a0+sign(td_y)*(a1-d)/2;
	sy=(sign(td_y+d)-sign(td_y-d))/2;
	a=(a0+td_y-a2)*sy+a2;
	sa=(sign(a+d)-sign(a-d))/2;
	fhan=-w*(a/d-sign(a))*sa-w*sign(a);
	return(fhan);
}
float fst(float x1,float x2,float w, float h)
{
	float td_y=0;
	float a0=0;
	float a=0;
	float fhan=0;
	float d=0;
	float d0=0;//dead
	//float flag_y=0;
	//float flag_a=0;
	
	d=w*h;
	d0=h*d;
	td_y=x1+h*x2;
	a0=sqrt(d*d+8*w*fabs(td_y));
	
	if(fabs(td_y)>d0)
		a=x2+0.5*(a0-d)*sign(td_y);
	else
		a=x2+td_y/h;
		
	//if(a>0) flag_a=1;
	//else 	flag_a=-1;	
	if (fabs(a)>d)
		fhan=-w*sign(a);
	else
		fhan=-w*a/d;
	return(fhan);
}
float fal(float e,float alfa,float delta)
{
	//float flag_e=0.0;
	float y=0.0;
	//if(e>0) flag_e=1.0;
	//if(e<0) flag_e=-1.0;
	//if(e==0) flag_e=0.0;

	if(fabs(e)>delta) y=pow(fabs(e),alfa)*sign(e);
	else			  y=e/pow(delta,1.0-alfa);
	return(y);	
}

float sign(float x)
{
  if(x>0)
      return(1);
  if(x<0)
      return(-1);
}


void SMOOTH_IN_ESO(ESO *eso_in,float in)
{
eso_in->v1+=eso_in->h0*eso_in->v2;                        //td_x1=v1;
eso_in->v2+=eso_in->h0*fst2(eso_in->v1-in,eso_in->v2,eso_in->r0,eso_in->h0);           //td_x2=v2;
}

//------------------------------ESO--------------------------------
float ESO_2N(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4)             // v是控制系统的输入，y是控制系统的输出，反馈给ESO，u是ADRC的输出控制量
{
	float e=0,fe,fe1;
	eso_in->h=eso_in->h0;
//********  eso  *************

	e=eso_in->e=my_deathzoom_2(eso_in->z[0]-y,0);
	fe=fal(e,0.5,eso_in->h0);
	fe1=fal(e,0.25,eso_in->h0);
	eso_in->z[0]+=eso_in->h0*(eso_in->z[1]-eso_in->beta0*e+eso_in->b0*Thr_Weight *u);
	eso_in->z[1]+=-eso_in->h0*eso_in->beta1*e;
	if(eso_in->n==0)
		eso_in->n=1;
	return eso_in->disturb=LIMIT(eso_in->z[1]/eso_in->n,-MAX,MAX);
}

//------------------------------ESO--------------------------------
float ESO_2N_R(ESO *eso_in,float v,float y,float u,float T,float MAX)             // v是控制系统的输入，y是控制系统的输出，反馈给ESO，u是ADRC的输出控制量
{
	float e=0,fe,fe1;
	eso_in->h=T;
//********  eso  *************
	
	e=eso_in->e=my_deathzoom_2(eso_in->z[0]-y,0);
	fe=fal(e,0.5,eso_in->h0);
	fe1=fal(e,0.25,eso_in->h0);
	eso_in->z[0]+=eso_in->h0*(eso_in->z[1]-eso_in->beta0*e+eso_in->b0*Thr_Weight *u);
	eso_in->z[1]+=-eso_in->h0*eso_in->beta1*fe;
	if(eso_in->n==0)
		eso_in->n=1;
	return eso_in->disturb=LIMIT(eso_in->z[1]/eso_in->n,-MAX,MAX);
}


float ESO_3N(ESO *eso_in,float v,float y,float u,float T,float MAX)             // v是控制系统的输入，y是控制系统的输出，反馈给ESO，u是ADRC的输出控制量
{
	float e=0,fe,fe1;
	eso_in->h=eso_in->h0;
//********  eso  *************
	e=eso_in->e=my_deathzoom_2(eso_in->z[0]-y,0.25);
	fe=fal(e,0.5,eso_in->h0);
	fe1=fal(e,0.25,eso_in->h0);
	eso_in->z[0]+=eso_in->h*(eso_in->z[1]-eso_in->beta0*e);
	eso_in->z[1]+=eso_in->h*(eso_in->z[2]-eso_in->beta1*fe+eso_in->b0*Thr_Weight *u);
	eso_in->z[2]+=-eso_in->h*eso_in->beta2*fe1;
		if(eso_in->n==0)
		eso_in->n=1;
	return eso_in->disturb=LIMIT(eso_in->z[2]/eso_in->n,-MAX,MAX);
}  

float K_ESO_D=0.0;
float ESO_CONTROL(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4)
{static float e0,e1,e2;
if(mode.use_px4_err)
{
e1=ero_px4;
e2=-eso_in->z[1];
}
else if(!eso_in->use_td)
{
e1=v-eso_in->z[0];
e2=-eso_in->z[1];
}
else{
e1=eso_in->v1-eso_in->z[0];
e2=eso_in->v2-eso_in->z[1];
}
e0+=eso_in->e;//*T;

	//switch(eso_in->out_mode)
	{
//		case 0:eso_in->u=//LIMIT( eso_in->KI*e0, -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT )
//										eso_in->KP*e1;
//										//+eso_in->KD*e2;
//										break;
//		case 1:
		eso_in->u=eso_in->KP*fal(e1,eso_in->alfa1,eso_in->tao);//+eso_in->KD*K_ESO_D*fal(e2,eso_in->alfa2,eso_in->tao);//-LIMIT( eso_in->KI*fal(e0,eso_in->alfa0,eso_in->tao), -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT )
										
										//+eso_in->KD*e2;
//										break;
//	  case 2:eso_in->u=-fst2(e1,eso_in->c*e2,eso_in->r1,eso_in->h1);break;
//		case 3:eso_in->u=LIMIT( eso_in->KI*e0, -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT )
//											-fst2(e1,e2,eso_in->r1,eso_in->h1);break;
	}
	
	if(eso_in->b0!=0){
  switch(eso_in->level){
		case 1:eso_in->disturb_u=eso_in->z[1]/eso_in->b0;break; 
		case 2:eso_in->disturb_u=eso_in->z[2]/eso_in->b0;break;
	}
  eso_in->u-=Thr_Weight *eso_in->disturb_u;
	}
return  eso_in->u=LIMIT(eso_in->u+eso_in->integer,-MAX,MAX);
}

#define YAW_ERO_MAX 45
float ESO_CONTROL_Y(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4)
{static float e0,e1,e2;
if(!eso_in->use_td)
{
e1=v-eso_in->z[0];
e2=-eso_in->z[1];
}
else{
e1=eso_in->v1-eso_in->z[0];
e2=eso_in->v2-eso_in->z[1];
}
e0+=eso_in->e;//*T;

	//switch(eso_in->out_mode)
	{
//		case 0:eso_in->u=//LIMIT( eso_in->KI*e0, -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT )
//										eso_in->KP*e1;
//										//+eso_in->KD*e2;
//										break;
//		case 1:
		eso_in->u=eso_in->KP*fal(LIMIT(To_180_degrees(e1),-YAW_ERO_MAX,YAW_ERO_MAX),eso_in->alfa1,eso_in->tao);//+eso_in->KD*K_ESO_D*fal(e2,eso_in->alfa2,eso_in->tao);//-LIMIT( eso_in->KI*fal(e0,eso_in->alfa0,eso_in->tao), -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT )
										
										//+eso_in->KD*e2;
//										break;
//	  case 2:eso_in->u=-fst2(e1,eso_in->c*e2,eso_in->r1,eso_in->h1);break;
//		case 3:eso_in->u=LIMIT( eso_in->KI*e0, -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT )
//											-fst2(e1,e2,eso_in->r1,eso_in->h1);break;
	}
	
	if(eso_in->b0!=0){
  switch(eso_in->level){
		case 1:eso_in->disturb_u=eso_in->z[1]/eso_in->b0;break; 
		case 2:eso_in->disturb_u=eso_in->z[2]/eso_in->b0;break;
	}
  eso_in->u-=Thr_Weight *eso_in->disturb_u;
	}
return  eso_in->u=LIMIT(eso_in->u+eso_in->integer,-MAX,MAX);
}




float ATT_CONTRL_OUTER_ESO_3(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4)
{ if(!eso_in->init)
	{
  eso_in->init=1;
	eso_in->level=1;//系统阶次	
 //-------------跟踪器
	eso_in->use_td=0;		
	eso_in->r0=4000;//跟踪速度
  eso_in->h0=(float)F_OUTTER/1000;//滤波因子
 //-------------观测器
		#if EN_TIM_INNER
	  eso_in->beta0=200;
	  eso_in->beta1=600;//1/(3*pow(eso_in->h0,2));
    eso_in->beta2=2000;//2/(64*pow(eso_in->h0,2));	
		#else
//		eso_in->beta0=100;
//	  eso_in->beta1=300;//1/(3*pow(eso_in->h0,2));
//    eso_in->beta2=1000;//2/(64*pow(eso_in->h0,2));	
			eso_in->beta0=1/(eso_in->h0);
   	  eso_in->beta1=1/(3*pow(eso_in->h0,2));
      eso_in->beta2=1000;//2/(64*pow(eso_in->h0,2));	
		#endif
 //-------------反馈----------------
	eso_in->out_mode=1;	
		//-------liner    0
		switch(eso_in->out_mode){
			case 0:
			//eso_in->KP=0.5;//
			eso_in->KP=ctrl_2.PID[PIDROLL].kp*2;
			//eso_in->KI=0;//ctrl_2.PID[PIDROLL].ki;	
			//eso_in->KD=2.2;
			eso_in->KD=ctrl_2.PID[PIDROLL].kd*4;
			break;
			case 1:
			eso_in->KP=ctrl_2.PID[PIDROLL].kp*2;//ctrl_2.PID[PIDROLL].kp*2;
    	eso_in->KI=0;//ctrl_2.PID[PIDROLL].ki;	
	    eso_in->KD=ctrl_2.PID[PIDROLL].kd*4;//ctrl_2.PID[PIDROLL].kd/400;
			break;
			
		}
		//-------noliner  1
	eso_in->alfa0=0.25;
  eso_in->alfa1=0.75;
	eso_in->alfa2=1.5;	
  eso_in->tao=eso_in->h0*2;		
	  //-------noliner  2 3
	eso_in->c=0.5;//阻尼因子	
	eso_in->r1=0.5/pow(eso_in->h0,2);
	eso_in->h1=eso_in->h0*5;
	//----------模型增益
			switch(mcuID[0]){
		case 3145777://300
						  eso_in->b0=10;		
		break;
		case 2818103://450
						  eso_in->b0=10;//15;		
		break;
		default:  eso_in->b0=100;		
		break;
		}

	}  
	#if ESO_PARA_USE_REAL_TIME
	    eso_in->h0=T;
			eso_in->beta0=1/(eso_in->h0+0.000001);
   	  eso_in->beta1=1/(3*pow(eso_in->h0,2)+0.000001);
	#endif
		if(mode.att_pid_tune&&mode.en_pid_sb_set){
	eso_in->KD=0.001*SPID.OD*4;
	eso_in->KP=0.001*SPID.OP*2;
		}
	//if(SPID.YD!=0)eso_in->b0=SPID.YD;
	SMOOTH_IN_ESO(eso_in,v);
	switch(eso_in->level){
		case 1:ESO_2N(eso_in,v, y, u, T, MAX,ero_px4);break;
		case 2:ESO_3N(eso_in,v, y, u, T, MAX);break;
	}

	eso_in->integer+=eso_in->KI*(v-y)*T;
	eso_in->integer = LIMIT( eso_in->integer, -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT );
	ESO_CONTROL(eso_in,v, y, u, T, MAX,ero_px4);
	return eso_in->u;
}


float ATT_CONTRL_OUTER_ESO_3_Y(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4)//航向
{ if(!eso_in->init)
	{
  eso_in->init=1;
	eso_in->level=1;//系统阶次	
 //-------------跟踪器
	eso_in->use_td=0;		
	eso_in->r0=4000;//跟踪速度
  eso_in->h0=T;//滤波因子
 //-------------观测器
		#if EN_TIM_INNER
	  eso_in->beta0=200;
	  eso_in->beta1=600;//1/(3*pow(eso_in->h0,2));
    eso_in->beta2=2000;//2/(64*pow(eso_in->h0,2));	
		#else
		eso_in->beta0=100;
	  eso_in->beta1=300;//1/(3*pow(eso_in->h0,2));
    eso_in->beta2=1000;//2/(64*pow(eso_in->h0,2));	
		#endif

 //-------------反馈----------------
	eso_in->out_mode=1;	
		//-------liner    0
		switch(eso_in->out_mode){
			case 0:
			//eso_in->KP=0.5;//
			eso_in->KP=ctrl_2.PID[PIDYAW].kp*2;
			//eso_in->KI=0;//ctrl_2.PID[PIDROLL].ki;	
			//eso_in->KD=2.2;
			eso_in->KD=ctrl_2.PID[PIDYAW].kd*4;
			break;
			case 1:
			eso_in->KP=ctrl_2.PID[PIDYAW].kp*2;//ctrl_2.PID[PIDROLL].kp*2;
    	eso_in->KI=0;//ctrl_2.PID[PIDROLL].ki;	
	    eso_in->KD=ctrl_2.PID[PIDYAW].kd*4;//ctrl_2.PID[PIDROLL].kd/400;
			break;
			
		}
		//-------noliner  1
	eso_in->alfa0=0.25;
  eso_in->alfa1=0.75;
	eso_in->alfa2=1.5;	
  eso_in->tao=eso_in->h0*2;		
	  //-------noliner  2 3
	eso_in->c=0.5;//阻尼因子	
	eso_in->r1=0.5/pow(eso_in->h0,2);
	eso_in->h1=eso_in->h0*5;
	//----------模型增益
			switch(mcuID[0]){
		case 3145777://300
						  eso_in->b0=10;		
		break;
		case 2818103://450
						  eso_in->b0=10;//15;		
		break;
		default:  eso_in->b0=100;		
		break;
		}

	}
		if(mode.att_pid_tune&&mode.en_pid_sb_set){
	eso_in->KD=0.001*SPID.OD*4;
	eso_in->KP=0.001*SPID.OP*2;
		}
	//if(SPID.YD!=0)eso_in->b0=SPID.YD;
	SMOOTH_IN_ESO(eso_in,v);
	switch(eso_in->level){
		case 1:ESO_2N(eso_in,v, y, u, T, MAX,ero_px4);break;
		case 2:ESO_3N(eso_in,v, y, u, T, MAX);break;
	}

	eso_in->integer+=eso_in->KI*(v-y)*T;
	eso_in->integer = LIMIT( eso_in->integer, -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT );
	ESO_CONTROL_Y(eso_in,v, y, u, T, MAX,ero_px4);
	return eso_in->u;
}

float ATT_CONTRL_INNER_ESO_3(ESO *eso_in,float v,float y,float u,float T,float MAX)
{ if(!eso_in->init)
	{
  eso_in->init=1;
	eso_in->level=1;//系统阶次	
 //-------------跟踪器
	eso_in->use_td=0;		
	eso_in->r0=20000;//跟踪速度
  eso_in->h0=T;//滤波因子
 //-------------观测器
	eso_in->beta0=100;
	eso_in->beta1=300;
  eso_in->beta2=2000;

 //-------------反馈----------------
	eso_in->out_mode=1;	
		//-------liner    0
		switch(eso_in->out_mode){
			case 0:
			eso_in->KP=ctrl_1.PID[PIDROLL].kp;
			eso_in->KI=0;//ctrl_2.PID[PIDROLL].ki;	
			eso_in->KD=ctrl_1.PID[PIDROLL].kd/40;
			break;
			case 1:
			eso_in->KP=ctrl_1.PID[PIDROLL].kp*10;
    	eso_in->KI=0;//ctrl_2.PID[PIDROLL].ki;	
	    eso_in->KD=ctrl_1.PID[PIDROLL].kd;
			break;
			
		}
		//-------noliner  1
	eso_in->alfa0=0.25;
  eso_in->alfa1=0.75;
	eso_in->alfa2=1.5;	
  eso_in->tao=eso_in->h0*2;		
	  //-------noliner  2 3
	eso_in->c=0.5;//阻尼因子	
	eso_in->r1=0.5/pow(eso_in->h0,2);
	eso_in->h1=eso_in->h0*5;
	//----------模型增益
  eso_in->b0=40;		
	}
	//if(SPID.YD!=0)eso_in->b0=SPID.YD;
	SMOOTH_IN_ESO(eso_in,v);
	switch(eso_in->level){
		case 1:ESO_2N_R(eso_in,v, y, u, T, MAX);break;
		case 2:ESO_3N(eso_in,v, y, u, T, MAX);break;
	}

	eso_in->integer+=eso_in->KI*(v-y)*T;
	eso_in->integer = LIMIT( eso_in->integer, -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT );
	ESO_CONTROL(eso_in,v, y, u, T, MAX,0);
	return eso_in->u;
}


//-----------------------------HEIGHT-ESO--------------------------------
float ESO_CONTROL_HEIGH(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4)
{static float e0,e1,e2;

e1=v-eso_in->z[0];
e2=-eso_in->z[1];
eso_in->u=eso_in->KP*e1;
	
	if(eso_in->b0!=0&&!mode.height_safe){
		eso_in->disturb_u=eso_in->z[1]/eso_in->b0;
    eso_in->u-=Thr_Weight *eso_in->disturb_u;
	}
return  eso_in->u=LIMIT(eso_in->u,-MAX,MAX);
}

float ESO_2N_H(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4)             // v是控制系统的输入，y是控制系统的输出，反馈给ESO，u是ADRC的输出控制量
{
	float e=0,fe,fe1;
	eso_in->h=eso_in->h0;
	e=eso_in->e=my_deathzoom_2(eso_in->z[0]-y,25);
	fe=fal(e,0.5,eso_in->h0);
	fe1=fal(e,0.25,eso_in->h0);
	eso_in->z[0]+=eso_in->h0*(eso_in->z[1]-eso_in->beta0*e+eso_in->b0*Thr_Weight *u);
	eso_in->z[1]+=-eso_in->h0*eso_in->beta1*e;
	eso_in->z[1]=LIMIT(eso_in->z[1],-MAX*Thr_Weight*(eso_in->b0+1),MAX*Thr_Weight*(eso_in->b0+1));
	if(mode.height_safe)
	eso_in->z[1]=0;	
	if(eso_in->n==0)
		eso_in->n=1;
	return eso_in->disturb=LIMIT(eso_in->z[1]/eso_in->n,-MAX,MAX);
}

float HIGH_CONTROL_SPD_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX)
{ if(!eso_in->init)
	{
  eso_in->init=1;
	eso_in->level=1;//系统阶次	
 //-------------跟踪器
	eso_in->use_td=0;		
	eso_in->r0=4000;//跟踪速度
  eso_in->h0=(float)H_INNER/1000;//滤波因子
 //-------------观测器
//		eso_in->beta0=100;
//	  eso_in->beta1=300;//1/(3*pow(eso_in->h0,2));
//    eso_in->beta2=1000;//2/(64*pow(eso_in->h0,2));	
			eso_in->beta0=1/(eso_in->h0);
   	  eso_in->beta1=1/(3*pow(eso_in->h0,2));
      eso_in->beta2=1000;//2/(64*pow(eso_in->h0,2));	
 //-------------反馈----------------
	eso_in->out_mode=1;	
		//-------liner    0
	eso_in->KP=wz_speed_pid.kp;
	eso_in->KI=0;
	eso_in->KD=2;
	//-------noliner  1
	eso_in->alfa0=0.25;
  eso_in->alfa1=0.75;
	eso_in->alfa2=1.5;	
  eso_in->tao=eso_in->h0*2;		
	  //-------noliner  2 3
	eso_in->c=0.5;//阻尼因子	
	eso_in->r1=0.5/pow(eso_in->h0,2);
	eso_in->h1=eso_in->h0*5;
	//----------模型增益
  eso_in->b0=35;		
	}   
	 #if ESO_PARA_USE_REAL_TIME
	    eso_in->h0=T;
  		eso_in->beta0=1/(eso_in->h0+0.000001);
   	  eso_in->beta1=1/(3*pow(eso_in->h0,2)+0.000001);
	 #endif
	eso_in->KP=wz_speed_pid.kp;
	if(SPID.YD!=0&&SPID.YD!=800&&(KEY[0]==1&&KEY[1]==1))eso_in->b0=SPID.YD;
  ESO_2N_H(eso_in,v, y, u, T, MAX,0);
	ESO_CONTROL_HEIGH(eso_in,v, y, u, T, MAX,0);
	return eso_in->u;
}


void ESO_BMP_INIT(ESO *eso_in)
{ 		
	eso_in->r0=1;//跟踪速度
  eso_in->h0=0.03;//滤波因子
}
