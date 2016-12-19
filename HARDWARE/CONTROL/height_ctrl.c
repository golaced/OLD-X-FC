#include "../HARDWARE/CONTROL/height_ctrl.h"
#include "../HARDWARE/MEMS/ultrasonic.h"
#include "../HARDWARE/MATH/my_math.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/MEMS/ms5611_2.h"
#include "../HARDWARE/DRIVER/rc_mine.h"
#include "../HARDWARE/FUSHION/alt_kf.h"
#include "../HARDWARE/CONTROL/eso.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include "../HARDWARE/include.h"
#include "../HARDWARE/ucos_task.h"
//#include "rng.h"

#define EXP_Z_SPEED  ( 4.2f *my_deathzoom( (thr-500), 80 ) )//----------遥控对应期望速度
//#if defined(ZHOU_550)
//	#define HOLD_THR 450+100 
//#elif defined(ZHOU_330)
//	#define HOLD_THR 500 
//#elif defined(ZHOU_300)
//	#define 
//#endif

float rng_time1;
float HOLD_THR =450+100; //悬停油门
float ALT_HOLD_THR_RANGE_SCALE=2;


float exp_spd_zv,thr_use,thr_in_view;
_st_height_pid_v wz_speed_pid_v;
_st_height_pid_v wz_speed_pid_v_safe;
_st_height_pid_v wz_speed_pid_v_eso;

_st_height_pid_v ultra_ctrl,ultra_ctrl_safe;

_st_height_pid wz_speed_pid,wz_speed_pid_safe;
_st_height_pid wz_speed_pid_use;
_st_height_pid ultra_pid_safe,ultra_pid;
 _st_height_pid ultra_pid_use;

#define MAX_HEIGH_ERO 1000
float exp_height_speed,exp_height;
float exp_height_speed_safe,exp_height_safe;
float ultra_speed,ultra_speed_safe;
float ultra_dis_lpf,ultra_dis_lpf_safe;
float ultra_ctrl_out_safe, ultra_ctrl_out,ultra_ctrl_out_use;
float baro_speed_ano;
float height_ctrl_out;
float wz_acc;
float adrc_out;
void Ultra_PID_Init()
{

	HOLD_THR =HOLD_THR_PWM;//《--------------------修改悬停油门  根据不同飞行器动力和重量实验 
//-------------------外环PID参数初始化
	ultra_pid.kp = 0.88;
	ultra_pid.ki = 0.05;		
		ultra_pid.kd = 1.2;
//------------------安全模式 只有速度环
  ultra_pid_safe.kp = 0;//0.45;//50.0;//1.8;//1.65;//1.5;
	ultra_pid_safe.ki = 0.0;//1;//add
	ultra_pid_safe.kd = 0.0;
}

  
void WZ_Speed_PID_Init()
{//use
	  wz_speed_pid.kp = 0.35;//1.25;//0.3;//0.25;//0.5;//0.3; 
		wz_speed_pid.ki = 0.1;//225;//1.4;//0.08;//0.5; 
		wz_speed_pid.kd = 2.0;//0.5;//8;//1.5;
	//------------------------------安全模式PID初始化
		wz_speed_pid_safe.kp = 0.35;//120.4;//0.5;//0.3; 
		wz_speed_pid_safe.ki = 0.1;//45.5; 
		wz_speed_pid_safe.kd = 2.0; 
}

#define BARO_SPEED_NUM 50
float baro_speed_arr[BARO_SPEED_NUM + 1];
u16 baro_cnt[2];
u8 switch_r;
int wz_acc_ukf;
void height_mode_switch(void)//气压计 超声波自动切换
{
static u8 state;
static u16 cnt,cnt1;	

// if(height_ctrl_mode==2){
//	 
//	 switch(state)
//			{ 
//			case 0:
//			if(ultra_ok==1){
//			if(baro_only_move>BMP_MAX_HEIGHT)
//			{state=1;height_ctrl_mode_use=1;cnt=0;}
//			else
//				height_ctrl_mode_use=2;
//			}
//			else
//			height_ctrl_mode_use=1;
//			break;
//			case 1:
//			if(ALT_POS_SONAR2*1000<ULTRA_MAX_HEIGHT-100)
//			cnt++;
//			else
//			cnt=0;

//			if(cnt>800)
//			{state=0;height_ctrl_mode_use=2;cnt=0;}

//			break;
//			}

//}
// else
 	height_ctrl_mode_use=height_ctrl_mode;//现在直接通过遥控器切换
}
float sonar_spd;
u8 BARO_HIHG_NUM=1;//10;//30;
float baro_h_arr[100 + 1];
u16 baro_h_cnt[2];
float baro_h_arr1[100 + 1];
u16 baro_h_cnt1[2];
float k_temp=12; 
u8 hs_ctrl_cnt_max[2]={3,1},hold_alt_flag;
float out_timer_high,in_timer_high;
float baro_only_move,baro_only_move_flt;
float ano_temp;
float k_acc_ukf=10;//20;
u8 mid_usp=5;
u8 EN_TUO_ACC_FIX_H=0;
float off_inner=2;
void Height_Ctrl(float T,float thr)
{			static u8 hs_ctrl_cnt;
	static float wz_speed_t;
	static u8 height_ctrl_start_f,height_mode_reg;
	static u16 hc_start_delay;
	float t_sonar_out;
	float temp;
	float t_h=Get_Cycle_T(GET_T_HIGH_CONTROL)+0.0001;
	switch( height_ctrl_start_f )
	{		
		case 0:

		#if EN_ATT_CAL_FC //如果使用FC模块计算姿态
		temp=(reference_vr_imd_down_fc[2] *mpu6050_fc.Acc.z + reference_vr_imd_down_fc[0] *mpu6050_fc.Acc.x + reference_vr_imd_down_fc[1 ] *mpu6050_fc.Acc.y - 4096  );
		#else
		temp=(reference_vr_imd_down[2] *mpu6050.Acc.z + reference_vr_imd_down[0] *mpu6050.Acc.x + reference_vr_imd_down[1 ] *mpu6050.Acc.y - 4096  );
		#endif
		
		//低通滤波计算垂直方向加速度
		wz_acc_ukf += ( 1 / ( 1 + 1 / ( k_acc_ukf *3.14f *T ) ) ) *my_deathzoom( (temp - wz_acc_ukf),0 );
		wz_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom( (temp - wz_acc),0);
		Moving_Average( (float)( baroAlt),baro_h_arr,BARO_HIHG_NUM, baro_h_cnt ,&baro_only_move ); //单位mm/s
		Moving_Average( (float)( baroAlt),baro_h_arr1,85, baro_h_cnt1 ,&baro_only_move_flt ); //单位mm/s
	if(mode.en_h_mode_switch)
				height_mode_switch();
	
	    if( hs_ctrl_cnt++>=(float)(H_INNER-off_inner)/(t_h*1000))//PA
			{  //----------------------mode switch----------------------
				in_timer_high=Get_Cycle_T(GET_T_HIGH_CONTROL_I);hs_ctrl_cnt=0;
				if(height_mode_reg==1&&height_ctrl_mode_use==2)//SONAR<-BMP
				{exp_height=ultra_dis_lpf=ALT_POS_SONAR2*1000;}
				else if(height_mode_reg==2&&height_ctrl_mode_use==1)//SONAR->BMP
				{exp_height=ultra_dis_lpf=baro_only_move_flt;}
				 height_mode_reg=height_ctrl_mode_use;
				//---------------------safe height--------------------------------
				static u8 safe_mode_reg;
				 if(safe_mode_reg==0&&mode.height_safe==1)//SONAR->BMP
				{exp_height_safe=ultra_dis_lpf_safe=baro_only_move_flt;}
				else if(safe_mode_reg==1&&mode.height_safe==0)
				{
				if(height_ctrl_mode_use==2)//SONAR<-BMP
				{exp_height=ultra_dis_lpf=ALT_POS_SONAR2*1000;}
				else if(height_ctrl_mode_use==1)//SONAR->BMP
				{exp_height=ultra_dis_lpf=baro_only_move_flt;}
				
				}safe_mode_reg=mode.height_safe;
				static u8 state_thr,cnt_down;
				static float thr_reg;
				thr_in_view=thr;
				
				thr_use=thr;//最终使用遥感量
				if(mode.height_safe)//安全模式下PID参数选择
				{
				mode.height_in_speed=1;
				ultra_pid_use.kp =ultra_pid_safe.kp;ultra_pid_use.ki =ultra_pid_safe.ki; ultra_pid_use.kd =ultra_pid_safe.kd;  
				wz_speed_pid_use.kp =wz_speed_pid_safe.kp;wz_speed_pid_use.ki =wz_speed_pid_safe.ki; wz_speed_pid_use.kd =wz_speed_pid_safe.kd;  
				}
				else
				{
				ultra_pid_use.kp =ultra_pid.kp;ultra_pid_use.ki =ultra_pid.ki; ultra_pid_use.kd =ultra_pid.kd;  
				wz_speed_pid_use.kp =wz_speed_pid.kp;wz_speed_pid_use.ki =wz_speed_pid.ki; wz_speed_pid_use.kd =wz_speed_pid.kd; 
				}
				 //------------------------SPD CONTOLLER------------------
				    exp_spd_zv=EXP_Z_SPEED;//调试用
					
						Moving_Average( (float)( baro_alt_speed_ano ),baro_speed_arr,BARO_SPEED_NUM, baro_cnt ,&ano_temp ); //单位mm/s
						int ultra_sp_tmp;
						 #if EN_ATT_CAL_FC
						 ultra_sp_tmp=Moving_Median(2,mid_usp,my_deathzoom_2((-ALT_VEL_BMP_UKF_OLDX)*1000,25));//my_deathzoom_2(ALT_VEL_BMP*1000,50);//Moving_Median(2,5,my_deathzoom_2(ALT_VEL_BMP*1000,50));	 
					   #else
						 ultra_sp_tmp=Moving_Median(2,mid_usp,my_deathzoom_2((-ALT_VEL_BMP_EKF)*1000,25)); 
						 #endif
						if( abs(ultra_sp_tmp) < 100 )
						{
							ultra_speed_bmp += ( 1 / ( 1 + 1 / ( 4*2 *3.14f *T*1*k_temp ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed_bmp );
						}
						else
						{
							ultra_speed_bmp += ( 1 / ( 1 + 1 / ( 1.0f*2 *3.14f *T*1.5*k_temp  ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed_bmp );
						}
							ultra_speed=LIMIT(-ultra_speed_bmp,-2.5*1000,2.5*1000);
				     if(!hold_alt_flag||mode.en_hinf_height_spd||mode.height_safe)
						 ultra_ctrl_out_use=EXP_Z_SPEED;
						 else
						 ultra_ctrl_out_use=ultra_ctrl_out; 
             static u16 cnt_rng;			
						 
//						 if(mode.en_ident1&&!mode.height_safe){//辨识用
//							if(cnt_rng++>20){
//							rng_time1=Get_Cycle_T(GET_T_RNG); 	
//							cnt_rng=0;
//							rng_thr=RNG_Get_RandomRange(-160,160);
//							ultra_ctrl_out_use=	2.7f *rng_thr;
//							}}
						 
					 if(ALT_POS_SONAR2<0.015&&thr_use<500&&!mode.height_safe)//智能起飞油门限制
						 ultra_ctrl_out_use=LIMIT(ultra_ctrl_out_use,0,1000);
						 height_speed_ctrl(in_timer_high,thr_use,LIMIT(ultra_ctrl_out_use,-1000,1000),ultra_speed);	//速度环 
					
			}//---end of speed control
			static u8 cnt_100ms;
			#if  defined(SONAR_SAMPLE1)
			hs_ctrl_cnt_max[0]=10;//50Hz  WT
			#endif
			if( cnt_100ms++>=(float)(H_OUTTER)/(t_h*1000))//PA
			{
	      out_timer_high=Get_Cycle_T(GET_T_HIGH_CONTROL_O);
				cnt_100ms=0;
				Ultra_Ctrl(out_timer_high,thr_use);//位置环
			}
			if(height_ctrl_mode_use)//定高模式
		{	
			  height_ctrl_out = wz_speed_pid_v.pid_out;
		}
		else//手动模式
		{		
		  	height_ctrl_out = thr-50*LIMIT(wz_acc_ukf/4096,-0.2,0.2)*9.8;   
		}		
		break; 
		default: break;		
	} //switch
}
#define MIN_THR_DOWN HOLD_THR-50
#define MIN_THR_RC_CHECK 200 //RC MIN_THR_CHECK
int Thr_down_min_portect(int thr_in,float T){//油门下降过快限制 未使用
static u8 state;
int out;
//state
	switch(state)	
	{
		case 0:if(!Thr_Low&&thr_in>MIN_THR_DOWN)//Check RC
						state=1;
			break;
		case 1:if(500 + CH_filter[THRr]<MIN_THR_RC_CHECK||!fly_ready)
						state=0;
			break;
		default:state=0; break;
	}
//output
	switch(state)	
	{
		case 0:out=thr_in;break;
		case 1:out=LIMIT(thr_in,MIN_THR_DOWN,HOLD_THR);break;
		default:out=thr_in;break;
	}
return out;
}	
///test 
float k_d=1;
float p1=0.4,p2=0.1;//0.35;//0.3;//WT
u8 speed_ctrl_sel=1,speed_spd_sel=1;
float wz_speed,wz_speed_old,wz_acc_mms2;
float height_thrv,wz_speed_pid_v_view,wz_speed_pid_v_view_eso;
float d_view;
float ero_view[2];
void height_speed_ctrl(float T,float thr,float exp_z_speed,float h_speed)//速度控制
{
static float thr_lpf;
float height_thr;
float wz_acc_mms21;
static float lpf_tmp,hc_speed_i,hc_speed_i_2,wz_speed_0,wz_speed_1,wz_speed_2,hc_acc_i;
	wz_acc_mms2 = (wz_acc_ukf/4096.0f) *9800;//-acc_bais*1000;//+ALT_BIAS_BMP*1000 ;

	if(!fly_ready)wz_speed_pid_v.err_i=0;
	height_thr = LIMIT( ALT_HOLD_THR_RANGE_SCALE * thr , 0, HOLD_THR );
	//height_thr = Thr_down_min_portect(height_thr,T);//add by gol 16.3.28 (WT)
	thr_lpf += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );
	height_thrv=thr_lpf;

	wz_speed=my_deathzoom_2( (h_speed) ,25);

  //-------------------------------------------PID-------------------------------
	wz_speed_pid_v.err = wz_speed_pid_use.kp *( exp_z_speed - wz_speed );
	wz_speed_pid_v.err_d = 0.002f/T *10*wz_speed_pid_use.kd * (-my_deathzoom( (LIMIT(wz_acc_mms2,-9800*0.8,9800*0.8) ) ,70)) *T;
	float d_view_temp=0.002f/T *10*wz_speed_pid_use.kd * (my_deathzoom( (LIMIT(-(wz_speed - wz_speed_pid_v.err_old)/T,-9800*0.8,9800*0.8) ) ,50)) *T;
	d_view=d_view*(1-k_d)+k_d*d_view_temp;//调试用
	
	wz_speed_pid_v.err_i += wz_speed_pid_use.ki *( exp_z_speed - h_speed ) *T;
	if(fabs(wz_speed_pid_v.err)<eso_att_inner_c[THRr].eso_dead||eso_att_inner_c[THRr].b0==0||mode.en_eso_h_in==0||mode.height_safe==1)
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *300,Thr_Weight *300);
	else
	wz_speed_pid_v.err_i=0;
	
	HIGH_CONTROL_SPD_ESO(&eso_att_inner_c[THRr],exp_z_speed,wz_speed,eso_att_inner_c[THRr].u,T,400);//速度环自抗扰控制
	
	if(mode.en_eso_h_in&&!mode.height_safe&&eso_att_inner_c[THRr].b0!=0){//ADRC
	wz_speed_pid_v.pid_out=thr_lpf + Thr_Weight *LIMIT(wz_speed_pid_use.kp*0 *LIMIT(exp_z_speed,-250,250)+eso_att_inner_c[THRr].u +wz_speed_pid_v.err_i+ wz_speed_pid_v.err_d ,-400,400);
	}
	else if(mode.en_hinf_height_spd&&!mode.height_safe){//鲁棒控制  未使用	
	//wz_speed_pid_v.pid_out=thr_lpf + Thr_Weight *LIMIT(wz_speed_pid_use.kp*0.35 *LIMIT(exp_z_speed,-250,250)+ h_inf_height_spd_out((float)exp_z_speed/1000,(float)wz_speed/1000) ,-400,400);
	}
  else{
	if(EXP_Z_SPEED!=0)
	wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT(( wz_speed_pid_use.kp*1 *LIMIT(exp_z_speed,-250,250)+wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-400,400);	
  else	
	wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT(( wz_speed_pid_use.kp*1  *LIMIT(exp_z_speed,-250,250)+wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-400,400);	
  }
	wz_speed_pid_v.err_old =wz_speed;// ( exp_z_speed - wz_speed ); 
}

u8 baro_ctrl_start;
float baro_height,baro_height_old,ultra_speed_bmp;
float ultra_sp_test[2];
float k_dh=1;
void Ultra_Ctrl(float T,float thr)//位置环PID
{
	float ultra_sp_tmp,ultra_dis_tmp;	
	#define MID_THR 500 //摇杆中位PWM
  static int bmp_expr;
	static float h_reg[2];
	static u16 cnt_in_mid;
	static u8 state_high_set=0;
	
	if(!mode.use_dji)
		exp_height_speed=EXP_Z_SPEED;
	else
	  exp_height_speed= ( 2.7f *my_deathzoom(CH_filter[THRr], 80 )) ;
	
	 #define CHECK_CNT_MID 0.25

  if(EXP_Z_SPEED!=0)
	{  hold_alt_flag=0;
	if(height_ctrl_mode_use==1)
		{
		#if EN_ATT_CAL_FC	
		exp_height=ALT_POS_BMP_UKF_OLDX*1000;
		#else
		exp_height=ALT_POS_BMP*1000;	
		#endif
	  }
		if(height_ctrl_mode_use==2)
		{exp_height=LIMIT(ALT_POS_SONAR2,0.04,10)*1000;}
	}
	else
		hold_alt_flag=1;
	
	
	if(mode_change){mode_change=0;
	if(height_ctrl_mode_use==1)
	{exp_height=ALT_POS_BMP_UKF_OLDX*1000;}
	if(height_ctrl_mode_use==2)
	{exp_height=ALT_POS_SONAR2*1000;}
	}
	static u8 mode_safe_reg;
	if(mode.height_safe&&!mode_safe_reg){mode_change=1;
	exp_height=ALT_POS_BMP_UKF_OLDX*1000;}
	mode_safe_reg=mode.height_safe;
	
		if(height_ctrl_mode_use==2&&!mode.height_safe&&ALT_POS_SONAR2>0.35&&mode.h_is_fix)//超声波模式期望高度固定为1.2m
			exp_height=1200;

	if(height_ctrl_mode_use==1||mode.height_safe)
	#if EN_ATT_CAL_FC	
	ultra_dis_tmp=ALT_POS_BMP_UKF_OLDX*1000;
	#else
	ultra_dis_tmp=ALT_POS_BMP*1000;	
	#endif
	else 
	ultra_dis_tmp=  ALT_POS_SONAR2*1000;

	ultra_dis_lpf=  ultra_dis_tmp;
		
	if(ultra_pid.ki==0||(mode.use_dji)||!fly_ready)ultra_ctrl.err_i=0;
	if(height_ctrl_mode_use==1||mode.height_safe)
	ultra_ctrl.err = ( ultra_pid_use.kp*LIMIT(my_deathzoom(exp_height - ultra_dis_lpf,15),-800/2,800/2) ); 
	else
	ultra_ctrl.err = ( ultra_pid_use.kp*LIMIT(my_deathzoom(exp_height - ultra_dis_lpf,10),-800/2,800/2) );

	ultra_ctrl.err_i += ultra_pid_use.ki *ultra_ctrl.err *T;
	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);
	ultra_ctrl.err_d = ultra_pid.kd *( 0.6f *(-wz_speed*T) + 0.4f *(ultra_ctrl.err - ultra_ctrl.err_old) );
	
//	if(mode.en_eso_h_in&&!mode.height_safe){//外环自抗扰 未使用
//	HIGH_CONTROL_SPD_ESO(&eso_att_inner_c[THRr],(exp_height),(ultra_dis_lpf),eso_att_inner_c[THRr].u,T,800);
//	ultra_ctrl.pid_out = eso_att_inner_c[THRr].u  + ultra_ctrl.err_d;	 
//	}

	ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;

	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-1000,1000);
	ultra_ctrl_out = ultra_ctrl.pid_out;
	ultra_ctrl.err_old = ultra_ctrl.err;
}





