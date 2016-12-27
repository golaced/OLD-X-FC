#include "../HARDWARE/CONTROL/visual_contol.h"
#include "../HARDWARE/DRIVER/usart_fc.h"
#include "../HARDWARE/FUSHION/alt_kf.h"
#include "../HARDWARE/CONTROL/sonar_avoid.h"
#include "../HARDWARE/include.h"
CIRCLE circle,track;
MARKER marker;
float nav_circle[2],nav_circle_last[2];
/*
        y add (0~256)  1
        |
        |


        -------- x decrease (0~320)  0


*/
float circle_check=0.01;
float circle_lfp=1;

void circle_control(float T)//对圆自动降落 未使用
{
}


#define NAV_ANGLE_MAX 0.8*MAX_CTRL_ANGLE
float nav[2];
float GPS_angle[2];
float  target_position[2];
float  now_position[2];
float actual_speed[2];
float tar_speed[2],tar_speed_avoidc[2];
float d_flow_watch[2];
float  integrator[2];
float k_break=0.85,k_flt_break=0.03;
u8 cnt_sb_sample_max=20;
float scale_flow_rc=0.05;
float r_circle=0.55;
float d_angle=0.4;
float circle_angle;
void GPS_calc_poshold(float T)//     光流定点 
{
}

float exp_center_cycle[5]={1.50,1.50,1,0,12};
void Nav_pos_set_test(u8 mode,float T)
{
switch(mode){
	case 0://cycle
		  exp_center_cycle[3]+=T*exp_center_cycle[4];
			if(exp_center_cycle[3]<0)exp_center_cycle[3]=360;
			if(exp_center_cycle[3]>360)exp_center_cycle[3]=0;
	        nav_pos_ctrl[X].exp=cos(exp_center_cycle[3]*0.0173)*exp_center_cycle[2]+exp_center_cycle[0];
					nav_pos_ctrl[Y].exp=sin(exp_center_cycle[3]*0.0173)*exp_center_cycle[2]-exp_center_cycle[1];
	break;

}
}


#define NAV_POS_INT        500//mm/s  
#define NAV_SPD_INT        300//mm/s
float yaw_qr_off;
float out_timer_nav,in_timer_nav;
float acc_temp[3];
_pos_pid nav_pos_pid;
_pos_pid nav_spd_pid;
_pos_control nav_pos_ctrl[2];
_pos_control nav_spd_ctrl[2];
void reset_nav_pos(u8 sel)
{
if(sel==Y)	
nav_pos_ctrl[Y].exp=POS_UKF_Y;//mm
if(sel==X)
nav_pos_ctrl[X].exp=POS_UKF_X;//mm
}
void Positon_control(float T)//     光流定点 
{
	u8 i;
	static u8 cnt[2],init;
	if(!init){init=1;
		nav_pos_ctrl[X].mode=2;
		nav_pos_pid.kp=0.2;
		nav_pos_pid.ki=0.0;
		nav_pos_pid.kd=0.0;
		nav_pos_pid.dead=0.02;
		
		
		nav_spd_pid.f_kp=0.2;
		nav_spd_pid.kp=0.2;
		nav_spd_pid.ki=0.01;
		nav_spd_pid.kd=0.05;
		nav_spd_pid.dead=20;
	}
	if(NS==0)
	Nav_pos_set_test(0,T);
	else {
	if(fabs(CH_filter[PITr])>50||ALT_POS_SONAR2<0.35||!fly_ready)
		 reset_nav_pos(Y);
	if(fabs(CH_filter[ROLr])>50||ALT_POS_SONAR2<0.35||!fly_ready)
		 reset_nav_pos(X);
	}
			/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	

	   
head  |    1 PIT y-   90d in marker
			| 
		   _____  0 ROL x+

		*/
  float pos[2];
	int spd[2],acc[2];
	float a_br[3];	
	static float acc_flt[2];
	
	a_br[0] =(float) mpu6050_fc.Acc.x/4096.;//16438.;
	a_br[1] =(float) mpu6050_fc.Acc.y/4096.;//16438.;
	a_br[2] =(float) mpu6050_fc.Acc.z/4096.;//16438.;

	acc_temp[0] = a_br[1]*reference_vr_imd_down[2]  - a_br[2]*reference_vr_imd_down[1] ;
	acc_temp[1] = a_br[2]*reference_vr_imd_down[0]  - a_br[0]*reference_vr_imd_down[2] ;
	acc_flt[0] += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom( (acc_temp[0] - acc_flt[0] ),0);
	acc_flt[1] += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom( (acc_temp[1] - acc_flt[1] ),0);
//输入数据	
	pos[Y]=POS_UKF_Y;//mm
  pos[X]=POS_UKF_X;//mm
	
	spd[Y]=VEL_UKF_Y*1000;//mm
  spd[X]=VEL_UKF_X*1000;//mm
	acc[Y]=acc_flt[1]*9800;//mm
  acc[X]=acc_flt[0]*9800;//mm
//位置
	if(cnt[0]++>1){cnt[0]=0;
	out_timer_nav=Get_Cycle_T(GET_T_OUT_NAV);
	for (i=0;i<2;i++){ 
		nav_pos_ctrl[i].now=pos[i];
		if(nav_pos_pid.ki==0||!fly_ready)nav_pos_ctrl[i].err_i=0;
		nav_pos_ctrl[i].err = ( nav_pos_pid.kp*LIMIT(my_deathzoom(nav_pos_ctrl[i].exp - nav_pos_ctrl[i].now,nav_pos_pid.dead),-3,3) )*1000;

		nav_pos_ctrl[i].err_i += nav_pos_pid.ki *nav_pos_ctrl[i].err *out_timer_nav;
		nav_pos_ctrl[i].err_i = LIMIT(nav_pos_ctrl[i].err_i,-Thr_Weight *NAV_POS_INT,Thr_Weight *NAV_POS_INT);
		nav_pos_ctrl[i].err_d =  nav_pos_pid.kd *( 0.6f *(-(float)spd[i]*out_timer_nav) + 0.4f *(nav_pos_ctrl[i].err - nav_pos_ctrl[i].err_old) );

		nav_pos_ctrl[i].pid_out = nav_pos_ctrl[i].err +nav_pos_ctrl[i].err_i + nav_pos_ctrl[i].err_d;
		nav_pos_ctrl[i].pid_out = LIMIT(nav_pos_ctrl[i].pid_out,-5*1000,5*1000);//m/s
		nav_pos_ctrl[i].err_old = nav_pos_ctrl[i].err;
		}
	}
	float Yaw_qr=To_180_degrees(Yaw+yaw_qr_off);
	if(nav_pos_ctrl[X].mode==2){//global  Yaw from IMU
	nav_spd_ctrl[Y].exp= nav_pos_ctrl[North].pid_out*cos(Yaw_qr*0.0173)+nav_pos_ctrl[East].pid_out*sin(Yaw_qr*0.0173); 
	nav_spd_ctrl[X].exp=-nav_pos_ctrl[North].pid_out*sin(Yaw_qr*0.0173)+nav_pos_ctrl[East].pid_out*cos(Yaw_qr*0.0173);
	}
	else
	{
	nav_spd_ctrl[Y].exp=nav_pos_ctrl[Y].pid_out;
	nav_spd_ctrl[X].exp=nav_pos_ctrl[X].pid_out;	
	}		
	
	static u8 state_tune_spd;
	static u8 flag_way;
	static u16 cnt_s1;
	switch(state_tune_spd){
	case 0:	
	if(mode.trig_flow_spd)
	{state_tune_spd=1;cnt_s1=0;flag_way=!flag_way;}
	break;
	case 1:
	if(mode.trig_flow_spd)
	{	if(flag_way)
	nav_spd_ctrl[X].exp=300;
	else
	nav_spd_ctrl[X].exp=-300;		
	}
	else
	state_tune_spd=0;	
	if(cnt_s1++>3/T)
	{cnt_s1=0;state_tune_spd=2;}
	break;
	case 2:
	nav_spd_ctrl[X].exp=0;			
	if(cnt_s1++>1.5/T)	
	state_tune_spd=0;
	if(!mode.trig_flow_spd)
	state_tune_spd=0;
	break;
	}
//速度环	
	in_timer_nav=Get_Cycle_T(GET_T_IN_NAV);
	for (i=0;i<2;i++){	
	nav_spd_ctrl[i].now=spd[i];
	nav_spd_ctrl[i].err = nav_spd_pid.kp *LIMIT(my_deathzoom( nav_spd_ctrl[i].exp - nav_spd_ctrl[i].now,nav_spd_pid.dead),-3000,3000);
	nav_spd_ctrl[i].err_d = 0.002f/T *10*nav_spd_pid.kd * my_deathzoom(-acc[i] ,50) *in_timer_nav;
	nav_spd_ctrl[i].err_i += nav_spd_pid.ki *nav_spd_ctrl[i].err *in_timer_nav;
	nav_spd_ctrl[i].err_i = LIMIT(nav_spd_ctrl[i].err_i,-Thr_Weight *NAV_SPD_INT,Thr_Weight *NAV_SPD_INT);
	//HIGH_CONTROL_SPD_ESO(&eso_att_inner_c[THRr],exp_z_speed,wz_speed,eso_att_inner_c[THRr].u,T,400);//速度环自抗扰控制
	nav_spd_ctrl[i].pid_out =LIMIT(( nav_spd_pid.f_kp*LIMIT(nav_spd_ctrl[i].exp,-100,100)+nav_spd_ctrl[i].err + nav_spd_ctrl[i].err_d + nav_spd_ctrl[i].err_i),-250,250)/10;	
	nav_spd_ctrl[i].err_old =nav_spd_ctrl[i].err;
  }
	
	nav[PITr]=nav_spd_ctrl[Y].pid_out;
	nav[ROLr]=nav_spd_ctrl[X].pid_out;

}


//--------------------------------------自动起飞降落 视觉导航状态机 未使用
u8 mode_change;
u16 AUTO_UP_CUARVE[]={1600,1660,1660,1655,1650,1650,1650,1650,1650};
u16 AUTO_DOWN_CUARVE[]={1500,1500-50,1500-150,1500-150,1500-200,1500-200};
u16 AUTO_DOWN_CUARVE1[]={1500-150,1500-150,1500-100,1500-100,1500-80,1500-80};
float SONAR_SET_HIGHT =0.06;
float AUTO_FLY_HEIGHT =2.5;
float SONAR_CHECK_DEAD =0.05;

float AUTO_LAND_HEIGHT_1= 2.5;// 3.5 //bmp check
float AUTO_LAND_HEIGHT_2= 1.6;//1.8
float AUTO_LAND_HEIGHT_3= 0.0925;

float MINE_LAND_HIGH= 0.35;
float AUTO_LAND_SPEED_DEAD =0.08;
u8 state_v;u16 cnt[10]={0};
float baro_ground_high;
float nav_land[3];
#define DEAD_NAV_RC 80
//state
#define SG_LOW_CHECK 0
#define SG_MID_CHECK 1
#define SU_UP1 2
#define SU_HOLD 3
#define SD_RETRY_UP 4
#define SD_RETRY_UP_HOLD 5


#define SD_HOLD 13
#define SD_MISS_SEARCH 14
#define SD_HOLD2 15
#define SD_HIGH_FAST_DOWN 16
#define SD_CIRCLE_SLOW_DOWN 17
#define SD_CIRCLE_HOLD 18
#define SD_CIRCLE_MID_DOWN  19
#define SD_CHECK_G 20
#define SD_SHUT_DOWN 21
#define SD_SAFE 22

#define RC_PITCH 0
#define RC_ROLL  1
#define RC_THR   2
#define RC_YAW   3
u16 Rc_Pwm_Inr_mine[8],Rc_Pwm_Out_mine[8];
float angle_imu_dj[3];
void AUTO_LAND_FLYUP(float T)
{
}
