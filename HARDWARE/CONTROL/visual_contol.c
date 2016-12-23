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
{static u8 state;
 u8 i;
 static int circle_reg[2],circle_use[2];
 float p,d,intit,rate_error[2],derivative[2];
 static float last_error[2],last_derivative[2];
 float  distance_use;
 static u16 cnt[3];	
 float out[2];
//state	
	switch(state){
		case 0:if(circle.check&&circle.connect)
			       cnt[0]++;
		       else
						 cnt[0]=0;
		       if(cnt[0]>circle_check/T)
					 {state=1;cnt[0]=0;}
		  break;
		case 1:
			     if(!circle.check||!circle.connect)
						 state=0;
					 
		      break;
		default:state=0;break;
	}
//output	
	switch(state){
		case 0:circle_use[0]=MID_X;circle_use[1]=MID_Y;break;
		case 1:circle_use[0]=circle.x;circle_use[1]=circle.y;break;
		default:circle_use[0]=MID_X;circle_use[1]=MID_Y;break;
	}
	 //if(circle.check&&circle.connect){
	 circle_use[0]=circle.x_flp;circle_use[1]=circle.y_flp; //}
//	 else{
//	 circle_use[0]=MID_X;circle_use[1]=MID_Y;}
//	if(ALT_POS_SONAR2<0.15)
//		 distance_use=0.8;
//	else
//		distance_use=ALT_POS_SONAR2;
		//circle_use[0]=circle.x_flp-160;
	   circle_use[0]-=MID_X;
		//circle_use[1]=circle.y_flp-128;
	   circle_use[1]-=MID_Y;
		rate_error[0]=circle_use[0]*distance_use;
		rate_error[1]=circle_use[1]*distance_use;
	static float integrator_circle[2];
 if(pid.circle.in.i==0)
 {
 integrator_circle[0]=integrator_circle[1]=0;
 }
	 for(i=0;i<=1;i++){
			p = rate_error[i]*pid.circle.in.p;
			derivative[i] = (rate_error[i] - last_error[i]) ;/// DT;
			derivative[i]=0.3* derivative[i]+ 0.7*last_derivative[i];
			// update state
			last_error[i] = rate_error[i] ;
			last_derivative[i]= derivative[i];
			// add in derivative component
			d = derivative[i]*pid.circle.in.d;//d
			  integrator_circle[i] += ((float) rate_error[i]* pid.circle.in.i);// *DT;
        intit = LIMIT(integrator_circle[i],-10,10) ;
			//nav_circle[i] = p + d;
  		out[i] =circle_lfp*(p+d+intit)+(1-circle_lfp)*nav_circle_last[i];
		  nav_circle_last[i]=out[i]; 
		  //nav_circle[i]=out[i];
		  //nav_circle[i] += 0. *T *3.14f * ( -nav_circle[i] + out[i] );
 	 }
	 
	 
	 int flag[2];
		 
	 for(i=0;i<2;i++)
   {
	 if(circle_use[i]>0)
	 flag[i]=1;
	 else if(circle_use[i]<0)
	 flag[i]=-1;
	 else
	 flag[i]=0; 
	 }	 
	 if(SPID.YP==1) //2
	 flag[0]=1;//nav_circle[0]=flag[0]*(float)SPID.YD/10;
	 else  if(SPID.YP==2)
	 flag[0]=-1;//*(float)SPID.YD/10;
	 else
		flag[0]=0; 
	 nav_circle[0]=LIMIT(flag[0]*circle_use[0]*pid.circle.in.p,-10,10);
 
	 if(SPID.YI==1) //2
	 flag[1]=1;//nav_circle[1]=flag[1]*(float)SPID.YD/10;
	 else  if(SPID.YI==2)
	 flag[1]=-1;//nav_circle[1]=-flag[1]*(float)SPID.YD/10;
	 else
		flag[1]=0; 
	 nav_circle[1]=LIMIT(flag[1]*circle_use[1]*pid.circle.in.p,-10,10);
	 
	 if(!circle.check)
	 nav_circle[0]=nav_circle[1]=0;
// nav_circle[0] =Moving_Median(18,10,out[0]);
// nav_circle[1] =Moving_Median(19,10,out[1]);	 
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
	float p, i, d,f_p;
	float output;
	float target_speed;
	static 	float last_derivative[2];
	float derivative[2];
	static int32_t last_error[2];
	int axis;
	float error_pos[2], rate_error[2];
	float cos_yaw,sin_yaw;	
	static u8  state[2]; 	
			/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	

	   
head  |    1 PIT y-   90d in marker
			| 
		   _____  0 ROL x+
			 
			 
	
		*/
		if(pid.nav.in.i==0)
		{integrator[1]=integrator[0]=0;}
	if(mode.flow_hold_position_use_global){	
		 actual_speed[LON]=my_deathzoom_2(imu_nav.flow.speed.east,pid.nav.in.dead)*10;//imu_nav.flow.speed.x;//mm
	   actual_speed[LAT]=my_deathzoom_2(imu_nav.flow.speed.west,pid.nav.in.dead)*10;//imu_nav.flow.speed.y;
		 //now_position[LON]=imu_nav.flow.position.east;
	   //now_position[LAT]=imu_nav.flow.position.west;
	}
	else
	{ if(!mode.flow_sel){
		 if(mode.flow_f_use_ukfm){
		 actual_speed[LON]=my_deathzoom_2(VEL_UKF_Y,pid.nav.in.dead)*10;//imu_nav.flow.speed.x;//mm
	   actual_speed[LAT]=my_deathzoom_2(VEL_UKF_X,pid.nav.in.dead)*10;//imu_nav.flow.speed.y; 
		 }
		 else{
   	 actual_speed[LON]=my_deathzoom_2(imu_nav.flow.speed.y_f,pid.nav.in.dead)*10;//imu_nav.flow.speed.x;//mm
	   actual_speed[LAT]=my_deathzoom_2(imu_nav.flow.speed.x_f,pid.nav.in.dead)*10;//imu_nav.flow.speed.y;
		 }
		//if(ultra_dis_lpf>650)
		//	{
		 //pid.nav.in.dead2=10*pid.nav.in.dead;
	  // now_position[LON]+=my_deathzoom_2(imu_nav.flow.speed.y_f,pid.nav.in.dead2)*T;
	   //now_position[LAT]+=my_deathzoom_2(imu_nav.flow.speed.x_f,pid.nav.in.dead2)*T;
	}
	else
	{
	   actual_speed[LON]=my_deathzoom_2(imu_nav.flow.speed.y,pid.nav.in.dead)*10;//imu_nav.flow.speed.x;//mm
	   actual_speed[LAT]=my_deathzoom_2(imu_nav.flow.speed.x,pid.nav.in.dead)*10;//imu_nav.flow.speed.y;
		//if(ultra_dis_lpf>650)
		//	{
		 //pid.nav.in.dead2=10*pid.nav.in.dead;
	   //now_position[LON]+=my_deathzoom_2(imu_nav.flow.speed.east,pid.nav.in.dead2)*T;
	   //now_position[LAT]+=my_deathzoom_2(imu_nav.flow.speed.west,pid.nav.in.dead2)*T;
	}
		//}
		 //pid.nav.in.p=0.55  
	}
	
	
	
	//--------------marker hold  position
	/*head  |  vy  + 1   90d in marker LAT
	    		| 
		       _____   vx  +0            LON
			 
			 */

	marker.ero_m[0]=my_deathzoom_2((marker.pos_set[0]-marker.pos_now[0]),pid.marker.out.dead)*pid.marker.out.p;
	marker.ero_m[1]=-my_deathzoom_2((marker.pos_set[1]-marker.pos_now[1]),pid.marker.out.dead)*pid.marker.out.p;
	static float Yaw_marker=0;
	
	if(marker.check&&marker.connect)
	marker.Yaw_marker=To_180_degrees(marker.angle[2]-90);
	marker.target_speed_m[0]=LIMIT(-sin(marker.Yaw_marker*0.017)*marker.ero_m[1]+cos(marker.Yaw_marker*0.017)*marker.ero_m[0],-20.5,20.5);
	marker.target_speed_m[1]=LIMIT( cos(marker.Yaw_marker*0.017)*marker.ero_m[1]+sin(marker.Yaw_marker*0.017)*marker.ero_m[0],-20.5,20.5);
	
	//---------------flow hold position
	float now_position_temp[2];  
	now_position_temp[0]=now_position[0];
	now_position_temp[1]=now_position[1];
    for (axis = 0; axis < 2; axis++) {
			//loc p
			 if(mode.en_marker&&marker.connect&&marker.check)
			 {integrator[0]=integrator[1]=0;
				target_position[LON]=now_position[LON];//m
				target_position[LAT]=now_position[LAT];//m
			 }
			 
			static float init_pos[2];
	
			static u8 mode_circle_reg;
			 
			 if(mode.rc_control_flow_pos_sel&&!mode_circle_reg)
			 {init_pos[LAT]=now_position[LAT];
				init_pos[LON]=now_position[LON]; 
				 circle_angle=0;
			 }
			 mode_circle_reg=mode.rc_control_flow_pos_sel;
			 if(mode.rc_control_flow_pos)
			 {
				switch(mode.rc_control_flow_pos_sel)
					{case 0:					
			 target_position[LAT]+=  CH_filter[ROLr]*scale_flow_rc*T/40.;
			 target_position[LON]+= -CH_filter[PITr]*scale_flow_rc*T/40.;
						break;
					case 1:	//circle
					//d_angle=pid.nav.out.d;
					circle_angle+=d_angle;
					if(circle_angle>360)
						circle_angle=0;
					
					target_position[LAT]=(init_pos[LAT]-r_circle)+cos(circle_angle*0.017)*r_circle;
					target_position[LON]=(init_pos[LON])-sin(circle_angle*0.017)*r_circle;
					break;}
			 }
			 
			error_pos[axis]=LIMIT(my_deathzoom_2(target_position[axis]-now_position[axis],pid.nav.out.dead),-15,15);//外环位置误差
			 		
			if(marker.check&&marker.connect&&mode.en_marker)
			switch(axis){
			case 0:
			tar_speed[0]=marker.target_speed_m[0];  break;
			case 1:
			tar_speed[1]=marker.target_speed_m[1]; break;
			}
			else if(mode.rc_control_flow_spd){
			switch(axis){
			case 0:
			tar_speed[0]=CH_filter[PITr]*scale_flow_rc;  break;
			case 1:
			tar_speed[1]=-CH_filter[ROLr]*scale_flow_rc; break;}
			}
			else if(mode.en_sonar_avoid&&need_avoid&&ALT_POS_SONAR2>0.35&&mode.flow_hold_position==2){
			switch(axis){
			case 0:
			tar_speed[0]=avoid_trace[1]*pid.avoid.out.p;  break;
			case 1:
			tar_speed[1]=avoid_trace[0]*pid.avoid.out.p; break;}
			}
			else {					
			switch(axis){
			case 0:
			tar_speed[0]=-error_pos[0]*pid.nav.out.p;  break;
			case 1:
			tar_speed[1]=-error_pos[1]*pid.nav.out.p; break;
			}
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
					tar_speed[0]=6;
						else
					tar_speed[0]=-6;		
					}
				  if(cnt_s1++>3/T)
					{cnt_s1=0;state_tune_spd=2;}
				break;
				case 2:
					if(!mode.trig_flow_spd)
						state_tune_spd=0;
				break;
			}
			
			switch(axis){//内环速度误差
				case 0:
				 rate_error[0] = tar_speed[0] - actual_speed[0];   // calc the speed error
				case 1:
				 rate_error[1] = tar_speed[1] - actual_speed[1];   // calc the speed error
			}
       //-----------rad
        p = rate_error[axis]*pid.nav.in.p;
			 	 
			 static float reg_ero_pos[2];
				 
			  //f_p = (error_pos[axis]-reg_ero_pos[axis])*pid.nav.out.i;
			  reg_ero_pos[axis]= error_pos[axis];
	     if((fabs(CH_filter[ROLr])<100)&&(fabs(CH_filter[PITr])<100)&&fabs(Pitch)<10&&fabs(Roll)<10)
			  integrator[axis] += ((float) rate_error[axis]* pid.nav.in.i);// *DT;

        i = LIMIT(integrator[axis],-MAX_FIX_ANGLE,MAX_FIX_ANGLE) ;
	
				derivative[axis] = (rate_error[axis] - last_error[axis]) ;/// DT;
				derivative[axis]=0.3* derivative[axis]+ 0.7*last_derivative[axis];

				last_error[axis] = rate_error[axis] ;
				last_derivative[axis]= derivative[axis];
    
			  d_flow_watch[axis]= derivative[axis];//调试用
				if( mode.flow_d_acc)//内环微分使用加速度还是直接对速度微分
				d = acc_body[axis]*pid.nav.in.d;//d
				else			
				d = derivative[axis]*pid.nav.in.d;//d

				f_p = tar_speed[axis]*pid.nav.in.fp;//前馈控制
        output =f_p+ p + i + d + tar_speed[axis]*pid.nav.out.d;

        GPS_angle[axis] = limit_mine(output,NAV_ANGLE_MAX);//
     
    }
	float out_temp[2];
		if(mode.flow_hold_position_use_global){//使用无头模式
		cos_yaw=cos(Yaw*0.017);
		sin_yaw=sin(Yaw*0.017);
			out_temp[PITr] = -(GPS_angle[LAT] * cos_yaw + GPS_angle[LON] * sin_yaw);// / 10;
			out_temp[ROLr] = +(GPS_angle[LON] * cos_yaw - GPS_angle[LAT] * sin_yaw);/// 10;
		}
		else{
			out_temp[PITr] = -GPS_angle[LAT];
			out_temp[ROLr] = +GPS_angle[LON];
		}
		

		
		 limit_mine( (out_temp[PITr]) ,NAV_ANGLE_MAX );
		 limit_mine( (out_temp[ROLr]) ,NAV_ANGLE_MAX );
		
//------------光流刹车平滑		#define ROLr 0  #define PITr 1
static float angle_temp[2];		
static float sb[2],sbr[2];
		sb[PITr]=my_deathzoom( (CH_filter[ROLr]) ,30 );
		sb[ROLr]=my_deathzoom( (CH_filter[PITr]) ,30 );
		static u8 cnt_sb_sample;
		if(cnt_sb_sample++>cnt_sb_sample_max)
		{
		cnt_sb_sample=0;
			sbr[PITr]=except_A_SB[PITr] ;
			sbr[ROLr]=except_A_SB[ROLr] ;
		}
	//PITr=1
		switch(state[PITr]){
		case 0:
			if(fabs(sb[PITr])>0&&mode.flow_hold_position>0)
				state[PITr]=1;
		break;
    case 1:
			if(fabs( sb[PITr]) <30 &&mode.flow_hold_position>0)
			{state[PITr]=2;angle_temp[PITr]=LIMIT(sbr[PITr]*k_break,-NAV_ANGLE_MAX,NAV_ANGLE_MAX);}
      
			if(!mode.flow_hold_position)state[PITr]=0;
    break;
		case 2:
			if(fabs( sb[PITr]) <30 &&mode.flow_hold_position>0)
			{
			angle_temp[PITr]-=angle_temp[PITr]*k_flt_break;
			
			if(fabs(angle_temp[PITr])<0.5)
				state[PITr]=0;
			}
			else state[PITr]=0;
		break;	
  }		
		
	switch(state[ROLr]){
		case 0:
			if(fabs(sb[ROLr])>0&&mode.flow_hold_position>0)
				state[ROLr]=1;
		break;
    case 1:
			if(fabs( sb[ROLr]) <30 &&mode.flow_hold_position>0)
			{state[ROLr]=2;angle_temp[ROLr]=LIMIT(-sbr[ROLr]*k_break,-NAV_ANGLE_MAX,NAV_ANGLE_MAX);}
      
			if(!mode.flow_hold_position)state[ROLr]=0;
    break;
		case 2:
			if(fabs( sb[ROLr]) <30 &&mode.flow_hold_position>0)
			{
			angle_temp[ROLr]-=angle_temp[ROLr]*k_flt_break;
			
			if(fabs(angle_temp[ROLr])<0.5)
				state[ROLr]=0;
			}
			else state[ROLr]=0;
		break;	
  }		
	
	
	if(mode.en_break){
//out--put	
		switch(state[PITr]){
		case 0:
				nav[PITr]=out_temp[PITr];
		break;
    case 1:
			  nav[PITr]=out_temp[PITr];
    break;
		case 2:
				nav[PITr]=angle_temp[PITr];
		break;	
  }		
//out--put	
		switch(state[ROLr]){
		case 0:
				nav[ROLr]=out_temp[ROLr];
		break;
    case 1:
			  nav[ROLr]=out_temp[ROLr];
    break;
		case 2:
				nav[ROLr]=angle_temp[ROLr];
		break;	
  }		
}
	else
	{
	nav[ROLr]=out_temp[ROLr];
	nav[PITr]=out_temp[PITr];
	}
		/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	   
head  |    1 PIT y-  RC0
			| 
		   _____  0 ROL x+   RC 1
	
		*/
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
{static u8 state,thr_sel[3],init,cnt_retry;
 static float sonar_r,bmp_r,bmp_thr;
	static u8 cnt_circle_check=0,init_circle_search=0;
	if(!init&&ALT_POS_BMP!=0){init=1;
		baro_ground_high=ALT_POS_BMP;
	}

	Rc_Pwm_Inr_mine[0]=(vs16)(CH_filter[PITr]+1500);//ultra_distance;
	Rc_Pwm_Inr_mine[1]=(vs16)(CH_filter[ROLr]+1500);//ultra_distance;
	Rc_Pwm_Inr_mine[2]=(vs16)(my_deathzoom(CH_filter[THRr],50)+1500);//ultra_distance;
	Rc_Pwm_Inr_mine[3]=(vs16)Rc_Get.YAW;
	if(Rc_Pwm_Inr_mine[RC_THR]<200+1000)
	{sonar_r=SONAR_SET_HIGHT+0.05;bmp_r=ALT_POS_BMP;}
//state_change 
	switch(state)
	{
		case SG_LOW_CHECK://low thr check
			mode.auto_land=0;	cnt_retry=0;
			if(mode.auto_fly_up==1&&mode.auto_land==0&&mode.en_visual_control==1&&ALT_POS_SONAR2<SONAR_SET_HIGHT+SONAR_CHECK_DEAD&&fabs(Pitch)<10&&fabs(Roll)<10){
					if(Rc_Pwm_Inr_mine[RC_THR]<200+1000)
					{state=SG_MID_CHECK;cnt[0]=0;mode_change=1;}//to auto fly
				}
			else if(mode.auto_fly_up==0&&mode.auto_land==0&&mode.en_visual_control==1&&ALT_POS_SONAR2>MINE_LAND_HIGH&&fabs(Pitch)<15&&fabs(Roll)<15){
					if((Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<600+1000))
					{state=SD_HOLD;cnt[0]=0;mode_change=1;}//to hold || land			
				}
			break;
		case SG_MID_CHECK://middle thr check
			if(mode.auto_fly_up==1&&mode.auto_land==0&&ALT_POS_SONAR2<SONAR_SET_HIGHT+0.05&&fabs(Pitch)<15&&fabs(Roll)<15){
					if((Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<600+1000))
						cnt[0]++;
					else cnt[0]=0;
					
					if(cnt[0]>2.5/T)
					{state=SU_UP1;thr_sel[0]=cnt[0]=cnt[1]=0;mode_change=1;}
				}//  
			else if(Rc_Pwm_Inr_mine[RC_THR]<200+1000||mode.auto_fly_up==0)
			{state=SG_LOW_CHECK;mode_change=1;}
			
			if(mode.en_visual_control==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;mode_change=1;}}
			break;
		case SU_UP1://take off
		if(mode.auto_fly_up==1&&mode.auto_land==0&&(Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<600+1000)){
				 if(cnt[0]++>5/T||ALT_POS_BMP-bmp_r>AUTO_FLY_HEIGHT)
				 {mode_change=1;state=SU_HOLD;}	 
	   }
		 else
		 {mode_change=1;state=SU_HOLD;} 
		 
		 if(ALT_POS_SONAR2>MINE_LAND_HIGH&&mode.auto_land==1&&mode.auto_fly_up==1)
		 {cnt[0]=0; state=SD_HOLD;mode_change=1;}//to land	  
		 
		 if(mode.en_visual_control==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
		 break;
		case SU_HOLD://keep high
			cnt_retry=0;
			if(mode.auto_fly_up==1&&mode.auto_land==1&&ALT_POS_SONAR2>MINE_LAND_HIGH&&mode.en_visual_control==1)
				 {cnt[0]=0;state=SD_HOLD;cnt[4]=0;mode_change=1;}//to land
			if(Rc_Pwm_Inr_mine[RC_THR]<200+1000&&ALT_POS_SONAR2<SONAR_SET_HIGHT+0.05&&mode.auto_fly_up==0&&mode.auto_land==0)
			 if(cnt[0]++>2.5/T)
				 {mode_change=1;state=SG_LOW_CHECK;cnt[0]=0;}	 	
			   
			
		break;
		 case SD_RETRY_UP://复飞
		if(mode.auto_fly_up==1&&mode.auto_land==1&&(Rc_Pwm_Inr_mine[RC_THR]>425+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<575+1000)){
				 if(cnt[0]++>5/T||ALT_POS_SONAR2>AUTO_LAND_HEIGHT_2*1.75)
				 {mode_change=1;
				 if(mode.circle_miss_fly)
				 { state=SD_MISS_SEARCH;init_circle_search=1;}
				 else
				 state=SD_RETRY_UP_HOLD;}	 
	   }
		 else
		 {mode_change=1; 
			 if(mode.circle_miss_fly)
			 {state=SD_MISS_SEARCH; init_circle_search=1;}
				 else
				 state=SD_RETRY_UP_HOLD;} 
		 		 
		 if(mode.en_visual_control==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
		 break;
		 case SD_RETRY_UP_HOLD:
    	if(mode.auto_fly_up==1&&mode.auto_land==1&&(Rc_Pwm_Inr_mine[RC_THR]>425+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<575+1000)){//中位开始下降 其他可以控制
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH){
					 if(cnt[4]++>3/T)
				 {state=SD_CIRCLE_SLOW_DOWN;thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;}} 
				 else
					cnt[4]=0; 		   
			 }
				 else
					cnt[0]=0; 
				 if(mode.en_visual_control==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
			break;
		//---------------------------MISS_CIRCLE------------
    case SD_MISS_SEARCH:	
     if(mode.auto_fly_up==1&&mode.auto_land==1&&(Rc_Pwm_Inr_mine[RC_THR]>425+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<575+1000)){//中位开始下降 其他可以控制
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH){
					 if(cnt[4]++>3.5/T||cnt[1]>8)
				 {state=SD_CIRCLE_SLOW_DOWN;thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;}} 
				 else
					cnt[4]=0; 		   
			 }else {state=SD_RETRY_UP_HOLD;thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;} 
				 
			   if(circle.check&&fabs(circle.x-160)<80&&fabs(circle.y-120)<80)
					 cnt[1]++;
				 		   
				 if(mode.en_visual_control==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
			break;
    break;		
		//-----------------------------------------------------------------------------------------------	 
				 
				 
		//-----------------------------------------auto_land---------------------------------------------
    case SD_HOLD://keep hight for tracking
			
		cnt_retry=0;
		     if(circle.check&&circle.connect)
					 cnt_circle_check++;
				 else
					 cnt_circle_check=0;
    	if(((Pitch_DJ>=track.forward_end_dj_pwm&&track.check&&circle.connect)||cnt_circle_check>20)&&
				fabs(Rc_Pwm_Inr_mine[RC_PITCH]-1500)<DEAD_NAV_RC&&fabs(Rc_Pwm_Inr_mine[RC_ROLL]-1500)<DEAD_NAV_RC&&fabs(Rc_Pwm_Inr_mine[RC_YAW]-1500)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15)
					 {
					 if(cnt[4]++>1/T)
				 {state=SD_HOLD2;thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;}} 
				 else
					cnt[4]=0; 
			   			 }
				 else
					cnt[0]=0; 
				 if(mode.en_visual_control==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
					break; 
    case SD_HOLD2://keep hight for circle
			//if(mode.auto_fly_up==1)
			//	mode.auto_land=1;
		cnt_retry=0;
    	if(mode.auto_fly_up==1&&mode.auto_land==1&&(Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)){//中位开始下降 其他可以控制
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH){
					 if(cnt[4]++>3.5/T)
				 {state=SD_HIGH_FAST_DOWN;thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;}} 
				 else
					cnt[4]=0; 
			   	}
			 else if((fabs(Rc_Pwm_Inr_mine[RC_PITCH]-1500)>DEAD_NAV_RC||fabs(Rc_Pwm_Inr_mine[RC_ROLL]-1500)>DEAD_NAV_RC)||mode.dj_by_hand)
				  {state=SD_HOLD;thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;}//拉后飞行器
			 else
					cnt[0]=0; 
			if(mode.en_visual_control==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
			
		break;
		case SD_HIGH_FAST_DOWN://load down curve 1  高空下降到对圆高度
			if(mode.auto_fly_up==1&&mode.auto_land==1&&(Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)){
				 if(fabs(ALT_POS_BMP-bmp_r)<AUTO_LAND_HEIGHT_1)//BMP check
				  {
					if(cnt[4]++>0.5/T)
					{state=SD_CIRCLE_SLOW_DOWN;cnt[1]=cnt[0]=0;cnt[4]=0;}
					}
					else
						cnt[4]=0;
				}
				else
					{cnt[0]=0; state=SD_HOLD;mode_change=1;}
			 
			 if(mode.en_visual_control==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}//restart until land
		break;
		case SD_CIRCLE_SLOW_DOWN://load down curve 2  对圆与缓慢下降
			if(mode.auto_fly_up==1&&mode.auto_land==1&&(Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)){
				 if(ALT_POS_SONAR2<AUTO_LAND_HEIGHT_2)//Sonar check  2m
				 {if(cnt[4]++>1/T)
					{state=SD_CIRCLE_HOLD;cnt[1]=cnt[0]=0;cnt[4]=cnt[5]=0;}
					}
				 else cnt[4]=0;
				}
				else
					{cnt[0]=0; state=SD_HOLD;mode_change=1;}
			 
			 if(mode.en_visual_control==0){if(cnt[3]++>3/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}//restart until land
				break;
   	case SD_CIRCLE_HOLD://     悬停对圆
			if(mode.auto_fly_up==1&&mode.auto_land==1&&(Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)){
				 if(mode.en_circle_locate==1&&mode.en_circle_nav)//
				 {if(cnt[4]++>2.125/T)
					{state=SD_CIRCLE_MID_DOWN;cnt[1]=cnt[0]=0;cnt[4]=cnt[5]=0;}
					}
				 else {cnt[4]=0;cnt[5]++;}
				}
				else
					{cnt[0]=0; state=SD_HOLD;mode_change=1;}
			 
					if(cnt[5]>6.6/T)//复飞
					{cnt[1]=cnt[0]=cnt[3]=cnt[4]=cnt[5]=0;state=SD_RETRY_UP;cnt_retry++;}
					if(cnt_retry>1)
					{state=SD_CIRCLE_MID_DOWN;cnt[1]=cnt[0]=cnt[4]=cnt[5]=cnt_retry=0;}	
			 if(mode.en_visual_control==0){if(cnt[3]++>3/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}//restart until land
				break;
			 			 
		case SD_CIRCLE_MID_DOWN://   在圆死区内中速下降
			if(mode.auto_fly_up==1&&mode.auto_land==1&&(Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)){
				 if(ALT_POS_SONAR2<AUTO_LAND_HEIGHT_3&&fabs(ALT_POS_BMP-bmp_r)<0.866)//Sonar check 0.5m
				 {if(cnt[4]++>1/T)
					{state=SD_CHECK_G;cnt[1]=cnt[0]=0;cnt[4]=0;}
					}
				}
				else
					{cnt[0]=0; state=SD_HOLD;mode_change=1;}
			 
			 if(mode.en_visual_control==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}//restart until land
				break;
		case SD_CHECK_G://shut motor  电机停转检测
			if(fabs(ALT_VEL_SONAR)<AUTO_LAND_SPEED_DEAD&&ALT_POS_SONAR2<AUTO_LAND_HEIGHT_3)
				cnt[2]++;
			else
				cnt[2]=0;
			if(cnt[2]>1.25/T)
				state=SD_SHUT_DOWN;
			
			if(mode.en_visual_control==0){mode_change=1;state=SD_SAFE;mode_change=1;}
		break;
		case SD_SHUT_DOWN://reset
    if(mode.auto_fly_up==0&&mode.auto_land==0&&(Rc_Pwm_Inr_mine[RC_THR]<200+1000)&&ALT_POS_SONAR2<SONAR_SET_HIGHT+0.05)
		{state=SG_LOW_CHECK;cnt_retry=0;}
		
		break;
		//------------------------------------SAFE------------------------------------------------
		case SD_SAFE://safe out
			cnt_retry=0;
		if(mode.auto_fly_up==0&&mode.auto_land==0&&(Rc_Pwm_Inr_mine[RC_THR]<200+1000)&&ALT_POS_SONAR2<SONAR_SET_HIGHT+0.05)
		state=SG_LOW_CHECK;	
		break;
		default:{mode_change=1;state=SD_SAFE;}break;
		
	}
	
#define DEAD_CIRCLE_CHECK 50
//circle ——check	
		if(circle.check&&circle.connect&&fabs(circle.x-160)<DEAD_CIRCLE_CHECK&&fabs(circle.y-120)<DEAD_CIRCLE_CHECK)
			mode.en_circle_nav=1;
		else
			mode.en_circle_nav=0;
	  
		//circle_search();//for test
//-----------------------NAV_OutPut--------------------
		switch(state){
			case SD_MISS_SEARCH:
		  //circle_search();
		  break;
			case SD_HOLD://for track
				  if((!mode.dj_by_hand&&mode.en_track_forward&&track.dj_fly_line&&track.check&&circle.connect))
					{
					nav_land[PITr]=track.forward;
					nav_land[ROLr]=0;
					}	
					else
					{nav_land[0]=	nav_land[1]=0;}
			break;
			case SD_HOLD2:
					if(mode.en_circle_locate&&circle.check&&circle.connect)
					{ 
					nav_land[PITr]=circle.control[0]*circle.control_k;
					nav_land[ROLr]=circle.control[1]*circle.control_k;
					}
					else  if((!mode.dj_by_hand&&mode.en_track_forward&&track.dj_fly_line&&track.check&&circle.connect&&
						Pitch_DJ<=track.forward_end_dj_pwm-100))
					{
					nav_land[PITr]=track.forward;
					nav_land[ROLr]=0;
					}	
					else
					{nav_land[0]=	nav_land[1]=0;}
			break;
			default:	
					if(mode.en_circle_locate&&circle.check&&circle.connect&&state>SD_HOLD2)
					{ 
					nav_land[PITr]=circle.control[0]*circle.control_k;
					nav_land[ROLr]=circle.control[1]*circle.control_k;
					}
					else
					{nav_land[0]=	nav_land[1]=0;}
			break;
		}
	
#define THR_RC_OFF 1500		
state_v=state;
u16 Heigh_thr=LIMIT(ultra_ctrl_out,-100,100)+THR_RC_OFF;
//-----------------state_out thr-----------------------
	switch(state)
	{
		case SG_LOW_CHECK:Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];break;
		case SG_MID_CHECK:Rc_Pwm_Out_mine[RC_THR]=LIMIT(Rc_Pwm_Inr_mine[RC_THR],0,1450);break;		
		case SU_UP1://load curve
			if(cnt[1]++>1/T)
			{thr_sel[0]++;cnt[1]=0;}
			Rc_Pwm_Out_mine[RC_THR]=AUTO_UP_CUARVE[thr_sel[0]];
		  break;
		case SU_HOLD://keep height
			if(mode.dji_sonar_high)//&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;
		case SD_RETRY_UP://retry
			Rc_Pwm_Out_mine[RC_THR]=THR_RC_OFF+80;
		  break;	
		case SD_RETRY_UP_HOLD:
		  if(mode.dji_sonar_high)
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		  break;
		case SD_MISS_SEARCH:
		  if(mode.dji_sonar_high)
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		  break;
		//-----------------------------------------auto_land---------------------------------------------
    case SD_HOLD://keep height
			if(mode.dji_sonar_high)//&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SD_HOLD2://keep height
		if(mode.dji_sonar_high)//&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
		Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
		else
		Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SD_HIGH_FAST_DOWN://load down curve 1
			if(cnt[1]++>1/T)
			{thr_sel[1]++;cnt[1]=0;}
			if(thr_sel[1]>5)
				thr_sel[1]=5;
			Rc_Pwm_Out_mine[RC_THR]=THR_RC_OFF-40;//AUTO_DOWN_CUARVE[LIMIT(thr_sel[1],0,5)];
		break;
		case SD_CIRCLE_SLOW_DOWN://load down curve 2
			if(cnt[1]++>1/T)
			{thr_sel[2]++;cnt[1]=0;}
			if(thr_sel[2]>5)
				thr_sel[2]=5;
			Rc_Pwm_Out_mine[RC_THR]=THR_RC_OFF-28;//AUTO_DOWN_CUARVE1[LIMIT(thr_sel[2],0,5)];
		break;
		case SD_CIRCLE_HOLD://keep check
			if(mode.dji_sonar_high)//&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SD_CIRCLE_MID_DOWN://land check
			Rc_Pwm_Out_mine[RC_THR]=THR_RC_OFF-30;
		break;
		case SD_CHECK_G://shut motor
			Rc_Pwm_Out_mine[RC_THR]=THR_RC_OFF-25;
		break;
		case SD_SHUT_DOWN://reset
      Rc_Pwm_Out_mine[RC_THR]=0+1000;
		break;
		//----------------------------
		case SD_SAFE://safe out
			if(mode.dji_sonar_high)//&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;
		default:Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];break;
	}
Rc_Pwm_Out_mine[RC_THR]=LIMIT(Rc_Pwm_Out_mine[RC_THR],1000,2000);


static u8 pwmin_selr,en_pid_r;	
	if(mode.en_visual_control!=pwmin_selr)
	mode_change=1;
	
	if(mode.dji_sonar_high!=en_pid_r)
	mode_change=1;
	
 en_pid_r=mode.dji_sonar_high;	
 pwmin_selr= mode.en_visual_control;	
}
