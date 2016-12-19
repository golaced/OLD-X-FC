#include "../HARDWARE/include.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/CONTROL/height_ctrl.h"
#include "../HARDWARE/DRIVER/pwm_out.h"
#include "../HARDWARE/FUSHION/alt_kf.h"
#include "../HARDWARE/CONTROL/eso.h"
#include "../HARDWARE/CONTROL/neuron_pid.h"
#include "../HARDWARE/CONTROL/visual_contol.h"
#include "../HARDWARE/CONTROL/h_inf.h"
#include "../HARDWARE/MATH/Quaternion.h"
#include "ident.h"
#include "../HARDWARE/CONTROL/sonar_avoid.h"
ctrl_t ctrl_1;
ctrl_t ctrl_2;
ctrl_t ctrl_2_fuzzy;
// Calculate nav_lat and nav_lon from the x and y error and the speed

float dj_angle,dj_angle_offset[3]={-2,3,9},dj_angle_set;
#define MAX_FIX_ANGLE_DJ 13


void Ctrl_Para_Init()		//设置默认参数
{
//====================================
	ctrl_1.PID[PIDROLL].kdamp  = 1;
	ctrl_1.PID[PIDPITCH].kdamp = 1;
	ctrl_1.PID[PIDYAW].kdamp 	 = 1;
	
	ctrl_1.FB = 0.20;   //内环  0<fb<1
}
float w_neuro[2]={1/(1.618+1),1/(1.618+1)};//1.618∶1
xyz_f_t except_A = {0,0,0};
xyz_f_t except_AR = {0,0,0};
xyz_f_t ctrl_angle_offset = {0,0,0};

xyz_f_t compensation;
#define YAW_ERO_MAX 45
float YAW_NO_HEAD;	
float except_A_SB[2],except_A_SB_lft[2],nav_angle[2],avoid_angle[2];
float scale_lf_sb=5.5;//感度
float scale_lf_nav=6;
float px_v,ix_v;
int flag_eso=1;
//--------------------1   -  0
//float off_temp[2]={(-3.24-(-4.6)),(1.57-(2.56)) };
float off_temp[2]={0 };
float off_yaw=0;//遥控 航向偏执
float k_eso_pid;
void CTRL_2(float T)//角度环
{ float px,py,ix,iy,d;
	static xyz_f_t acc_no_g;
	static xyz_f_t acc_no_g_lpf;
	float nav_angle_lft[2]={0,0};
  float dj_temp;
	static float dj_sb,dj_angle_set_out;
  static u8 flag_yaw_out=0;
  float cos1,sin1;
	float temp,temp_yaw;
	static u8 no_head;
/*   head  |    1 PIT y    AUX2
	         | 
	         _____  0 ROL x+   AUX1
	
 ROL= 0,
 PIT=1
	*/
//=========================== 期望角度 ========================================
	 except_A_SB_lft[PITr] =  my_deathzoom_2(MAX_CTRL_ANGLE  *( my_deathzoom( (CH_filter[PITr]) ,30 )/500.0f ),1);  
	 except_A_SB_lft[ROLr] =  my_deathzoom_2(MAX_CTRL_ANGLE  *( my_deathzoom( (CH_filter[ROLr]) ,30 )/500.0f ),1);  
	 except_A_SB[PITr]  += scale_lf_sb *T*3.14f * ( except_A_SB_lft[PITr] - except_A_SB[PITr] );//fix by gol 2015.11.8
	 except_A_SB[ROLr]  += scale_lf_sb *T *3.14f * ( except_A_SB_lft[ROLr] - except_A_SB[ROLr] );
//---------------------------NAV_angle------------------------------------	

	if(mode.flow_hold_position>0//&&((fabs(except_A_SB[PITr])<3.5)&&(fabs(except_A_SB[ROLr])<3.5))
			&&NAV_BOARD_CONNECT==1)//add by gol 2015.10.22
			{
			
				if(ALT_POS_SONAR2>0.1){
				nav_angle_lft[PITr]= my_deathzoom_2(nav[PITr],0.1);//  -nav_ukf_g[PIT];//nav[PIT];
				nav_angle_lft[ROLr]= my_deathzoom_2(nav[ROLr],0.1);// nav_ukf_g[ROL];//nav[ROL];	
				}
				else if(ALT_POS_SONAR2>0.05)
				{
				nav_angle_lft[PITr]=LIMIT(my_deathzoom_2(nav[PITr],0.1),-3,3);//  -nav_ukf_g[PIT];//nav[PIT];
				nav_angle_lft[ROLr]=LIMIT(my_deathzoom_2(nav[ROLr],0.1),-3,3);// nav_ukf_g[ROL];//nav[ROL];	
				}
        else
				{
				nav_angle_lft[PITr]=0;//LIMIT(-my_deathzoom(nav[PITr],0.5),-3,3);//  -nav_ukf_g[PIT];//nav[PIT];
				nav_angle_lft[ROLr]=0;//LIMIT(-my_deathzoom(nav[ROLr],0.5),-3,3);// nav_ukf_g[ROL];//nav[ROL];	
				}
				
			}



	if((mode.rc_control_flow_spd||mode.rc_control_flow_pos)&&mode.flow_hold_position==2&&imu_nav.flow.rate>20){
	nav_angle[PITr] =   nav_angle_lft[PITr];//pid.nav.out.d*nav_angle[PITr]+(1-pid.nav.out.d ) * ( nav_angle_lft[PITr]  );//y
	nav_angle[ROLr] =   nav_angle_lft[ROLr] ;//pid.nav.out.d*nav_angle[ROLr]+(1-pid.nav.out.d ) * ( nav_angle_lft[ROLr]  );//x
	}
	else
	{
	if(fabs(except_A_SB[PITr])<2)//y
		nav_angle[PITr] = nav_angle_lft[PITr];// pid.nav.out.d*nav_angle[PITr]+(1-pid.nav.out.d ) * ( nav_angle_lft[PITr]  );//y
	else
		nav_angle[PITr] = 0;

	if(fabs(except_A_SB[ROLr])<2)//x
		nav_angle[ROLr] = nav_angle_lft[ROLr] ;// pid.nav.out.d*nav_angle[ROLr]+(1-pid.nav.out.d ) * ( nav_angle_lft[ROLr]  );//x
	else
		nav_angle[ROLr] = 0;
	}	


		
  static u8 flow_pos_set_state;
	static u16 cnt_flow_set_pos;
	switch(flow_pos_set_state)
	{
		case 0:
			if((fabs(except_A_SB_lft[PITr])>2||fabs(except_A_SB_lft[ROLr])>2)&&ALT_POS_SONAR2>0.04)
			{flow_pos_set_state=1;cnt_flow_set_pos=0;}
			
		
		break;
		case 1:
			 if((fabs(except_A_SB_lft[PITr])<2&&fabs(except_A_SB_lft[ROLr])<2)&&ALT_POS_SONAR2>0.04)
				cnt_flow_set_pos++;
			 else
				 cnt_flow_set_pos=0;
				 
			 if(cnt_flow_set_pos>1.5/T)
			 {flow_pos_set_state=2;cnt_flow_set_pos=0;} 
		break;
	  case 2:
			flow_pos_set_state=0;
		break;
	}
	//out
	if(mode.rc_control_flow_pos&&imu_nav.flow.rate>20)
		flow_pos_set_state=0;
	else
		{
	switch(flow_pos_set_state)
	{	
		case 0:
		
		break;
		case 1:
			integrator[0]=integrator[1]=0;
			target_position[LON]=now_position[LON];//imu_nav.flow.position.east;
			target_position[LAT]=now_position[LAT];//imu_nav.flow.position.west; 
		break;
	  case 2:
		 	integrator[0]=integrator[1]=0;
			target_position[LON]=now_position[LON];//imu_nav.flow.position.east;
			target_position[LAT]=now_position[LAT];//imu_nav.flow.position.west; 
		break;
	}}
		if(ALT_POS_SONAR2<0.12||!fly_ready||mode.flow_hold_position==0||need_avoid)	
		{
			integrator[0]=integrator[1]=0;
			target_position[LON]=now_position[LON];//imu_nav.flow.position.east;
			target_position[LAT]=now_position[LAT];//imu_nav.flow.position.west; 
		}
	
	/*YAW -180~180     -90  135
		-	 0  + 
			 |
			 |
		-	180 +
	*/
//----------------------------------no head mode -------------------------------
	
	switch(no_head)//add by gol 11.8 (WT)
	{
		case 0:if(Thr_Low == 0 && ultra_distance>350)
		{no_head=1;YAW_NO_HEAD=Yaw;}
		else
			{
			except_AR.x=except_A_SB[ROLr];
			except_AR.y=except_A_SB[PITr];	
			}
		break;
		case 1:
			if(mode.no_head)
			{ 
			temp=Yaw-YAW_NO_HEAD;
			cos1=cos(temp*0.017);
			sin1=sin(temp*0.017);
			except_AR.x=except_A_SB[ROLr]*cos1+except_A_SB[PITr]*sin1;
			except_AR.y=except_A_SB[PITr]*cos1-except_A_SB[ROLr]*sin1;
			}	
			else
			{
			except_AR.x=except_A_SB[ROLr];
			except_AR.y=except_A_SB[PITr];	
			}
			if(Thr_Low == 1 && ultra_distance<200)
				no_head=0;
			break;
	}
	
	float cos2=cos(off_yaw*0.0173);
	float sin2=sin(off_yaw*0.0173);
/*   head  |    1 PIT y-    AUX2
	         | 
	         _____  0 ROL x+   AUX1
	
 ROL= 0,
 PIT=1
	*/
	except_AR.x=except_AR.x*cos2+sin2*except_AR.y;
	except_AR.y=except_AR.y*cos2+sin2*except_AR.x;

  if((mode.rc_control_flow_spd||mode.rc_control_flow_pos)&&mode.flow_hold_position>0){
	except_A.x=limit_mine(nav_angle[ROLr],MAX_CTRL_ANGLE);	
	except_A.y=limit_mine(nav_angle[PITr],MAX_CTRL_ANGLE);
	}
	else
	{
	except_A.x=limit_mine(except_AR.x+nav_angle[ROLr],MAX_CTRL_ANGLE);	
	except_A.y=limit_mine(except_AR.y+nav_angle[PITr],MAX_CTRL_ANGLE);
	}	
	
	
	//--------------------------IMU Control Yaw
	 if( !Thr_Low)// && ultra_distance>SONAR_HEIGHT+20 )//fix by gol 2015.11.7 && ultra_distance>150)
	{
		if((flag_yaw_out==2&&CH_filter[YAWr]>100)||(flag_yaw_out==1&&CH_filter[YAWr]<-100)||flag_yaw_out==0)
		except_AR.z += (s16)( MAX_CTRL_YAW_SPEED *( my_deathzoom_2( (CH_filter[YAWr]) ,40 )/500.0f ) ) *T ;  //50
	}
	else
	{	
		#if EN_ATT_CAL_FC
		except_AR.z += 1 *3.14 *T *( Yaw_fc - except_AR.z );
		#else
		except_AR.z += 1 *3.14 *T *( Yaw - except_AR.z );
		#endif
	}

  except_A.z  = To_180_degrees(except_AR.z);

	if(mode.tune_ctrl_angle_offset)
	{ctrl_angle_offset.x=0;off_temp[0]=LIMIT(except_AR.x,-6,6);//
	 ctrl_angle_offset.y=0;off_temp[1]=LIMIT(except_AR.y,-6,6);}
	else
	{ctrl_angle_offset.x=off_temp[0];//
	 ctrl_angle_offset.y=off_temp[1];}	
	
	//-----------------------------------ATT PID  TUNING--------------------------

	if(mode.att_pid_tune)
	{
		
	if(mcuID[0]==TUNNING_DRONE_CHIP_ID){
	if(KEY_SEL[0])//TRIG for tuning
	except_A.x=-15;
	else
	except_A.x=LIMIT(except_A.x,-30,30);	
	}else{	
	if(KEY_SEL[0])//TRIG for tuning
	#if TUNING_X	
	except_A.x=-15;
	#else
	except_A.y=15;
	#endif
	else
	#if TUNING_X	
  except_A.x=LIMIT(except_A.x,-15,15);	
	#else
	except_A.y=LIMIT(except_A.y,-15,15);	
	#endif
  }
	
		
	
	cal_ero_outter_px4(); 
  if(mode.use_px4_err){
  ctrl_2.err.x =  my_deathzoom_2(ero_angle_px4[0],0.0);
	ctrl_2.err.y =  my_deathzoom_2(ero_angle_px4[1],0.0);
	#if EN_ATT_CAL_FC
	ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw_fc	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#else	
	ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#endif
	}else{	
	ctrl_2.err.x =  my_deathzoom_2(To_180_degrees( ctrl_angle_offset.x + except_A.x - Roll  ),0.1);
	ctrl_2.err.y =  my_deathzoom_2(To_180_degrees( ctrl_angle_offset.y + except_A.y - Pitch ),0.1);}
	#if EN_ATT_CAL_FC
	ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw_fc	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#else	
	ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#endif	
	}
	else{	
	cal_ero_outter_px4(); 
  /* 得到角度误差 */
	if(mode.use_px4_err){
  ctrl_2.err.x =  my_deathzoom_2(ero_angle_px4[0],0.0);
	ctrl_2.err.y =  my_deathzoom_2(ero_angle_px4[1],0.0);
	#if EN_ATT_CAL_FC
	ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw_fc	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#else	
	ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#endif
	}else{	
	ctrl_2.err.x =  my_deathzoom_2(To_180_degrees( ctrl_angle_offset.x + except_A.x - Roll  ),0.1);
	ctrl_2.err.y =  my_deathzoom_2(To_180_degrees( ctrl_angle_offset.y + except_A.y - Pitch ),0.1);}
	#if EN_ATT_CAL_FC
	ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw_fc	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#else	
	ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#endif
	}
	
  
	//------------------YAW PROTECTOR--------------------------------
	#if EN_ATT_CAL_FC
	if(fabs(To_180_degrees(except_A.z - Yaw_fc))>80){
		except_A.z  = To_180_degrees(Yaw);
	#else
	if(fabs(To_180_degrees(except_A.z - Yaw))>80){
		except_A.z  = To_180_degrees(Yaw);
	#endif
	ctrl_2.err.z=0;
	}
	
	my_deathzoom(except_A.x,0.01);
	my_deathzoom(except_A.y,0.01);
	my_deathzoom(except_A.z,0.01);
	
	
  if((ctrl_2.err.z)>=YAW_ERO_MAX)
		flag_yaw_out=1;
	else  if((ctrl_2.err.z)<=-YAW_ERO_MAX)
		flag_yaw_out=2;
	else
		flag_yaw_out=0;


	px=py=ctrl_2.PID[PIDROLL].kp;
	ix=iy=ctrl_2.PID[PIDROLL].ki;
	d=ctrl_2.PID[PIDROLL].kd;	

//----------------------------------------ESO FORWARD FEEDBACK-------------------------------
	/* 计算角度误差权重 */
	ctrl_2.err_weight.x = ABS(ctrl_2.err.x)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.y = ABS(ctrl_2.err.y)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.z = ABS(ctrl_2.err.z)/ANGLE_TO_MAX_AS;
//	/* 角度误差微分（跟随误差曲线变化）*/
//	if(0){//mode.en_fuzzy_angle_pid){
//	ctrl_2.err_d.x = 10 *eso_att_outter_c[PITr].KD *(ctrl_2.err.x - ctrl_2.err_old.x) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.x );
//	ctrl_2.err_d.y = 10 *eso_att_outter_c[PITr].KD *(ctrl_2.err.y - ctrl_2.err_old.y) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.y );
//	}else{	
	ctrl_2.err_d.x = 10 *d *(ctrl_2.err.x - ctrl_2.err_old.x) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.x );
	ctrl_2.err_d.y = 10 *d *(ctrl_2.err.y - ctrl_2.err_old.y) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.y );
	//}
	ctrl_2.err_d.z = 10 *ctrl_2.PID[PIDYAW].kd 	 *(ctrl_2.err.z - ctrl_2.err_old.z) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.z );
	/* 角度误差积分 */
	ctrl_2.err_i.x +=ix  *ctrl_2.err.x *T;
	ctrl_2.err_i.y +=iy  *ctrl_2.err.y *T;
	ctrl_2.err_i.z += ctrl_2.PID[PIDYAW].ki 	*ctrl_2.err.z *T;
	/* 角度误差积分分离 */
	ctrl_2.eliminate_I.x = Thr_Weight *CTRL_2_INT_LIMIT;
	ctrl_2.eliminate_I.y = Thr_Weight *CTRL_2_INT_LIMIT;
	ctrl_2.eliminate_I.z = Thr_Weight *CTRL_2_INT_LIMIT;
	/* 角度误差积分限幅 */
	if(fabs(ctrl_2.err.x)<eso_att_outter_c[PITr].eso_dead||eso_att_outter_c[PITr].b0==0)
	ctrl_2.err_i.x = LIMIT( ctrl_2.err_i.x, -ctrl_2.eliminate_I.x,ctrl_2.eliminate_I.x );
	else
	ctrl_2.err_i.x=0;

	if(fabs(ctrl_2.err.y)<eso_att_outter_c[PITr].eso_dead||eso_att_outter_c[PITr].b0==0)
	ctrl_2.err_i.y = LIMIT( ctrl_2.err_i.y, -ctrl_2.eliminate_I.y,ctrl_2.eliminate_I.y );
	else
	ctrl_2.err_i.y=0;
	
	ctrl_2.err_i.z = LIMIT( ctrl_2.err_i.z, -ctrl_2.eliminate_I.z,ctrl_2.eliminate_I.z );
	/* 对用于计算比例项输出的角度误差限幅 */
	ctrl_2.err.x = LIMIT( ctrl_2.err.x, -90, 90 );
	ctrl_2.err.y = LIMIT( ctrl_2.err.y, -90, 90 );
	ctrl_2.err.z = LIMIT( ctrl_2.err.z, -90, 90 );
	//---------------------SELF DISTURB------------------------------
	ATT_CONTRL_OUTER_ESO_3(&eso_att_outter_c[PITr],except_A.y,Pit_fc,ctrl_2.out.y,T,ANGLE_TO_MAX_AS,ctrl_2.err.y);
	ATT_CONTRL_OUTER_ESO_3(&eso_att_outter_c[ROLr],except_A.x,Rol_fc,ctrl_2.out.x,T,ANGLE_TO_MAX_AS,ctrl_2.err.x);
	float x_out,y_out;
	x_out=px*( ctrl_2.err.x + ctrl_2.err_d.x + ctrl_2.err_i.x );
	y_out=py*( ctrl_2.err.y + ctrl_2.err_d.y + ctrl_2.err_i.y );
//-----------------------------------CONTROL OUT SEL---------------------	
	
	k_eso_pid=LIMIT(10*fabs(ctrl_2.err.x)/MAX_CTRL_ANGLE,0.3,1);///eso_att_outter_c[PITr].eso_dead;
	if(eso_att_outter_c[PITr].b0!=0){//use eso
	//ctrl_2.err_i.x=ctrl_2.err_i.y=ctrl_2.err_i.z=0;
	ctrl_2.out.x=(eso_att_outter_c[ROLr].u+px  *( ctrl_2.err_d.x+ ctrl_2.err_i.x  ));//*w_neuro[0]+(1-w_neuro[0])*x_out;
	ctrl_2.out.y=	(eso_att_outter_c[PITr].u+py  *( ctrl_2.err_d.y+ ctrl_2.err_i.y  ));//*w_neuro[1]+(1-w_neuro[1])*y_out;
	}
	else{//origin pid
	ctrl_2.out.x = x_out;	//rol
	ctrl_2.out.y = y_out;  //pit
  }
	
	ctrl_2.out.z = ctrl_2.PID[PIDYAW].kp   *( ctrl_2.err.z + ctrl_2.err_d.z + ctrl_2.err_i.z );
	/* 记录历史数据 */	
	ctrl_2.err_old.x = ctrl_2.err.x;
	ctrl_2.err_old.y = ctrl_2.err.y;
	ctrl_2.err_old.z = ctrl_2.err.z;

  
}

//----------------------------------------INNER------------------------------------------------
xyz_f_t except_AS;
int inner_set=0;
u8 en_h_inf=0;
float g_old[7];
float k_rc_gyro_spd=0.4;
void CTRL_1(float T)  //x roll,y pitch,z yaw 角速度  内环  2ms
{float ctrl_angle_out[3]={0},ctrl_angle_weight[3]={0};
	xyz_f_t EXP_LPF_TMP;
	
  if(ctrl_2.PID->kp==0||KEY[7]==0||inner_set){
	if(fabs(Rol_fc)<40)
	ctrl_angle_out[0]=except_A.x*k_rc_gyro_spd;
	else if((Rol_fc)>40)
	ctrl_angle_out[0]=LIMIT(except_A.x*k_rc_gyro_spd,-MAX_CTRL_ANGLE*k_rc_gyro_spd,0);	
	else if((Rol_fc)<-40)
	ctrl_angle_out[0]=LIMIT(except_A.x*k_rc_gyro_spd,0,MAX_CTRL_ANGLE*k_rc_gyro_spd);	
  else
	ctrl_angle_out[0]=ctrl_2.out.x;
	ctrl_angle_out[1]=ctrl_2.out.y;
	ctrl_angle_out[2]=ctrl_2.out.z;
		
	}else{
	ctrl_angle_out[0]=ctrl_2.out.x;
	ctrl_angle_out[1]=ctrl_2.out.y;
	ctrl_angle_out[2]=ctrl_2.out.z;}
	
	ctrl_angle_weight[0]=ctrl_2.err_weight.x;
	ctrl_angle_weight[1]=ctrl_2.err_weight.y;
	ctrl_angle_weight[2]=ctrl_2.err_weight.z;
	
	/* 给期望（目标）角速度 */
	EXP_LPF_TMP.x = MAX_CTRL_ASPEED *(ctrl_angle_out[0]/ANGLE_TO_MAX_AS);//*( (CH_filter[0])/500.0f );//
	EXP_LPF_TMP.y = MAX_CTRL_ASPEED *(ctrl_angle_out[1]/ANGLE_TO_MAX_AS);//*( (CH_filter[1])/500.0f );//
	EXP_LPF_TMP.z = MAX_CTRL_ASPEED *(ctrl_angle_out[2]/ANGLE_TO_MAX_AS);

	except_AS.x = EXP_LPF_TMP.x;//20 *3.14 *T *( EXP_LPF_TMP.x - except_AS.x );//
	except_AS.y = EXP_LPF_TMP.y;//20 *3.14 *T *( EXP_LPF_TMP.y - except_AS.y );//
	except_AS.z = EXP_LPF_TMP.z;//20 *3.14 *T *( EXP_LPF_TMP.z - except_AS.z );//
	/* 期望角速度限幅 */
	except_AS.x = LIMIT(except_AS.x, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
	except_AS.y = LIMIT(except_AS.y, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
	except_AS.z = LIMIT(except_AS.z, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );

	/* 角速度直接微分（角加速度），负反馈可形成角速度的阻尼（阻碍角速度的变化）*/
	ctrl_1.damp.x = ( mpu6050.Gyro_deg.x - g_old[A_X]) *( 0.002f/T );//ctrl_1.PID[PIDROLL].kdamp
	ctrl_1.damp.y = (-mpu6050.Gyro_deg.y - g_old[A_Y]) *( 0.002f/T );//ctrl_1.PID[PIDPITCH].kdamp *
	ctrl_1.damp.z = (-mpu6050.Gyro_deg.z - g_old[A_Z]) *( 0.002f/T );//ctrl_1.PID[PIDYAW].kdamp	 *
	/* 角速度误差 */
	ctrl_1.err.x =  ( except_AS.x - mpu6050.Gyro_deg.x ) *(300.0f/MAX_CTRL_ASPEED);
	ctrl_1.err.y =  ( except_AS.y + mpu6050.Gyro_deg.y ) *(300.0f/MAX_CTRL_ASPEED);  //-y
	ctrl_1.err.z =  ( except_AS.z + mpu6050.Gyro_deg.z ) *(300.0f/MAX_CTRL_ASPEED);	 //-z
//----------------------------------ESO-----------------------------		
	
	/* 角速度误差权重 */
	ctrl_1.err_weight.x = ABS(ctrl_1.err.x)/MAX_CTRL_ASPEED;
	ctrl_1.err_weight.y = ABS(ctrl_1.err.y)/MAX_CTRL_ASPEED;
	ctrl_1.err_weight.z = ABS(ctrl_1.err.z)/MAX_CTRL_YAW_SPEED;
	/* 角速度微分 */	
	ctrl_1.err_d.x = ( ctrl_1.PID[PIDROLL].kd  *( -10 *ctrl_1.damp.x) *( 0.002f/T ) );
	ctrl_1.err_d.y = ( ctrl_1.PID[PIDPITCH].kd *( -10 *ctrl_1.damp.y) *( 0.002f/T ) );
	ctrl_1.err_d.z = ( ctrl_1.PID[PIDYAW].kd   *( -10 *ctrl_1.damp.z) *( 0.002f/T ) );
 
	/* 角速度误差积分 */
	ctrl_1.err_i.x += ctrl_1.PID[PIDROLL].ki  *(ctrl_1.err.x - ctrl_1.damp.x) *T;
	ctrl_1.err_i.y += ctrl_1.PID[PIDPITCH].ki *(ctrl_1.err.y - ctrl_1.damp.y) *T;
	ctrl_1.err_i.z += ctrl_1.PID[PIDYAW].ki 	*(ctrl_1.err.z - ctrl_1.damp.z) *T;
	/* 角速度误差积分分离 */
	ctrl_1.eliminate_I.x = Thr_Weight *CTRL_1_INT_LIMIT ;
	ctrl_1.eliminate_I.y = Thr_Weight *CTRL_1_INT_LIMIT ;
	ctrl_1.eliminate_I.z = Thr_Weight *CTRL_1_INT_LIMIT ;
	/* 角速度误差积分限幅 */
	if(fabs(ctrl_1.err.x)<eso_att_inner_c[PITr].eso_dead||eso_att_inner_c[PITr].b0==0)
	ctrl_1.err_i.x = LIMIT( ctrl_1.err_i.x, -ctrl_1.eliminate_I.x,ctrl_1.eliminate_I.x );
	else
	ctrl_1.err_i.x=0;
	
	if(fabs(ctrl_1.err.y)<eso_att_inner_c[PITr].eso_dead||eso_att_inner_c[PITr].b0==0)
	ctrl_1.err_i.y = LIMIT( ctrl_1.err_i.y, -ctrl_1.eliminate_I.y,ctrl_1.eliminate_I.y );
	else
	ctrl_1.err_i.y=0;
	if(fabs(ctrl_1.err.z)<eso_att_inner_c[PITr].eso_dead||eso_att_inner_c[PITr].b0==0)
	ctrl_1.err_i.z = LIMIT( ctrl_1.err_i.z, -ctrl_1.eliminate_I.z,ctrl_1.eliminate_I.z );
	else
	ctrl_1.err_i.z=0;
	//-----------------------------------------ESO
	ATT_CONTRL_INNER_ESO_3(&eso_att_inner_c[PITr],except_AS.y,-mpu6050.Gyro_deg.y,eso_att_inner_c[PITr].u,T,200);
	ATT_CONTRL_INNER_ESO_3(&eso_att_inner_c[ROLr],except_AS.x,mpu6050.Gyro_deg.x,eso_att_inner_c[ROLr].u,T,200);
	ATT_CONTRL_INNER_ESO_3(&eso_att_inner_c[YAWr],except_AS.z,mpu6050.Gyro_deg.z,eso_att_inner_c[YAWr].u,T,200);
	float inf_out;
	
	AUTO_B0(&eso_att_inner_c[PITr],0,0,0,0,0);
	AUTO_B0(&eso_att_inner_c[ROLr],0,0,0,0,0);
	AUTO_B0(&eso_att_inner_c[YAWr],0,0,0,0,0);
	if(en_h_inf){
	inf_out=h_inf_att_inner1(except_AS.x,mpu6050.Gyro_deg.x,200);	
	ctrl_1.out.x = inf_out;
	}	
	else if(eso_att_inner_c[PITr].b0!=0){
		ctrl_1.FB=0;
	//ctrl_1.err_i.x=ctrl_1.err_i.y=ctrl_1.err_i.z=0;
	ctrl_1.out.x = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.x),0,1)*except_AS.x+( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDPITCH].kp *( ctrl_1.err_d.x + ctrl_1.err_i.x)) 
								+( 1 - ctrl_1.FB ) *eso_att_inner_c[ROLr].u;
	ctrl_1.out.y = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.y),0,1)*except_AS.y+( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDPITCH].kp *( ctrl_1.err_d.y + ctrl_1.err_i.y))
								+( 1 - ctrl_1.FB ) *eso_att_inner_c[PITr].u;
	ctrl_1.out.z = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.z),0,1)*except_AS.z+( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDYAW].kp *(  ctrl_1.err_i.z  + ctrl_1.err_i.z) )
								+( 1 - ctrl_1.FB ) *eso_att_inner_c[PITr].u;;
	}else{	
		ctrl_1.FB=0.2;
	/* 角速度PID输出 */
	ctrl_1.out.x = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.x),0,1)*except_AS.x + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDROLL].kp  *( ctrl_1.err.x + ctrl_1.err_d.x + ctrl_1.err_i.x ) );
	ctrl_1.out.y = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.y),0,1)*except_AS.y + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDPITCH].kp *( ctrl_1.err.y + ctrl_1.err_d.y + ctrl_1.err_i.y ) );
	ctrl_1.out.z = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.z),0,1)*except_AS.z + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDYAW].kp   *( ctrl_1.err.z + ctrl_1.err_d.z + ctrl_1.err_i.z ) );
	}

	if(inner_set!=0)
	ctrl_1.out.x=inner_set;	
#if !EN_TIM_INNER
	Thr_Ctrl(T);// 油门控制
#endif

	if(mode.att_pid_tune)
	{	
	if(mcuID[0]==TUNNING_DRONE_CHIP_ID){
	ctrl_1.out.z=0;	
	ctrl_1.out.y=0;	
	}else{
	ctrl_1.out.z=0;
	#if TUNING_X		
	ctrl_1.out.y=0;	
	#else
	ctrl_1.out.x=0;		
	#endif
  }
	}

	All_Out(ctrl_1.out.x,ctrl_1.out.y,ctrl_1.out.z);


	ctrl_1.err_old.x = ctrl_1.err.x;
	ctrl_1.err_old.y = ctrl_1.err.y;
	ctrl_1.err_old.z = ctrl_1.err.z;

	g_old[A_X] =  mpu6050.Gyro_deg.x ;
	g_old[A_Y] = -mpu6050.Gyro_deg.y ;
	g_old[A_Z] = -mpu6050.Gyro_deg.z ;
}


//----------------------------------------------THR --------------------------------

u16 att_tuning_thr_limit;
int baro_to_ground,baro_ground_off;
float thr_value;
u8 Thr_Low,force_Thr_low=0;
float Thr_Weight,Thr_Weight_ATT;
float thr_test;
float k_thr_att=1.321;
void Thr_Ctrl(float T)
{	float delta_thr;
	static float thr;
	static float Thr_tmp;
		static u8 cnt_thr_add,fly_ready_r;
	if(!fly_ready)
	   thr=0;		 
	else
    thr = 500 + CH_filter[THRr]; //油门值 0 ~ 1000
//----------Drop protector-----------------
	if(!fly_ready&&500 + CH_filter[THRr]<100)
	force_Thr_low=0;
	if((fabs(Pit_fc)>60||fabs(Rol_fc)>60)&&fly_ready)
		force_Thr_low=1;
//protect flag init	
	if(fly_ready_r==0&&fly_ready==1&&500 + CH_filter[THRr]>100)
		force_Thr_low=1;
		fly_ready_r=fly_ready;
	
	if(mode.use_dji)
		force_Thr_low=1;
	if(force_Thr_low)
		thr=0;
	
	
	Thr_tmp += 10 *3.14f *T *(thr/200.0f - Thr_tmp); //低通滤波
	Thr_Weight = LIMIT(Thr_tmp,0,1);    							//后边多处分离数据会用到这个值
	Thr_Weight_ATT = LIMIT(Thr_tmp*k_thr_att,0,1); 
if(mode.use_dji)
{
if(500 + CH_filter[THRr]<50)
		Thr_Low = 1;
	else
		Thr_Low = 0;
}
else{
	if( thr < 50 )
	{
		Thr_Low = 1;
	
	}
	else
	{ 
		Thr_Low = 0;
	} 
}
	
	if( 500 + CH_filter[THRr]<100)	baro_ground_off=ALT_POS_BMP*1000;
	baro_to_ground=LIMIT(ALT_POS_BMP*1000-baro_ground_off,10,8000);
	
	Height_Ctrl(T,thr);
	thr_value = Thr_Weight *height_ctrl_out;   //实际使用值	
	if(mode.att_pid_tune)
	{	
	if(att_tuning_thr_limit==0)
	#if defined(ZHOU_550)//-----------------------------------------550----------------------------------	
  att_tuning_thr_limit=550;
  #elif defined(ZHOU_300)	
	att_tuning_thr_limit=400;
	#endif
	thr_test=thr_value = LIMIT(thr,0,LIMIT(att_tuning_thr_limit,0,800));
	}		
	else
	thr_test=thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);
}


float motor[MAXMOTORS];
float posture_value[MAXMOTORS];
float curve[MAXMOTORS];
#if defined(ZHOU_550)
float scale_thr_fix=1.25;//油门补偿系数
#elif defined(ZHOU_300)
float scale_thr_fix=1;//油门补偿系数
#endif
#define MAX_THR_FIX_ANGLE MAX_CTRL_ANGLE
int thr_value_fix;
s16 motor_out[MAXMOTORS];
int posture_value_test[6];
u8 en_motor_sel=0;
void All_Out(float out_roll,float out_pitch,float out_yaw)
{
	u8 i;
	float posture_value[MAXMOTORS];
  float curve[MAXMOTORS];
	static float motor_last[MAXMOTORS];
  #if  defined(ZHOU_300)
	out_yaw = LIMIT( out_yaw , -5*MAX_THR ,5*MAX_THR ); //50%
	#else
	out_yaw = LIMIT( out_yaw , -5*MAX_THR ,5*MAX_THR ); //50%
	#endif
	#if DRONE_X6
	posture_value[0] = - 0.5*out_roll + 0.866*out_pitch + out_yaw ;
	posture_value[1] = + 0.5*out_roll + 0.886*out_pitch - out_yaw ;
	posture_value[4] = + out_roll + out_yaw ;
	
	posture_value[2] = + 0.5*out_roll - 0.866*out_pitch - out_yaw ;
	posture_value[3] = - 0.5*out_roll - 0.866*out_pitch + out_yaw ;
	posture_value[5] = - out_roll - out_yaw ;
	if(en_motor_sel)
	for(i=0;i<6;i++)
	posture_value[i]=posture_value_test[i];
	#else
	posture_value[0] = - out_roll + out_pitch + out_yaw ;
	posture_value[1] = + out_roll + out_pitch - out_yaw ;
	posture_value[2] = + out_roll - out_pitch + out_yaw ;
	posture_value[3] = - out_roll - out_pitch - out_yaw ;
	#endif
	if(mcuID[0]==TUNNING_DRONE_CHIP_ID){
	posture_value[0] = - out_roll ;
	posture_value[1] = + out_roll ;	
	}
	
	
	for(i=0;i<6;i++)
	{
		posture_value[i] = LIMIT(posture_value[i], -1000,1000 );
	}
	
	curve[0] = (0.55f + 0.45f *ABS(posture_value[0])/1000.0f) *posture_value[0] ;
	curve[1] = (0.55f + 0.45f *ABS(posture_value[1])/1000.0f) *posture_value[1] ;
	curve[2] = (0.55f + 0.45f *ABS(posture_value[2])/1000.0f) *posture_value[2] ;
	curve[3] = (0.55f + 0.45f *ABS(posture_value[3])/1000.0f) *posture_value[3] ;
	curve[4] = (0.55f + 0.45f *ABS(posture_value[4])/1000.0f) *posture_value[4] ;
	curve[5] = (0.55f + 0.45f *ABS(posture_value[5])/1000.0f) *posture_value[5] ;

	int date_throttle;
	if(!mode.att_pid_tune){//add by gol 16.3.28 (WT)  油门补偿
		thr_value_fix=LIMIT((thr_value/cos(LIMIT(my_deathzoom_2(Pitch,5),-MAX_THR_FIX_ANGLE,MAX_THR_FIX_ANGLE)/57)/
							      cos(LIMIT(my_deathzoom_2(Roll,5),-MAX_THR_FIX_ANGLE,MAX_THR_FIX_ANGLE)/57)-thr_value),0,200)*scale_thr_fix;
		date_throttle=thr_value+thr_value_fix;
	}
		else{
		date_throttle	= thr_value;
	}
	
	date_throttle	= thr_value;
	if(en_motor_sel){date_throttle=0;Thr_Weight=1;}
	motor[0] = date_throttle + Thr_Weight_ATT *curve[0] ;
	motor[1] = date_throttle + Thr_Weight_ATT *curve[1] ;
	motor[2] = date_throttle + Thr_Weight_ATT *curve[2] ;
	motor[3] = date_throttle + Thr_Weight_ATT *curve[3] ;
	motor[4] = date_throttle + Thr_Weight_ATT *curve[4] ;
	motor[5] = date_throttle + Thr_Weight_ATT *curve[5] ;
	mode.en_moto_smooth=0;
	  if(mode.en_moto_smooth){
     for(i=0;i<MAXMOTORS;i++){
        if(motor[i] > motor_last[i]) 
					motor[i] = (1 * (int16_t) motor_last[i] + motor[i]) / 2;  //mean of old and new
        else                                         
					motor[i] = motor[i] - (motor_last[i] - motor[i]) * 1; // 2 * new - old
			}
			 for(i=0;i<MAXMOTORS;i++)
					motor_last[i] = motor[i];  //mean of old and new
	    }
			
	/* 是否解锁 */
	if(fly_ready)
	{
		if( !Thr_Low )  //油门拉起
		{
			for(i=0;i<6;i++)
			{
				motor[i] = LIMIT(motor[i], (10 *READY_SPEED),(10*MAX_PWM) );
			}
		}
		else						//油门低
		{
			for(i=0;i<6;i++)
			{
				motor[i] = LIMIT(motor[i], 0,(10*MAX_PWM) );
			}
		}
	}
	else
	{
		for(i=0;i<6;i++)
		{
			motor[i] = 0;
		}
	}

  motor_out[0] = (s16)(motor[0]);  
	motor_out[1] = (s16)(motor[1]);	 
	motor_out[2] = (s16)(motor[2]);
	motor_out[3] = (s16)(motor[3]);
	motor_out[4] = (s16)(motor[4]);
	motor_out[5] = (s16)(motor[5]);

	SetPwm(motor_out,0,1000); //
}
//



