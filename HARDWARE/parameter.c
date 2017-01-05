
#include "../HARDWARE/include.h"
#include "../HARDWARE/MEMS/mpu6050.h"
#include "../HARDWARE/MEMS/hml_sample.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/CONTROL/height_ctrl.h"
#include "../HARDWARE/CONTROL/eso.h"
#include "string.h"
#define SENSOR_SETUP_FILE      "sensor.bin"
#define PID_SETUP_FILE         "pid.bin"
u8 flash_init_error;
static sensor_setup_t sensor_setup;
static pid_setup_t pid_setup;

struct _DRONE drone;
static void Drone_init(void)
{drone.type=DRONE_N;
 drone.use_drone_pid=1;
	
 drone.body.l=0.14;
 drone.body.mess_l=0.005;
 
 drone.bldc.num=4;	
 drone.bldc.mess=0.03;	
 drone.bldc.gain=0.0025;
 drone.bldc.time=0.014;
 drone.bldc.off=-0.2734;
switch(drone.type){	
	case DRONE_N:
		drone.ctrl1.roll.kp=drone.ctrl1.pitch.kp=(2*(drone.bldc.mess+2*drone.body.mess_l)*drone.body.l-drone.bldc.off)/(57.3*drone.bldc.gain*5.73);
  break;		
	
}
}


static void  Param_SetSettingToFC(void) //fly thr 45% at 4s full 4.4Ah
{ Drone_init();
	memcpy(&mpu6050.Acc_Offset,&sensor_setup.Offset.Accel,sizeof(xyz_f_t));
	memcpy(&mpu6050.Gyro_Offset,&sensor_setup.Offset.Gyro,sizeof(xyz_f_t));
	memcpy(&ak8975.Mag_Offset,&sensor_setup.Offset.Mag,sizeof(xyz_f_t));
	mpu6050.Temprea_Offset = sensor_setup.Offset.Temperature;

////--------------------------------------Angle----------------------------------------	
	switch(mcuID[0]){
		case DRONE_330_ID://300
						//p
  pid_setup.groups.ctrl2.roll.kp=pid_setup.groups.ctrl2.pitch.kp =0.4;//0.35;//28;//0.425;//0.425;//<-----------mini0.45 for 4050  0.4 for 6030 WT
	SPID.IP=pid_setup.groups.ctrl2.roll.kp*1000;
	//i
	pid_setup.groups.ctrl2.roll.ki =pid_setup.groups.ctrl2.pitch.ki = 0.00;//5;//5;//0.15;//0.25;//0.05;
	SPID.II=pid_setup.groups.ctrl2.roll.ki*1000;
	//d
	pid_setup.groups.ctrl2.roll.kd  = pid_setup.groups.ctrl2.pitch.kd =0.3;//0.2;//0.3;//2;//0.425;//35;//<-------------mini
	SPID.ID=pid_setup.groups.ctrl2.roll.kd*1000;
	//---------------------------------------GRO--------------------------------
	//p
	pid_setup.groups.ctrl1.roll.kp=pid_setup.groups.ctrl1.pitch.kp = 0.800;//0.3;//0.425;//0.35;//<-----------mini  WT
	SPID.OP=pid_setup.groups.ctrl1.roll.kp*1000;
	//i
  pid_setup.groups.ctrl1.roll.ki =pid_setup.groups.ctrl1.pitch.ki =0.1;// 0.05;//0.1;
	SPID.OI=pid_setup.groups.ctrl1.roll.ki*1000;
	//d
	pid_setup.groups.ctrl1.roll.kd=pid_setup.groups.ctrl1.pitch.kd = 2.0;//4;//2.0;
	SPID.OD=pid_setup.groups.ctrl1.roll.kd*1000;
	//--------------------------------------YAW-------------------------------------------------
	pid_setup.groups.ctrl2.yaw.kp   = 0.8;//0.4;//0.8;	
	SPID.YD=pid_setup.groups.ctrl2.yaw.kp*1000;
	pid_setup.groups.ctrl2.yaw.ki   = 0;//0.05;	
	SPID.YI=pid_setup.groups.ctrl2.yaw.ki*1000;
  pid_setup.groups.ctrl2.yaw.kd   = 0;//0.3;//0.1;//0.3;
	//------------------------------------
	pid_setup.groups.ctrl1.yaw.kp   = 0.8;//1.2;//0.8;
	SPID.YP=pid_setup.groups.ctrl1.yaw.kp*1000;	
	
	pid_setup.groups.ctrl1.yaw.ki   = 0.1;//1.0;//0.5;//0.1;	
	pid_setup.groups.ctrl1.yaw.kd   = 1.0;//1.0;//1.5;	
	pid_setup.groups.ctrl1.roll.kdamp=pid_setup.groups.ctrl1.pitch.kdamp = 1;
	pid_setup.groups.ctrl1.yaw.kdamp   = 1;
 //--------------------------------------------光流定位----------------------------
	  pid.nav.out.p=5;////0.35;
		pid.nav.out.i=0.25;//8;//(wt)
		pid.nav.out.d=0;//8;//(wt)
		pid.nav.in.p =0.8;//3.5;//0.365;//0.350;//0.4;//fix 2015.10.21 导航均值滤波 阴天//0.025;//150
		pid.nav.in.i =0.005;

		pid.nav.in.d =0.2;//0.250;
		pid.nav.out.dead= 0.025;//位置死区mm
		pid.nav.in.dead=  0.025;//3;//速度死区mm/s
		pid.nav.in.dead2=  pid.nav.in.dead*1.5;//4.5;//速度积分死区mm/s
 //-----------------------------壁障----------------------------------
	pid.avoid.out.p=0.1;
 //----------------------------ESO------------------------------------
	eso_att_outter[PITr].n=eso_att_outter[ROLr].n=eso_att_outter[YAWr].n=15;
  eso_att_inner[PITr].n=eso_att_inner[ROLr].n=eso_att_inner[YAWr].n=66;
		break;
		case DRONE_350_ID://450-------------------------------------------------------------------
					//p
  pid_setup.groups.ctrl2.roll.kp=pid_setup.groups.ctrl2.pitch.kp =0.4;//0.325;//0.4;//0.35;//28;//0.425;//0.425;//<-----------mini0.45 for 4050  0.4 for 6030 WT
	SPID.IP=pid_setup.groups.ctrl2.roll.kp*1000;
	//i
	pid_setup.groups.ctrl2.roll.ki =pid_setup.groups.ctrl2.pitch.ki = 0.00;//5;//5;//0.15;//0.25;//0.05;
	SPID.II=pid_setup.groups.ctrl2.roll.ki*1000;
	//d
	pid_setup.groups.ctrl2.roll.kd  = pid_setup.groups.ctrl2.pitch.kd =0.3;//0.2;//0.3;//0.3;//2;//0.425;//35;//<-------------mini
	SPID.ID=pid_setup.groups.ctrl2.roll.kd*1000;
	//---------------------------------------GRO--------------------------------
	//p
	pid_setup.groups.ctrl1.roll.kp=pid_setup.groups.ctrl1.pitch.kp = 0.7;//0.425;//0.3;//0.425;//0.35;//<-----------mini  WT
	SPID.OP=pid_setup.groups.ctrl1.roll.kp*1000;
	//i
  pid_setup.groups.ctrl1.roll.ki =pid_setup.groups.ctrl1.pitch.ki =0.1;// 0.05;//0.1;
	SPID.OI=pid_setup.groups.ctrl1.roll.ki*1000;
	//d
	pid_setup.groups.ctrl1.roll.kd=pid_setup.groups.ctrl1.pitch.kd =2;//4;//2.0;
	SPID.OD=pid_setup.groups.ctrl1.roll.kd*1000;
	//--------------------------------------YAW-------------------------------------------------
	pid_setup.groups.ctrl2.yaw.kp   = 0.8;//0.4;//0.8;	
	SPID.YD=pid_setup.groups.ctrl2.yaw.kp*1000;
	pid_setup.groups.ctrl2.yaw.ki   = 0;//0.05;	
	SPID.YI=pid_setup.groups.ctrl2.yaw.ki*1000;
  pid_setup.groups.ctrl2.yaw.kd   = 0;//0.3;//0.1;//0.3;
	//------------------------------------
	pid_setup.groups.ctrl1.yaw.kp   = 0.8;//1.2;//0.8;
	SPID.YP=pid_setup.groups.ctrl1.yaw.kp*1000;	
	
	pid_setup.groups.ctrl1.yaw.ki   = 0.1;//1.0;//0.5;//0.1;	
	pid_setup.groups.ctrl1.yaw.kd   = 1.0;//1.0;//1.5;	
	pid_setup.groups.ctrl1.roll.kdamp=pid_setup.groups.ctrl1.pitch.kdamp = 1;
	pid_setup.groups.ctrl1.yaw.kdamp   = 1;
 //--------------------------------------------光流定位----------------------------
		pid.nav.out.p=0;//4;////0.35;
		pid.nav.out.i=0;//8;//(wt)
		pid.nav.in.p =0.65;//3.5;//0.365;//0.350;//0.4;//fix 2015.10.21 导航均值滤波 阴天//0.025;//150
		pid.nav.in.i =0.01;

		pid.nav.in.d =0.2;//0.250;
		pid.nav.out.dead= 0.4;//位置死区mm
		pid.nav.in.dead=  0.025;//3;//速度死区mm/s
		pid.nav.in.dead2=  pid.nav.in.dead*1.5;//4.5;//速度积分死区mm/s
 //-----------------------------壁障----------------------------------
	pid.avoid.out.p=0.1;
 //----------------------------ESO------------------------------------
	eso_att_outter[PITr].n=eso_att_outter[ROLr].n=eso_att_outter[YAWr].n=70;
  eso_att_inner[PITr].n=eso_att_inner[ROLr].n=eso_att_inner[YAWr].n=100;
		break;
		default:	//p-----------------------------------------------------------------------------------------------------
	#if EN_TIM_INNER		
		pid_setup.groups.ctrl2.roll.kp=pid_setup.groups.ctrl2.pitch.kp =0.66;//0.4;//0.35;//28;//0.425;//0.425;//<-----------mini0.45 for 4050  0.4 for 6030 WT
			SPID.IP=pid_setup.groups.ctrl2.roll.kp*1000;
			//i
			pid_setup.groups.ctrl2.roll.ki =pid_setup.groups.ctrl2.pitch.ki = 0.15;//5;//5;//0.15;//0.25;//0.05;
			SPID.II=pid_setup.groups.ctrl2.roll.ki*1000;
			//d
			pid_setup.groups.ctrl2.roll.kd  = pid_setup.groups.ctrl2.pitch.kd =0.2;//0.3;//0.3;//2;//0.425;//35;//<-------------mini
			SPID.ID=pid_setup.groups.ctrl2.roll.kd*1000;
			//---------------------------------------GRO--------------------------------
			//p
			pid_setup.groups.ctrl1.roll.kp=pid_setup.groups.ctrl1.pitch.kp = 0.6;//0.65;//0.3;//0.425;//0.35;//<-----------mini                                                                               WT
			SPID.OP=pid_setup.groups.ctrl1.roll.kp*1000;
			//i
			pid_setup.groups.ctrl1.roll.ki =pid_setup.groups.ctrl1.pitch.ki =0.1;// 0.05;//0.1;
			SPID.OI=pid_setup.groups.ctrl1.roll.ki*1000;
			//d
			pid_setup.groups.ctrl1.roll.kd=pid_setup.groups.ctrl1.pitch.kd =4;// 2.0;//4;//2.0;
			SPID.OD=pid_setup.groups.ctrl1.roll.kd*1000;
			//--------------------------------------YAW-------------------------------------------------
			pid_setup.groups.ctrl2.yaw.kp   = 0.8;//0.4;//0.8;	
			SPID.YD=pid_setup.groups.ctrl2.yaw.kp*1000;
			pid_setup.groups.ctrl2.yaw.ki   = 0.05;	
			SPID.YI=pid_setup.groups.ctrl2.yaw.ki*1000;
			pid_setup.groups.ctrl2.yaw.kd   = 0.3;//0.1;//0.3;
			//------------------------------------
			pid_setup.groups.ctrl1.yaw.kp   = 0.8;//1.2;//0.8;
			SPID.YP=pid_setup.groups.ctrl1.yaw.kp*1000;	
			
			pid_setup.groups.ctrl1.yaw.ki   = 0.1;//1.0;//0.5;//0.1;	
			pid_setup.groups.ctrl1.yaw.kd   = 1.0;//1.0;//1.5;	
			pid_setup.groups.ctrl1.roll.kdamp=pid_setup.groups.ctrl1.pitch.kdamp = 1;
			pid_setup.groups.ctrl1.yaw.kdamp   = 1;
		 //--------------------------------------------光流定位----------------------------
				pid.nav.out.p=4;////0.35;
				pid.nav.out.i=0;//8;//(wt)
				pid.nav.in.p =0.3;//3.5;//0.365;//0.350;//0.4;//fix 2015.10.21 导航均值滤波 阴天//0.025;//150
				pid.nav.in.i =0.01;

				pid.nav.in.d =0.2;//0.250;
				pid.nav.out.dead= 0.4;//位置死区mm
				pid.nav.in.dead=  0.025;//3;//速度死区mm/s
				pid.nav.in.dead2=  pid.nav.in.dead*1.5;//4.5;//速度积分死区mm/s
		 //-----------------------------壁障----------------------------------
			pid.avoid.out.p=0.1;
		 //----------------------------ESO------------------------------------
			eso_att_outter[PITr].n=eso_att_outter[ROLr].n=eso_att_outter[YAWr].n=0;//10;
			eso_att_inner[PITr].n=eso_att_inner[ROLr].n=eso_att_inner[YAWr].n=0;
	
	#else //100HZ
	
			#if WIN8 //8045
			pid_setup.groups.ctrl2.roll.kp=pid_setup.groups.ctrl2.pitch.kp =0.425;//0.4;//0.35;//28;//0.425;//0.425;//<-----------mini0.45 for 4050  0.4 for 6030 WT
			SPID.IP=pid_setup.groups.ctrl2.roll.kp*1000;
			//i
			pid_setup.groups.ctrl2.roll.ki =pid_setup.groups.ctrl2.pitch.ki = 0.00;//5;//5;//0.15;//0.25;//0.05;
			SPID.II=pid_setup.groups.ctrl2.roll.ki*1000;
			//d
			pid_setup.groups.ctrl2.roll.kd  = pid_setup.groups.ctrl2.pitch.kd =0.5;//0.3;//0.3;//2;//0.425;//35;//<-------------mini
			SPID.ID=pid_setup.groups.ctrl2.roll.kd*1000;
			//---------------------------------------GRO--------------------------------
			//p
			pid_setup.groups.ctrl1.roll.kp=pid_setup.groups.ctrl1.pitch.kp = 0.425;//0.65;//0.3;//0.425;//0.35;//<-----------mini                                                                               WT
			SPID.OP=pid_setup.groups.ctrl1.roll.kp*1000;
			//i
			pid_setup.groups.ctrl1.roll.ki =pid_setup.groups.ctrl1.pitch.ki =0.1;// 0.05;//0.1;
			SPID.OI=pid_setup.groups.ctrl1.roll.ki*1000;
			//d
			pid_setup.groups.ctrl1.roll.kd=pid_setup.groups.ctrl1.pitch.kd =8;// 2.0;//4;//2.0;
			SPID.OD=pid_setup.groups.ctrl1.roll.kd*1000;
			//--------------------------------------YAW-------------------------------------------------
			pid_setup.groups.ctrl2.yaw.kp   = 0.8;//0.4;//0.8;	
			SPID.YD=pid_setup.groups.ctrl2.yaw.kp*1000;
			pid_setup.groups.ctrl2.yaw.ki   = 0.05;	
			SPID.YI=pid_setup.groups.ctrl2.yaw.ki*1000;
			pid_setup.groups.ctrl2.yaw.kd   = 0.3;//0.1;//0.3;
			//------------------------------------
			pid_setup.groups.ctrl1.yaw.kp   = 0.8;//1.2;//0.8;
			SPID.YP=pid_setup.groups.ctrl1.yaw.kp*1000;	
			
			pid_setup.groups.ctrl1.yaw.ki   = 0.1;//1.0;//0.5;//0.1;	
			pid_setup.groups.ctrl1.yaw.kd   = 1.0;//1.0;//1.5;	
			pid_setup.groups.ctrl1.roll.kdamp=pid_setup.groups.ctrl1.pitch.kdamp = 1;
			pid_setup.groups.ctrl1.yaw.kdamp   = 1;
		 //--------------------------------------------光流定位----------------------------
				pid.nav.out.p=4;////0.35;
				pid.nav.out.i=0;//8;//(wt)
				pid.nav.in.p =0.65;//3.5;//0.365;//0.350;//0.4;//fix 2015.10.21 导航均值滤波 阴天//0.025;//150
				pid.nav.in.i =0.01;

				pid.nav.in.d =0.2;//0.250;
				pid.nav.out.dead= 0.4;//位置死区mm
				pid.nav.in.dead=  0.025;//3;//速度死区mm/s
				pid.nav.in.dead2=  pid.nav.in.dead*1.5;//4.5;//速度积分死区mm/s
		 //-----------------------------壁障----------------------------------
			pid.avoid.out.p=0.1;
		 //----------------------------ESO------------------------------------
			eso_att_outter[PITr].n=eso_att_outter[ROLr].n=eso_att_outter[YAWr].n=15;//10;
			eso_att_inner[PITr].n=eso_att_inner[ROLr].n=eso_att_inner[YAWr].n=66;
			#else //9045
			pid_setup.groups.ctrl2.roll.kp=pid_setup.groups.ctrl2.pitch.kp =0.4;//0.35;//0.425;//0.66;//0.35;//28;//0.425;//0.425;//<-----------mini0.45 for 4050  0.4 for 6030 WT
			SPID.IP=pid_setup.groups.ctrl2.roll.kp*1000;
			//i
			pid_setup.groups.ctrl2.roll.ki =pid_setup.groups.ctrl2.pitch.ki = 0.05;//5;//5;//0.15;//0.25;//0.05;
			SPID.II=pid_setup.groups.ctrl2.roll.ki*1000;
			//d
			pid_setup.groups.ctrl2.roll.kd  = pid_setup.groups.ctrl2.pitch.kd =0.8;//0.5;//0.3;//0.3;//2;//0.425;//35;//<-------------mini
			SPID.ID=pid_setup.groups.ctrl2.roll.kd*1000;
			//---------------------------------------GRO--------------------------------
			//p
			pid_setup.groups.ctrl1.roll.kp=pid_setup.groups.ctrl1.pitch.kp = 0.5;//0.35;//0.425;//0.466;//0.3;//0.425;//0.35;//<-----------mini                                                                               WT
			SPID.OP=pid_setup.groups.ctrl1.roll.kp*1000;
			//i
			pid_setup.groups.ctrl1.roll.ki =pid_setup.groups.ctrl1.pitch.ki =0.05;//0.1;// 0.05;//0.1;
			SPID.OI=pid_setup.groups.ctrl1.roll.ki*1000;
			//d
			pid_setup.groups.ctrl1.roll.kd=pid_setup.groups.ctrl1.pitch.kd =8;//4;//8;// 2.0;//4;//2.0;
			SPID.OD=pid_setup.groups.ctrl1.roll.kd*1000;
			//--------------------------------------YAW-------------------------------------------------
			pid_setup.groups.ctrl2.yaw.kp   = 0.8;//0.4;//0.8;	
			SPID.YD=pid_setup.groups.ctrl2.yaw.kp*1000;
			pid_setup.groups.ctrl2.yaw.ki   = 0.05;	
			SPID.YI=pid_setup.groups.ctrl2.yaw.ki*1000;
			pid_setup.groups.ctrl2.yaw.kd   = 0.3;//0.1;//0.3;
			//------------------------------------
			pid_setup.groups.ctrl1.yaw.kp   = 0.8;//1.2;//0.8;
			SPID.YP=pid_setup.groups.ctrl1.yaw.kp*1000;	
			
			pid_setup.groups.ctrl1.yaw.ki   = 0.1;//1.0;//0.5;//0.1;	
			pid_setup.groups.ctrl1.yaw.kd   = 1.0;//1.0;//1.5;	
			pid_setup.groups.ctrl1.roll.kdamp=pid_setup.groups.ctrl1.pitch.kdamp = 1;
			pid_setup.groups.ctrl1.yaw.kdamp   = 1;
		 //--------------------------------------------光流定位----------------------------
				pid.nav.out.p=2;//4;//5;////0.35;
				pid.nav.out.i=0;//8;//(wt)
				pid.nav.in.p =0.55;//3.5;//0.365;//0.350;//0.4;//fix 2015.10.21 导航均值滤波 阴天//0.025;//150
				pid.nav.in.i =0.01;

				pid.nav.in.d =1;//0.250;
				pid.nav.out.dead= 0.01;//位置死区mm
				pid.nav.in.dead=  0.036;//3;//速度死区mm/s
				pid.nav.in.dead2=  pid.nav.in.dead*1.5;//4.5;//速度积分死区mm/s
		 //-----------------------------壁障----------------------------------
			pid.avoid.out.p=0.06;
		 //----------------------------ESO------------------------------------
//			eso_att_outter_c[PITr].n=eso_att_outter_c[ROLr].n=eso_att_outter_c[YAWr].n=18;//100;//13;//10;need tuning
//			eso_att_inner_c[PITr].n=eso_att_inner_c[ROLr].n=eso_att_inner_c[YAWr].n=0;
			#endif
	
	
	
	#endif
		break;
		}
	

//#endif
  memcpy(&ctrl_1.PID[PIDROLL],&pid_setup.groups.ctrl1.roll,sizeof(pid_t));
	memcpy(&ctrl_1.PID[PIDPITCH],&pid_setup.groups.ctrl1.pitch,sizeof(pid_t));
	memcpy(&ctrl_1.PID[PIDYAW],&pid_setup.groups.ctrl1.yaw,sizeof(pid_t));
	
	
	memcpy(&ctrl_2.PID[PIDROLL],&pid_setup.groups.ctrl2.roll,sizeof(pid_t));
	memcpy(&ctrl_2.PID[PIDPITCH],&pid_setup.groups.ctrl2.pitch,sizeof(pid_t));
	memcpy(&ctrl_2.PID[PIDYAW],&pid_setup.groups.ctrl2.yaw,sizeof(pid_t));
}

void Para_Init()
{
	Param_SetSettingToFC();
	Ctrl_Para_Init();
	WZ_Speed_PID_Init();
	Ultra_PID_Init();
}



