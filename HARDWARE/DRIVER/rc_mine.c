#include "../HARDWARE/DRIVER/rc_mine.h"
#include "../HARDWARE/MEMS/mpu6050.h"
#include "../HARDWARE/include.h"
#include "../HARDWARE/parameter.h"
#include "../HARDWARE/CONTROL/height_ctrl.h"
#include "../HARDWARE/CONTROL/eso.h"
#include "../HARDWARE/CONTROL/att.h"
s8 CH_in_Mapping[CH_NUM] = {0,1,2,3,4,5,6,7};    //通道映射
u16 RX_CH[CH_NUM];
u16 RX_CH_PWM[CH_NUM];
int RX_CH_FIX[4];
int RX_CH_FIX_PWM[4];
void CH_Mapping_Fun(u16 *in,u16 *Mapped_CH)
{
	u8 i;
	for( i = 0 ; i < CH_NUM ; i++ )
	{
		*( Mapped_CH + i ) = *( in + CH_in_Mapping[i] );
	}
}

s16 CH[CH_NUM];

float CH_Old[CH_NUM];
float CH_filter[CH_NUM];
float CH_filter_Old[CH_NUM];
float CH_filter_D[CH_NUM];
u8 NS=0,CH_Error[CH_NUM];
u16 NS_cnt,CLR_CH_Error[CH_NUM];
 
s16 MAX_CH[CH_NUM]  = {1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 };	//摇杆最大
s16 MIN_CH[CH_NUM]  = {1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 };	//摇杆最小
char CH_DIR[CH_NUM] = {0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    };  //摇杆方向
#define CH_OFFSET 500


float filter_A;

void RC_Duty( float T , u16 tmp16_CH[CH_NUM] )
{
	u8 i;
	s16 CH_TMP[CH_NUM];
	static u16 Mapped_CH[CH_NUM];

	if( NS == 1 )
	{
		CH_Mapping_Fun(tmp16_CH,Mapped_CH);
	}
	else if( NS == 2 )
	{
		#if USE_RECIVER_MINE	
		CH_Mapping_Fun(RX_CH,Mapped_CH);
		#else
		CH_Mapping_Fun(RX_CH_PWM,Mapped_CH);
		#endif
	}
	
	for( i = 0;i < CH_NUM ; i++ )
	{
		if( (u16)Mapped_CH[i] > 2500 || (u16)Mapped_CH[i] < 500 )
		{
			CH_Error[i]=1;
			CLR_CH_Error[i] = 0;
		}
		else
		{
			CLR_CH_Error[i]++;
			if( CLR_CH_Error[i] > 200 )
			{
				CLR_CH_Error[i] = 2000;
				CH_Error[i] = 0;
			}
		}

		if( NS == 1 || NS == 2 )
		{
			if( CH_Error[i] ) //单通道数据错误
			{
				
			}
			else
			{
				//CH_Max_Min_Record();
				CH_TMP[i] = ( Mapped_CH[i] ); //映射拷贝数据，大约 1000~2000
				
				if( MAX_CH[i] > MIN_CH[i] )
				{
					if( !CH_DIR[i] )
					{
						CH[i] =   LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), -500, 500); //归一化，输出+-500
					}
					else
					{
						CH[i] = - LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), -500, 500); //归一化，输出+-500
					}
				}	
				else
				{
					fly_ready = 0;
				}
			}
		}	
		else //未接接收机或无信号（遥控关闭或丢失信号）
		{

		}
//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter =================================== 		
			
			filter_A = 3.14f *36 *T;
			
			if( ABS(CH_TMP[i] - CH_filter[i]) <100 )
			{
				CH_filter[i] += filter_A *(CH[i] - CH_filter[i]) ;
			}
			else
			{
				CH_filter[i] += 0.5f *filter_A *( CH[i] - CH_filter[i]) ;
			}
// 					CH_filter[i] = Fli_Tmp;
			CH_filter_D[i] 	= ( CH_filter[i] - CH_filter_Old[i] );
			CH_filter_Old[i] = CH_filter[i];
			CH_Old[i] 		= CH[i];
	}
	//======================================================================
//	Fly_Ready(T);		//解锁判断
	//======================================================================
	if(++NS_cnt>600)  // 400ms  未插信号线。
	{
		NS_cnt = 0;
		NS = 0;
	}
}

u8 fly_ready = 0;
s16 ready_cnt=0;

void Fly_Ready(float T)
{
	if( CH_filter[2] < -400 )  							//油门小于10%
	{
		if( fly_ready && ready_cnt != -1 ) //解锁完成，且已退出解锁上锁过程
		{
			//ready_cnt += 1000 *T;
		}
#if(USE_TOE_IN_UNLOCK)		
		if( CH_filter[3] < -400 )							
		{
			if( CH_filter[1] > 400 )
			{
				if( CH_filter[0] > 400 )
				{
					if( ready_cnt != -1 )				   //外八满足且退出解锁上锁过程
					{
						ready_cnt += 3 *1000 *T;
					}
				}

			}

		}
#else
		if( CH_filter[3] < -400 )					      //左下满足		
		{
			if( ready_cnt != -1 && fly_ready )	//判断已经退出解锁上锁过程且已经解锁
			{
				ready_cnt += 1000 *T;
			}
		}
		else if( CH_filter[3] > 400 )      			//右下满足
		{
			if( ready_cnt != -1 && !fly_ready )	//判断已经退出解锁上锁过程且已经上锁
			{
				ready_cnt += 1000 *T;
			}
		}
#endif		
		else if( ready_cnt == -1 )						//4通道(CH[3])回位
		{
			ready_cnt=0;
		}
	}
	else
	{
		ready_cnt=0;
	}

	
	if( ready_cnt > 1000 ) // 1000ms 
	{
		ready_cnt = -1;
		if( !fly_ready )
		{
			fly_ready = 1;
			mpu6050.Gyro_CALIBRATE = 2;
		}
		else
		{
			fly_ready = 0;
		}
	}

}

void Feed_Rc_Dog(u8 ch_mode) //400ms内必须调用一次
{
	NS = ch_mode;
	NS_cnt = 0;
}


u8 height_ctrl_mode = 0,height_ctrl_mode_use;
extern u8 ultra_ok;//超声波正常标志位
void Mode_FC(void)
{static u8 last_height_ctrl_mode;
 #if USE_RECIVER_MINE		 //使用我的手柄   未使用
		if( RX_CH[AUX4r] < 100 )
		{
			height_ctrl_mode = 0;
		}
		else if(RX_CH[AUX4r] >800  )
		{
			height_ctrl_mode = 1;//气压计
		}
		else
		{
			if(ultra_ok == 1)
			{
				height_ctrl_mode = 2;//超声波
			}
			else
			{
				height_ctrl_mode = 1;
			}
		}	
#else
    //定高模式判断
		if( RX_CH_PWM[AUX4r] >1900 )
		{
				if(ultra_ok == 1)
			{
				height_ctrl_mode = 2;//超声波
			}
			else
			{
				height_ctrl_mode = 1;//气压计
			}
		}
		else if(RX_CH_PWM[AUX4r] >1400 &&RX_CH_PWM[AUX4r] <1600 )
		{
			height_ctrl_mode = 1;//气压计
		}
		else if(RX_CH_PWM[AUX4r] <1200 )
		{
			
				height_ctrl_mode = 0;//手动
		
		}
		
		//定点模式判断
		if(RX_CH_PWM[AUX3r]>1800)		
		mode.flow_hold_position=2;	//智能
		else if(RX_CH_PWM[AUX3r]<1400)
		mode.flow_hold_position=0;  //手动
    else
		mode.flow_hold_position=1;	//光流		
#endif	
}





#define RX_DR			6		
#define TX_DS			5
#define MAX_RT		4
u8 	NRF24L01_RXDATA[RX_PLOAD_WIDTH];		
u8 	NRF24L01_TXDATA[RX_PLOAD_WIDTH];		
vs16 QH,ZY,XZ;
u8 is_lock=1;
u8 EN_FIX_GPSF=0;
u8 EN_FIX_LOCKWF=0;
u8 EN_CONTROL_IMUF=0;
u8 EN_FIX_INSF=0;
u8 EN_FIX_HIGHF=0;
u8 tx_lock=1;
u8 EN_FIX_GPS=0;
u8 EN_FIX_LOCKW=0;
u8 EN_CONTROL_IMU=0;
u8 EN_FIX_INS=0;
u8 EN_FIX_HIGH=0;
u8 EN_TX_GX=1;
u8 EN_TX_AX=1;
u8 EN_TX_HM=1;
u8 EN_TX_YRP=1;
u8 EN_TX_GPS=1;
u8 EN_TX_HIGH=1;
u8 up_load_set=0;
u8 up_load_pid=0;
u8 key_rc[6]={1,1,1,1,1,1};
u16 Yaw_sb_rc=0;
u8 cnt_rst=0,delta_pitch=0,delta_roll=0,delta_yew=0;

//中值滤波
float GetMedianNum(float * bArray, u16 iFilterLen)
{  
    int i,j;// 循环变量  
    float bTemp;  
      
    // 用冒泡法对数组进行排序  
    for (j = 0; j < iFilterLen - 1; j ++)  
    {  
        for (i = 0; i < iFilterLen - j - 1; i ++)  
        {  
            if (bArray[i] > bArray[i + 1])  
            {  
                // 互换  
                bTemp = bArray[i];  
                bArray[i] = bArray[i + 1];  
                bArray[i + 1] = bTemp;  
            }  
        }  
    }  
      
    // 计算中值  
    if ((iFilterLen & 1) > 0)  
    {  
        // 数组有奇数个元素，返回中间一个元素  
        bTemp = bArray[(iFilterLen + 1) / 2];  
    }  
    else  
    {  
        // 数组有偶数个元素，返回中间两个元素平均值  
        bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2;  
    }  
  
    return bTemp;  
}  
u16 data_rate;
#define MID_RC_KEY 15
#define MID_RC_GET 4
u8 key_rc_reg[7][MID_RC_KEY];
float RC_GET[4][MID_RC_GET];
float control_scale=1;
float ypr_sb[3];
u8 loss_nrf=1;
u32 cnt_loss_nrf;
void NRF_DataAnl(void)// NRF 解码函数
{ 
int16_t RC_GET_TEMP[4];
float RC_GETR[4][MID_RC_GET];	
u8 temp_key[7];
u8 temp;
	u8 i=0,j,sum = 0;
	for( i=0;i<31;i++)
		sum += NRF24L01_RXDATA[i];
	if(!(sum==NRF24L01_RXDATA[31]))	
	{i=0;
		return;
		}	
	if(!(NRF24L01_RXDATA[0]==0x8A))		
		{
			i=0;
		return;
			}	
	
	if(NRF24L01_RXDATA[1]==0x8A)								
	{ 
		data_rate++;
		loss_nrf=0;
		RC_GET_TEMP[0]= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		Rc_Get.THROTTLE=  Moving_Median(25,3,(vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4]);
		Rc_Get.YAW			= Moving_Median(26,3,(vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6]);
		Rc_Get.PITCH		= Moving_Median(27,3,(vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8]);
		Rc_Get.ROLL 		= Moving_Median(28,3,(vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10]);
		RC_GET_TEMP[1]= (vs16)((NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6])/3+1000;
		RC_GET_TEMP[2]= (vs16)((NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8])/3+1000;	
		RC_GET_TEMP[3]= (vs16)((NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10])/3+1000;
		RX_CH[AUX1r]=Rc_Get.AUX1			= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];
		RX_CH[AUX4r]=Rc_Get.AUX4			= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		RX_CH[AUX3r]=Rc_Get.AUX3			= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		RX_CH[AUX2r]=Rc_Get.AUX2			= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];
		Rc_Get.AUX5			= (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];

		ctrl_angle_offset.x =(float)(Rc_Get.AUX1-500)/1000.*MAX_FIX_ANGLE*2;
		ctrl_angle_offset.y =(float)(Rc_Get.AUX2-500)/1000.*MAX_FIX_ANGLE*2;

		if(fabs(ctrl_angle_offset.x )<0.2)  {ctrl_angle_offset.x =0;}
	 	if(fabs(ctrl_angle_offset.y )<0.2)  {ctrl_angle_offset.y =0;}
		
	  RX_CH[THRr]=	Rc_Get.THROTTLE-RX_CH_FIX[THRr]	;
	  RX_CH[ROLr]=  my_deathzoom_rc(Rc_Get.ROLL-RX_CH_FIX[ROLr],100)	;
	  RX_CH[PITr]=  my_deathzoom_rc(Rc_Get.PITCH-RX_CH_FIX[PITr],100)	;
		
		KEY_SEL[0]=(NRF24L01_RXDATA[21])&0x01;
		KEY_SEL[1]=(NRF24L01_RXDATA[21]>>1)&0x01;
		KEY_SEL[2]=(NRF24L01_RXDATA[21]>>2)&0x01;
		KEY_SEL[3]=(NRF24L01_RXDATA[21]>>3)&0x01; 
	  KEY[0]=(NRF24L01_RXDATA[22])&0x01;
		KEY[1]=(NRF24L01_RXDATA[22]>>1)&0x01;
		KEY[2]=(NRF24L01_RXDATA[22]>>2)&0x01;
		KEY[3]=(NRF24L01_RXDATA[22]>>3)&0x01;
		KEY[4]=(NRF24L01_RXDATA[22]>>4)&0x01;
		KEY[5]=(NRF24L01_RXDATA[22]>>5)&0x01;
		KEY[6]=(NRF24L01_RXDATA[22]>>6)&0x01;
		KEY[7]=(NRF24L01_RXDATA[22]>>7)&0x01;
		
		ypr_sb[0]=(int)(((NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24]))/100.;
		ypr_sb[1]=(int)(((NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26]))/100.;
		ypr_sb[2]=(int)(((NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28]))/100.;		
		for(j=0;j<3;j++)
		  if(ypr_sb[j]>360)
				ypr_sb[j]-=655.35;
		if(!mode.yaw_imu_control)	
		RX_CH[YAWr]=  Rc_Get.YAW-RX_CH_FIX[YAWr]	;
		else{	
		if(fabs( ypr_sb[2])>32&&fabs(ypr_sb[1])<15)	
		RX_CH[YAWr]=  limit_mine(ypr_sb[2]*10,500)	+1500;
		else
		RX_CH[YAWr]=1500;	
	}
		if(mode.dj_lock){
		if(fabs( ypr_sb[1]-17)>40&&fabs(ypr_sb[2])<15)	
		{dj_angle_set+= (ypr_sb[1]-17)*0.008;
		dj_angle_set=LIMIT(dj_angle_set,-12,12);
		}
	}
		else
		dj_angle_set=0;	
//-------------------------------------------------------------------------------------------------------------------------		
	 }
	else if(NRF24L01_RXDATA[1]==0x8B)	//手机APP模式							
	{
			tx_lock=(NRF24L01_RXDATA[3]);
			EN_FIX_GPS=(NRF24L01_RXDATA[4]);
			EN_FIX_LOCKW=(NRF24L01_RXDATA[5]);
			EN_CONTROL_IMU=(NRF24L01_RXDATA[6]);
			EN_FIX_INS=(NRF24L01_RXDATA[7]);
			EN_FIX_HIGH=(NRF24L01_RXDATA[8]);
			EN_TX_GX=(NRF24L01_RXDATA[9]);
			EN_TX_AX=(NRF24L01_RXDATA[10]);
			EN_TX_HM=(NRF24L01_RXDATA[11]);
			EN_TX_YRP=(NRF24L01_RXDATA[12]);
			EN_TX_GPS=(NRF24L01_RXDATA[13]);
			EN_TX_HIGH=(NRF24L01_RXDATA[14]);
			(up_load_set)=(NRF24L01_RXDATA[15]);
			(up_load_pid)=(NRF24L01_RXDATA[16]);
		
			EN_FIX_GPSF=EN_FIX_GPS;
			EN_FIX_LOCKWF=EN_FIX_GPS;
			EN_CONTROL_IMUF=EN_FIX_GPS;
			EN_FIX_INSF=EN_FIX_GPS;
			EN_FIX_HIGHF=EN_FIX_GPS;	
	}
	else 		if(NRF24L01_RXDATA[1]==0x8C)	//APP  PID写入选择
	{
		  if(mode.en_pid_sb_set){
			SPID.OP = (float)((vs16)(NRF24L01_RXDATA[4]<<8)|NRF24L01_RXDATA[5]);
			SPID.OI = (float)((vs16)(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7]);
			SPID.OD = (float)((vs16)(NRF24L01_RXDATA[8]<<8)|NRF24L01_RXDATA[9]);
		  SPID.IP = (float)((vs16)(NRF24L01_RXDATA[10]<<8)|NRF24L01_RXDATA[11]);
			SPID.II = (float)((vs16)(NRF24L01_RXDATA[12]<<8)|NRF24L01_RXDATA[13]);
			SPID.ID = (float)((vs16)(NRF24L01_RXDATA[14]<<8)|NRF24L01_RXDATA[15]);
		  SPID.YP = (float)((vs16)(NRF24L01_RXDATA[16]<<8)|NRF24L01_RXDATA[17]);
			SPID.YI = (float)((vs16)(NRF24L01_RXDATA[18]<<8)|NRF24L01_RXDATA[19]);
			SPID.YD = (float)((vs16)(NRF24L01_RXDATA[20]<<8)|NRF24L01_RXDATA[21]);
				
				
						if(mode.att_pid_tune){//PID调参模式

						ctrl_2.PID[PIDPITCH].kp =ctrl_2.PID[PIDROLL].kp  = 0.001*SPID.OP;
						ctrl_2.PID[PIDPITCH].ki =ctrl_2.PID[PIDROLL].ki  = 0.001*SPID.OI;
						ctrl_2.PID[PIDPITCH].kd =ctrl_2.PID[PIDROLL].kd  = 0.001*SPID.OD;
						ctrl_1.PID[PIDPITCH].kp =ctrl_1.PID[PIDROLL].kp  = 0.001*SPID.IP;
						ctrl_1.PID[PIDPITCH].ki =ctrl_1.PID[PIDROLL].ki  = 0.001*SPID.II;
						ctrl_1.PID[PIDPITCH].kd =ctrl_1.PID[PIDROLL].kd  = 0.01*SPID.ID;
					 // ctrl_1.FB	= 0.001*SPID.YI;	
						//		ctrl_2.PID[PIDYAW].kp 	= 0.001*SPID.YP;
						//		ctrl_2.PID[PIDYAW].ki 	= 0.001*SPID.YI;
						//		ctrl_2.PID[PIDYAW].kd 	= 0.001*SPID.YD;
		
						eso_att_outter_c[PITr].b0=eso_att_outter_c[ROLr].b0= SPID.YD;//姿态控制自抗扰b0
						eso_att_inner_c[PITr].b0=eso_att_inner_c[ROLr].b0= SPID.YI;//角速度控制自抗扰b0	
						if(SPID.YP!=0)
						att_tuning_thr_limit=SPID.YP;//  PID调试时油门限制
						else
						att_tuning_thr_limit=450;	
						}
	
						//------------7 6 5 4  |  3 2 1 0  KEY
						//height  11   定高PID
						 else if(KEY[0]==1&&KEY[1]==1){
							ultra_pid.kp =  		0.001*(float)SPID.OP;
							ultra_pid.ki =  		0.001*(float)SPID.OI;
							ultra_pid.kd = 			0.001*(float)SPID.OD;
							wz_speed_pid.kp =   0.001*(float)SPID.IP; 
							wz_speed_pid.ki =   0.001*(float)SPID.II;
							wz_speed_pid.kd =   0.001*(float)SPID.ID;	 
							eso_att_inner_c[THRr].b0= SPID.YD;//定高内环ESO b0

				//			ultra_pid_safe.kp =  		 0.001*(float)SPID.OP;
				//			ultra_pid_safe.ki =  		 0.001*(float)SPID.OI;
				//			ultra_pid_safe.kd = 		 0.001*(float)SPID.OD;
				//			wz_speed_pid_safe.kp =   0.001*(float)SPID.IP; 
				//			wz_speed_pid_safe.ki =   0.001*(float)SPID.II;
				//			wz_speed_pid_safe.kd =   0.001*(float)SPID.ID;
						}
						else{//定点PID
							nav_pos_pid.kp=0.001*(float)SPID.OP;//0.2;
							nav_pos_pid.ki=0.001*(float)SPID.OI;//0.0;
							nav_pos_pid.kd=0.001*(float)SPID.OD;//0.0;
							nav_pos_pid.dead=(float)SPID.YI/1000.;//0.02;
							
							nav_spd_pid.f_kp=(float)SPID.YP/1000.;
							nav_spd_pid.kp=0.001*(float)SPID.IP;//0.2;
							nav_spd_pid.ki=0.001*(float)SPID.II;//0.01;
							nav_spd_pid.kd=0.001*(float)SPID.ID;//0.05;
							nav_spd_pid.dead=(float)SPID.YD;//20;
							//-----------------OLD--------------------
							pid.nav.out.p= 0.01*(float)SPID.OP;
							pid.nav.out.i= 0.01*(float)SPID.OI;
							pid.nav.out.d= 0.001*(float)SPID.OD;
							
							pid.nav.in.p = 0.001*(float)SPID.IP;
							pid.nav.in.i = 0.001*(float)SPID.II;
							pid.nav.in.d = 0.001*(float)SPID.ID;
						
							pid.nav.out.dead= (float)SPID.YP/1000.;//位置死区mm
							pid.nav.in.dead=  (float)SPID.YI/1000.;//速度死区mm/s
							pid.nav.in.fp = (float)SPID.YD/1000.;
						}
					//<---------------可自行按上面的方式添加	
						
			}
		}
	else 		if(NRF24L01_RXDATA[1]==0x8D)	//未使用
		{
			HPID.OP = (float)((vs16)(NRF24L01_RXDATA[4]<<8)|NRF24L01_RXDATA[5]);
			HPID.OI = (float)((vs16)(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7]);
			HPID.OD = (float)((vs16)(NRF24L01_RXDATA[8]<<8)|NRF24L01_RXDATA[9]);
		}
}


