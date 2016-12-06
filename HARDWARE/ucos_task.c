#include "../HARDWARE/include.h" 
#include "../HARDWARE/SYS_IDENT/ident.h"
#include "../HARDWARE/ucos_task.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
u8 read_rc_flag;
float YawR,PitchR,RollR;	
float fRPY[3] = {0};
float inner_loop_time,inner_loop_time_time,inner_loop_time_imu;
u16 Rc_Pwm_In[8];
//定时器3中断服务函数
float GET_T_INNER_TIM_USE1;
u32 rc_read_interupt[2];
void TIM3_IRQHandler(void)//     400Hz控制时内环中断
{static u8 cnt,cnt1,cnt_init,init;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
	#if EN_TIM_INNER
	inner_loop_time_time = Get_Cycle_T(GET_T_INNER_TIM); 						//获取内环准确的执行周期	
	if(!init){
	if(cnt_init++>10)
	init=1;
	inner_loop_time_time=0.0025;
	}
	else{
	#if EN_ATT_CAL_FC
	MPU6050_Read(); 															//读取mpu6轴传感器
	MPU6050_Data_Prepare( inner_loop_time_time );	//mpu6轴传感器数据处理
	if(cnt++>=4){cnt=0;I2C_FastMode=0;ANO_AK8975_Read();I2C_FastMode=1;	}			  //获取电子罗盘数据	
	if(cnt1++>=4){cnt1=0;
	MS5611_ThreadNew();}													//读取气压计
	#endif		
	CTRL_1( inner_loop_time_time ); 							//内环控制					
	}
	#endif	
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清中断
}


//========================外环  任务函数============================
OS_STK INNER_TASK_STK[INNER_STK_SIZE];
float dj[2];
void inner_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;
  u8 i,j;	
 	while(1)
	{	
	float temp = Get_Cycle_T(GET_T_INNER_TIM);								
	if(temp>0)
	inner_loop_time_time=temp;
	else
	inner_loop_time_time=1.0/F_INNER;
	if(!init){if(cnt_init++>40)
		init=1;
	inner_loop_time_time=0.005;
	}
	else{
	#if EN_ATT_CAL_FC	                            //使用FC模块进行姿态解算
	MPU6050_Read(); 															
	MPU6050_Data_Prepare( inner_loop_time_time);			
	if(cnt++>=2){cnt=0;ANO_AK8975_Read();}			
	if(cnt1++>=2){cnt1=0;MS5611_ThreadNew();}	
	#endif
	
	#if !EN_TIM_INNER		
	RC_Duty( inner_loop_time_time , Rc_Pwm_In );//RC滤波		
	#endif	
	CTRL_1( inner_loop_time_time ); 	
  }
	delay_ms(F_INNER);
	}
}		


//========================外环  任务函数============================
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];

float outer_loop_time,outer_loop_time_C;
void outer_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;
  u8 i,j;	
 	while(1)
	{	float temp = Get_Cycle_T(GET_T_OUTTER_C);								
		if(temp>0)
		outer_loop_time_C=temp;
		else
		outer_loop_time_C=1.0/F_OUTTER;
	if(!init){if(cnt_init++>40)
		init=1;
	outer_loop_time_C=0.01;
	}
	else{
	#if EN_TIM_INNER		
	RC_Duty( outer_loop_time_C , Rc_Pwm_In );		
	#endif	
	//if(!mode.en_h_inf)
	CTRL_2( outer_loop_time_C ); 			//外环自抗扰PID										
	// else
	//h_inf_att_out(except_A.x,Roll);
#if EN_TIM_INNER	
	Thr_Ctrl(outer_loop_time_C);// 油门控制
#endif
  }
#if EN_TIM_INNER	
	delay_ms(5);
#else
	delay_ms(F_OUTTER);
	//OSTimeDly(10*OS_TICKS_PER_SEC/1000);
#endif	

	}
}		


//=======================姿态解算任务函数============================
OS_STK EKF_TASK_STK[EKF_STK_SIZE];
float ekf_loop_time;
float Pit_fc,Rol_fc,Yaw_fc,Yaw_fc_q;
int flag_hml[3]={1,-1,1};
void ekf_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
	float temp0,temp1,temp2,temp3;
 	while(1)
	{	
	if(!init){if(cnt_init++>40)
		init=1;
	 ekf_loop_time=0.01;
	}
	else
	{ float temp = Get_Cycle_T(GET_T_EKF);								
		if(temp>0)
		ekf_loop_time=temp;
		else
		ekf_loop_time=1.0/F_EKF;
			
	if(cnt1++>1){cnt1=0;
  	IMUupdate(0.5f *ekf_loop_time*2,my_deathzoom_2(mpu6050_fc.Gyro_deg.x,0.0), my_deathzoom_2(mpu6050_fc.Gyro_deg.y,0.0), 
		my_deathzoom_2(mpu6050_fc.Gyro_deg.z,0.5), mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z,
		ak8975_fc.Mag_Val.x,ak8975_fc.Mag_Val.y,ak8975_fc.Mag_Val.z,
		&temp0,&temp1,&Yaw_fc);		
				}//只计算航向
	
		MadgwickAHRSupdate(ekf_loop_time,my_deathzoom_2(mpu6050_fc.Gyro_deg.x,0.5)/57.3, my_deathzoom_2(mpu6050_fc.Gyro_deg.y,0.5)/57.3, 
		my_deathzoom_2(mpu6050_fc.Gyro_deg.z,0.5)/57.3,(float)mpu6050_fc.Acc.x/4096., (float)mpu6050_fc.Acc.y/4096., (float)mpu6050_fc.Acc.z/4096.,
		0,0,0,
		//ak8975_fc.Mag_Val.x*flag_hml[0],ak8975_fc.Mag_Val.y*flag_hml[1],ak8975_fc.Mag_Val.z*flag_hml[2],
		&Rol_fc,&Pit_fc,&Yaw_fc_q);//计算俯仰和横滚
		}

#if EN_TIM_INNER	
	delay_ms(5);
#else
	delay_ms(F_EKF);
#endif	
	}
}		


//========================位置控制  任务函数============================
OS_STK POS_TASK_STK[POS_STK_SIZE];
float Yaw_DJ,Pitch_DJ;	
void pos_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
 	while(1)
	{	
	if(cnt++>5-1){cnt=0;
	if(!mode.use_dji) 
	GPS_calc_poshold(0.01*5);//光流GPS定点
 }
	if(cnt1++>5-1){cnt1=0;
	if(!mode.use_dji)
	circle_control(0.01*5);//图像目标跟踪 未使用
 }
	if(cnt2++>0.01*10&&mode.en_sonar_avoid){cnt2=0;
		;//oldx_avoid();  //避障 未使用
	}
	delay_ms(F_POS);
	}
}		

//=========================射频  模式切换 任务函数======================
OS_STK NRF_TASK_STK[NRF_STK_SIZE];
void nrf_task(void *pdata)
{	static u8 KEY_REG[8];						 
	static u16 cnt,cnt2,cnt3;
	u8 i;
 	while(1)
	{
		//GET_T_INNER_TIM_USE1 = Get_Cycle_T(GET_T_INNER_TIM_USE); 	
		 if(cnt_loss_nrf++>1500/50){cnt_loss_nrf=1500/50+1;loss_nrf=1;}
		 if(imu_loss_cnt++>1500/50){imu_loss_cnt=1500/50+1;NAV_BOARD_CONNECT=0;}
	
		//---------------use now
		//------------0 1   |   2 3       KEY_SEL
		#if USE_RECIVER_MINE		
		mode.flow_hold_position=KEY_SEL[0];
    #else
    mode.en_sonar_avoid=KEY_SEL[0];		 
    #endif
		if(force_sd_save) 
		mode.en_sd_save=1;	
		else
		mode.en_sd_save=KEY_SEL[1];
		mode.en_pid_sb_set=KEY_SEL[2];//使能PID设置	
//-------------------------------------------------	
		#if  DEBUG_WITHOUT_SB
		if(cnt2++>200)//
		{fly_ready=1;cnt2=200+1;}
		#else
			#if !USE_RC_GROUND
				if(Rc_Get_PWM.AUX1>1500)
					fly_ready=1;
					else
					fly_ready=0;
			#else
			fly_ready=KEY_SEL[3];//解锁
			#endif
		#endif
					
	  //------------7 6 5 4  |  3 2 1 0  KEY
		//mode.trig_flow_spd= KEY[7];//1
		//mode.trig_h_spd=KEY[4];
	 
    mode.flow_f_use_ukfm=1;//KEY[7];
		//mode.baro_f_use_ukfm=1;//
		mode.en_eso_h_in=1;
	  mode.flow_d_acc=KEY[7];//光流速度环加速度D
		//mode.baro_lock=KEY[6];//气压侧飞锁定
		mode.yaw_sel=!KEY[3];
		//mode.att_ident1=KEY[4];
		//if(Rc_Get_PWM.AUX1>1500)
		//mode.height_safe=1;//mode.en_sd_save=1;
		//else
		//mode.height_safe=0;//	mode.en_sd_save=0;
		if(mode.flow_hold_position<1)
			mode.height_safe=1;
		else{
		#if USE_RC_GROUND
			if(Rc_Get_PWM.AUX1>1500)
			mode.height_safe=1;//mode.en_sd_save=1;
			else
			mode.height_safe=0;//	mode.en_sd_save=0;
		#endif	
			}
		mode.att_pid_tune=KEY[6]&&KEY[5]&&KEY[3]&&KEY[2]&&KEY[1]&&KEY[0];
		
		if(!mode.att_pid_tune){
		if(KEY[2]!=KEY_REG[2] && !fly_ready &&Thr_Low)//加速度计校准条件
		ak8975.Mag_CALIBRATED=1;
		}
		
		for(i=0;i<=7;i++)
		KEY_REG[i]=KEY[i];
		delay_ms(50);
	}
}		

//=======================气压计融合任务函数==================
float baro_task_time;
OS_STK BARO_TASK_STK[BARO_STK_SIZE];
void baro_task(void *pdata)
{	static u8 init;
  static u16 cnt_init;	
 	while(1)
	{
	if(!init){if(cnt_init++>40)
		init=1;
	 baro_task_time=0.020;
	}
	else
	{ float temp = Get_Cycle_T(GET_T_BARO_UKF);							
		if(temp>0)
		baro_task_time=temp;
		else
		baro_task_time=1.0/F_BARO;	
 
	  ukf_baro_task1(baro_task_time)	;
	}
	 delay_ms(F_BARO);
  }
}	

//=======================ROS 任务函数==================

OS_STK ROS_TASK_STK[ROS_STK_SIZE];
void ros_task(void *pdata)//ROS TASK
{	 static u8 dj_mode_reg;					  
 	while(1)
	{

	 delay_ms(50);
  }
}	

//=======================IDENT 任务函数==================

OS_STK IDENT_TASK_STK[IDENT_STK_SIZE];
void ident_task(void *pdata)//IDNET TASK
{	 static u8 dj_mode_reg;					  
 	while(1)
	{
		if(mode.att_ident1)
		;//ident();	//未使用
		delay_ms(10);
  }
}	

//=======================串口 任务函数===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 UART_UP_LOAD_SEL=26;//<------------------------------上传数据选择
u8 force_flow_ble_debug;
u8 state_test=26;
void uart_task(void *pdata)
{	static u8 cnt[5];					 		
 	while(1)
	{
				//To  Odroid 图像模块
				if(cnt[0]++>2){cnt[0]=0;
						#if EN_DMA_UART3 
					if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//等待DMA2_Steam7传输完成
								{ 
							DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA2_Steam7传输完成标志
							data_per_uart3();
						  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3);     //开始一次DMA传输！	
								}	
						#else
								UsartSend_GPS(state_test);//Send_IMU_TO_GPS();	
						#endif
							}			
				
				//To  IMU模块	
				if(cnt[1]++>1){cnt[1]=0;	
				  #if EN_DMA_UART2 					
					if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA2_Steam7传输完成
								{ 
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
							data_per_uart2();
					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     //开始一次DMA传输！	
								}	
					#else
								 GOL_LINK_TASK();	
					#endif
							}					
							
				//BLE UPLOAD《----------------------蓝牙调试
					
						if(cnt[4]++>0){cnt[4]=0;		
						#if USE_BLE_FOR_APP			  
						APP_LINK();
						#endif
						}
				if(cnt[2]++>1){cnt[2]=0;
					
					if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
							{ 	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
									#if !BLE_BAD
								    if(mode.att_pid_tune){//PID TUNING
											if(KEY[7])//OUTTER
											data_per_uart1(
											0,-except_A.x*10,0,
											#if EN_ATT_CAL_FC
											0,-Rol_fc*10,0,
											#else
											0,-Roll*10,0,
											#endif
											-ctrl_2.err.y*10,-eso_att_outter[ROLr].disturb*10,0,
											(int16_t)(eso_att_outter_c[ROLr].disturb_u*100),(int16_t)(0*10.0),(int16_t)(0*100.0),0/10,0,0/10,0*0);
											else//INNER
											data_per_uart1(
											0,-except_AS.x,0,
											0,-mpu6050.Gyro_deg.x,0,  
											-ctrl_1.err.y,-eso_att_inner[ROLr].disturb,0,
											(int16_t)(eso_att_outter_c[ROLr].disturb_u*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);	
										}
										else if(flow_debug.en_ble_debug||force_flow_ble_debug)//DEBUG  FLOW
											data_per_uart1(
											flow_debug.ax,flow_debug.ay,flow_debug.az,
										  flow_debug.gx,flow_debug.gy,flow_debug.gz,
										  flow_debug.hx,flow_debug.hy,flow_debug.hz,
											(int16_t)(inner_loop_time*10000.0),(int16_t)(outer_loop_time*10000.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);
										else{//DEBUG-------------------------Normal mode--------------------------------
								    switch(UART_UP_LOAD_SEL)
											{
											case 0://BMP UKF
											data_per_uart1(
											baroAlt/10,baroAlt_fc/10,ALT_POS_BMP_UKF_OLDX*100,
											ALT_VEL_BMP_UKF_OLDX*100,ALT_VEL_BMP_EKF*100,wz_speed/10,
											0*100,ALT_VEL_BMP_UKF_OLDX*100,0,
											//-accz_bmp*100,baro_matlab_data[1]/10,0*100,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_test,0,0/10,0);break;	
											case 1://GPS UKF
											data_per_uart1(
											0,imu_nav.gps.Y_UKF,0,
											0,imu_nav.gps.Y_O,0,  
											imu_nav.gps.J,0,0,
											(int16_t)(Yaw*10.0),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),0/10,0,0/10,0*0);break;	
											case 2://SONAR BMP SPEED
											data_per_uart1(
											0,ultra_speed/10,wz_speed/10,
											0,-ALT_VEL_BMP*100,0,  
											Yaw*10,0,0,
											(int16_t)(yaw_mag_view[0]*10.0),(int16_t)(yaw_mag_view[1]*10.0),(int16_t)(yaw_mag_view[2]*10),0/10,0,0/10,0*0);break;	
//											case 3://NEURON PID
//											data_per_uart1(
//											0,ctrl_2.out.y*10,0,
//											0,neuron_pid_outter[PITr].u*10,0,  
//											0,neuron_pid_outter[ROLr].u*10,0,
//											(int16_t)(thr_in_view*10.0),(int16_t)(thr_use*10.0),(int16_t)(ALT_POS_SONAR2*1000),0/10,0,0/10,0*0);break;	
											case 4://ESO PID OUT
											data_per_uart1(
											0,ctrl_2.out.y*10,0,
											0,eso_att_outter_c[PITr].u*10,0,  
											0,eso_att_outter_c[PITr].disturb_u*10,0,
											(int16_t)(except_A.y*10.0),(int16_t)(eso_att_outter_c[PITr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 5://ESO PID IN
											data_per_uart1(
											0,ctrl_1.out.y*10,0,
											0,eso_att_inner_c[PITr].u*10,0,  
											0,eso_att_inner_c[PITr].disturb_u*10,0,
											(int16_t)(except_AS.y*10.0),(int16_t)(eso_att_inner_c[PITr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;												
											case 6://FLOW
											data_per_uart1(
											0,imu_nav.flow.speed.x*100,imu_nav.flow.speed.x_f*100,
											0,imu_nav.flow.speed.y*100,imu_nav.flow.speed.y_f*100,  
											0,imu_nav.flow.speed.x_f*100,imu_nav.flow.speed.y_f*100,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 7://SONAR AVOID
											data_per_uart1(
											sonar_avoid[0],sonar_avoid[1],sonar_avoid[2],
											sonar_avoid[3],sonar_avoid[4],sonar_avoid[5],  
											sonar_avoid[6],sonar_avoid[7],0,
											(int16_t)(sonar_avoid_c[0]),(int16_t)(sonar_avoid_c[1]),(int16_t)(0*10),0/10,0,0/10,0*0);break;		
											case 8://ESO PID HIGH IN
											data_per_uart1(
											0,wz_speed_pid_v_view,ultra_ctrl.err,
											0,eso_att_inner_c[THRr].u,0,  
											0,wz_speed_pid_v.pid_out,0,
											(int16_t)(ultra_ctrl_out_use*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;	
											case 9://CIRCLE
											data_per_uart1(
											0,nav_circle[0],nav_circle[1],
											0,eso_att_inner_c[THRr].u,0,  
											0,wz_speed_pid_v.pid_out,0,
											(int16_t)(ultra_ctrl_out_use*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;		
											case 10://EKF vs ARSH
											data_per_uart1(
											0,Pitch*10,fRPY[1]*10,
											0,Roll*10,fRPY[0]*10,  
											0,Yaw*10,fRPY[2]*10,
											(int16_t)(ultra_ctrl_out_use*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;		
											case 12://CIRCLE
											data_per_uart1(
											0,0,circle.x_flp-MID_X,
											0,0,circle.y_flp-MID_Y,  
											0,nav_circle[0]*10,nav_circle[1]*10,
											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 13://H_inf vs PID
											data_per_uart1(
											0,0,ctrl_inf_att_out*10,
											0,0,ctrl_1.out.x,  
											0,0,ctrl_inf_att_out2*10,
											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 14://SONAR_KAL
											data_per_uart1(
											0,exp_height,ultra_distance,
											0,0,ALT_POS_SONAR2*1000,  
											0,ultra_ctrl_out_use,ultra_speed,
											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
//											case 15://DJI_RC
//											data_per_uart1(
//											0,DJI_RC[1],DJI_RC[0],
//											0,inner_loop_time_time*10000,DJI_RC[2],  
//											0,0,DJI_RC[3],
//											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 16://SONAR
											data_per_uart1(
											0,0,ultra_distance,
											0,0,ALT_POS_SONAR3*1000,  
											0,ultra_dis_lpf,ALT_POS_SONAR2*1000,
											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 17://FLOW_BREAK
											data_per_uart1(
											0,0,nav[ROLr]*10,
											0,0,nav[PITr]*10,  
											0,except_A.x*10,except_A.y*10,
											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 18://IMUVS
											data_per_uart1(
											0,0,fRPY[0]*10,
											0,0,fRPY[1]*10,  
											0,0,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 19://FLOW_POS
											data_per_uart1(
											0,0,target_position[LAT]*100,
											0,0,now_position[LAT]*100,
											0,0,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
										  case 20://H_SPD 
											data_per_uart1(
											0,Rc_Get.PITCH,Rc_Get.ROLL,
											0,Rc_Get.THROTTLE,eso_att_inner_c[THRr].z[1],
											0,wz_speed_pid_v.pid_out,wz_speed_pid_v_view,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 21://TRIG PID TUNNING 
											data_per_uart1(
											0,flow_matlab_data[0]*1000,flow_matlab_data[1]*1000,
											0,flow_matlab_data[2]*1000,flow_matlab_data[3]*1000,
											0,0,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 22://TRIG PID TUNNING 
											data_per_uart1(
											0,avoid_trace[0]*1,flow_matlab_data[1]*0,
											0,avoid_trace[1]*1,flow_matlab_data[3]*0,
											0,0,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 23://TRIG PID TUNNING 
											data_per_uart1(
											0,0,height_ctrl_out,
											0,ultra_speed/10,ultra_ctrl_out_use/10,
											0,0,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 24://TRIG PID TUNNING 
											data_per_uart1(
											0,0,d_flow_watch[0]*100,
											0,0,d_flow_watch[1]*100,
											0,acc_body[1]*100,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 25://TRIG PID TUNNING 
											data_per_uart1(
											yaw_mag_view[4]*10,yaw_mag_view[0]*10,yaw_mag_view[1]*10,
											yaw_mag_view[3]*10,0,0,
											X_kf_yaw[0]*10,yaw_kf*10,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 26://GPS Test 
											data_per_uart1(
											POS_UKF_X*100,POS_UKF_Y*100,0,
											VEL_UKF_X*100,VEL_UKF_Y*100,0,
											gpsx_o.posslnum,Yaw,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											
											default:break;
											}
										}
										#endif
							USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     //开始一次DMA传输！	  
							}	
						}
				
						
				//To  SD卡
						static u8 sd_sel;
				if(cnt[3]++>1){cnt[3]=0;
			     #if EN_DMA_UART4 			
					if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)
								{ 
							DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);
							
							if(KEY[3]&&!mode.en_sd_save)
							data_per_uart4(SEND_DEBUG);
						  else
							switch(sd_sel){
							case 0:sd_sel=1;		
							data_per_uart4(SEND_IMU);
							break;
							case 1:sd_sel=2;
							data_per_uart4(SEND_ALT);
							break;
							case 2:sd_sel=3;
							data_per_uart4(SEND_FLOW);
							break;
							case 3:sd_sel=4;
							data_per_uart4(SEND_GPS);
							break;
							case 4:sd_sel=0;
							data_per_uart4(SEND_DEBUG);
							
							}
							USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);    
							MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE4+2);   
								}		
					#else
							SD_LINK_TASK2(SEND_IMU);	
					#endif
							}		
				
			delay_ms(5);  
			}
}	

//=======================故障保护 任务函数==================
OS_STK ERROR_TASK_STK[ERROR_STK_SIZE];
void error_task(void *pdata)
{							  
 	while(1)
	{
	LEDRGB();//LED显示
	MEMS_CAL();//校准IMU模块传感器
	Mode_FC();//飞控模式切换	
		
	if(!fly_ready&&Thr_Low)//未使用   失控保护判断
	{ ero.ero_rst_att=ero.ero_rst_h=1; ero.ero_att=ero.ero_hight=0;}
  else	
	{
   att_ero_check();
   hight_ero_check();
	}
	
	if(app_connect_fc_loss++>4/0.5)
	app_connect_fc=0;
	
	
	if(circle.lose_cnt++>4/0.5)
	circle.connect=0;
	if(ultra_pid.kp==0||mode.height_safe)
		mode.height_in_speed=1;
	else
		mode.height_in_speed=0;
	
	 delay_ms(500); 
	}
}	

//------------------------------软件定时器  未使用----------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	  OSCPUUsage
//每100ms执行一次,用于显示CPU使用率和内存使用率		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{
 		
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}

//软件定时器2的回调函数	《----------------------------	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
static u8 cnt;
	
}

//软件定时器3的回调函数				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
} 


//