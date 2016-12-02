
#include "../HARDWARE/include.h"
#include "../HARDWARE/DRIVER/usart_fc.h"
#include "../HARDWARE/MEMS/ultrasonic.h"
#include "../HARDWARE/MEMS/hml_sample.h"
#include "../HARDWARE/MEMS/mpu6050.h"
#include "../HARDWARE/MEMS/ms5611_2.h"
#include "../HARDWARE/FUSHION/alt_kf.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/CONTROL/height_ctrl.h"
#include "../HARDWARE/CONTROL/visual_contol.h"
#include "../HARDWARE/error.h"
#include "../HARDWARE/CONTROL/sonar_avoid.h"


void Usart2_Init(u32 br_num)//--GOL-link
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}
RESULT color;
float dt_flow_rx;
u16 data_rate_gol_link;
PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct _PID_SET pid;
RC_GETDATA Rc_Get;
RC_GETDATA Rc_Get_PWM;
struct _plane plane;
struct _slam slam;
struct _IMU_NAV imu_nav;
struct _MODE mode;
u8 LOCK, KEY[8],KEY_SEL[4];
struct _FLOW_DEBUG flow_debug;
u8 NAV_BOARD_CONNECT=0;
u32 imu_loss_cnt;
float flow_matlab_data[4],baro_matlab_data[2];
float POS_UKF_X,POS_UKF_Y,VEL_UKF_X,VEL_UKF_Y;
int k_scale_pix;
 void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==0x01)//FLOW_MINE_frame
  { float temp_pos[2];
	  imu_loss_cnt=0;
    NAV_BOARD_CONNECT=1;
		imu_nav.flow.rate=*(data_buf+4);
		imu_nav.flow.speed.y_f=(float)((int16_t)((*(data_buf+5)<<8)|*(data_buf+6)))/1000.;
		imu_nav.flow.speed.x_f=(float)((int16_t)((*(data_buf+7)<<8)|*(data_buf+8)))/1000.;
		imu_nav.flow.speed.y = (float)(int16_t)((*(data_buf+9)<<8)|*(data_buf+10))/1000.;
		imu_nav.flow.speed.x = (float)(int16_t)((*(data_buf+11)<<8)|*(data_buf+12))/1000.;
		temp_pos[LON]=(float)(int16_t)((*(data_buf+13)<<8)|*(data_buf+14))/100.;//m  lon->0 X
		temp_pos[LAT]=(float)(int16_t)((*(data_buf+15)<<8)|*(data_buf+16))/100.;//m  lat->1 Y
		ALT_VEL_SONAR=(float)(int16_t)((*(data_buf+17)<<8)|*(data_buf+18))/1000.;//m
		float temp=(float)(int16_t)((*(data_buf+19)<<8)|*(data_buf+20))/1000.;//m
			if(temp<3.2)
	  ALT_POS_SONAR2 = temp;
		ALT_VEL_BMP=(float)(int16_t)((*(data_buf+21)<<8)|*(data_buf+22))/1000.;//m
		ALT_POS_BMP=(float)(int32_t)((*(data_buf+23)<<24)|(*(data_buf+24)<<16)|(*(data_buf+25)<<8)|*(data_buf+26))/1000.;//m
		//ALT_VEL_BMP_EKF=(float)(int16_t)((*(data_buf+27)<<8)|*(data_buf+28))/1000.;//m
		ALT_POS_BMP_EKF=(float)(int32_t)((*(data_buf+29)<<24)|(*(data_buf+30)<<16)|(*(data_buf+31)<<8)|*(data_buf+32))/1000.;//m
		POS_UKF_X=(float)(int16_t)((*(data_buf+33)<<8)|*(data_buf+34))/1000.;//m  ->0
		POS_UKF_Y=(float)(int16_t)((*(data_buf+35)<<8)|*(data_buf+36))/1000.;//m  ->1
		VEL_UKF_Y=(float)(int16_t)((*(data_buf+37)<<8)|*(data_buf+38))/1000.;//m
		VEL_UKF_X=(float)(int16_t)((*(data_buf+39)<<8)|*(data_buf+40))/1000.;//m			
				
    if(mode.flow_f_use_ukfm){
    now_position[LON]=POS_UKF_X;//m  lon->0 X
		now_position[LAT]=POS_UKF_Y;//m  lat->1 Y			
		}
    else
		{
		now_position[LON]=temp_pos[LON];//m  lon->0 X
		now_position[LAT]=temp_pos[LAT];//m  lat->1 Y			
		}			
		static u8 reg;
		if(mode.flow_f_use_ukfm!=reg)
		{
		integrator[0]=integrator[1]=0;
		target_position[LON]=now_position[LON];//m
		target_position[LAT]=now_position[LAT];//m
		}
		reg=mode.flow_f_use_ukfm;
    sys.flow=1;
	}		
  else if(*(data_buf+2)==0x12)//debug
  {
	flow_debug.en_ble_debug= *(data_buf+4);	
	flow_debug.ax=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	flow_debug.ay=((int16_t)(*(data_buf+7)<<8)|*(data_buf+8));
	flow_debug.az=((int16_t)(*(data_buf+9)<<8)|*(data_buf+10));
	flow_debug.gx=((int16_t)(*(data_buf+11)<<8)|*(data_buf+12));
	flow_debug.gy=((int16_t)(*(data_buf+13)<<8)|*(data_buf+14));
	flow_debug.gz=((int16_t)(*(data_buf+15)<<8)|*(data_buf+16));
	flow_debug.hx=((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
	flow_debug.hy=((int16_t)(*(data_buf+19)<<8)|*(data_buf+20));
  flow_debug.hz=((int16_t)(*(data_buf+21)<<8)|*(data_buf+22));

	}		
	else if(*(data_buf+2)==0x02)//CAL
  {
	  mpu6050.Acc_Offset.x=(int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
    mpu6050.Acc_Offset.y=(int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		mpu6050.Acc_Offset.z=(int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		mpu6050.Gyro_Offset.x=(int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		mpu6050.Gyro_Offset.y=(int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		mpu6050.Gyro_Offset.z=(int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		ak8975.Mag_Offset.x=(int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		ak8975.Mag_Offset.y=(int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		ak8975.Mag_Offset.z=(int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		ak8975.Mag_Gain.x=(float)((int16_t)(*(data_buf+22)<<8)|*(data_buf+23))/1000.;
		ak8975.Mag_Gain.y=(float)((int16_t)(*(data_buf+24)<<8)|*(data_buf+25))/1000.;
		ak8975.Mag_Gain.z=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/1000.;
		k_scale_pix=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29));
	
	}		
	else if(*(data_buf+2)==0x10)//Mems
  {
	  mpu6050.Acc.x=(int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
    mpu6050.Acc.y=(int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		mpu6050.Acc.z=(int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		mpu6050.Gyro.x=(int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		mpu6050.Gyro.y=(int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		mpu6050.Gyro.z=(int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		mpu6050.Gyro_deg.x = mpu6050.Gyro.x *TO_ANGLE;
		mpu6050.Gyro_deg.y = mpu6050.Gyro.y *TO_ANGLE;
		mpu6050.Gyro_deg.z = mpu6050.Gyro.z *TO_ANGLE;
		ak8975.Mag_Val.x=(int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		ak8975.Mag_Val.y=(int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		ak8975.Mag_Val.z=(int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		int temp=(int16_t)(*(data_buf+22)<<8)|*(data_buf+23);
		if(temp<3200)
	  ultra_distance = temp;
	  baro_matlab_data[0]=baroAlt=(int32_t)((*(data_buf+24)<<24)|(*(data_buf+25)<<16)|(*(data_buf+26)<<8)|*(data_buf+27));
		
		flow_matlab_data[0]=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29))/1000.;
		flow_matlab_data[1]=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31))/1000.;
		flow_matlab_data[2]=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33))/1000.;
		flow_matlab_data[3]=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+35))/1000.;
		baro_matlab_data[1]=((int16_t)(*(data_buf+36)<<8)|*(data_buf+37));
	}		
	else if(*(data_buf+2)==0x11)//Att
  { dt_flow_rx= Get_Cycle_T(GET_T_FLOW_UART);	
	  Pitch=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
    Roll=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		Yaw=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;
		q_nav[0]=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		q_nav[1]=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		q_nav[2]=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/1000.;
		q_nav[3]=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/1000.;
		Pitch_mid_down=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/10.;
    Roll_mid_down=(float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10.;
		Yaw_mid_down=(float)((int16_t)(*(data_buf+22)<<8)|*(data_buf+23))/10.;
		ref_q_imd_down[0]=(float)((int16_t)(*(data_buf+24)<<8)|*(data_buf+25))/1000.;
		ref_q_imd_down[1]=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/1000.;
		ref_q_imd_down[2]=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29))/1000.;
		ref_q_imd_down[3]=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31))/1000.;
		ALT_VEL_BMP_EKF=(float)(int16_t)((*(data_buf+32)<<8)|*(data_buf+33))/1000.;//m
		
		
  	reference_vr[0]=reference_v.x = 2*(q_nav[1]*q_nav[3] - q_nav[0]*q_nav[2]);
  	reference_vr[1]=reference_v.y = 2*(q_nav[0]*q_nav[1] + q_nav[2]*q_nav[3]);
	  reference_vr[2]=reference_v.z = 1 - 2*(q_nav[1]*q_nav[1] + q_nav[2]*q_nav[2]);
	  reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
	  reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
	  reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
	
	}
	else if(*(data_buf+2)==0x13)//CAL
  {
		
	  DIS_IN[0]=(int16_t)(*(data_buf+5)<<8)|*(data_buf+6);
		DIS_IN[1]=(int16_t)(*(data_buf+7)<<8)|*(data_buf+8);
		DIS_IN[2]=(int16_t)(*(data_buf+9)<<8)|*(data_buf+10);
		DIS_IN[3]=(int16_t)(*(data_buf+11)<<8)|*(data_buf+12);
		DIS_IN[4]=(int16_t)(*(data_buf+13)<<8)|*(data_buf+14);
		DIS_IN[5]=(int16_t)(*(data_buf+15)<<8)|*(data_buf+16);
		DIS_IN[6]=(int16_t)(*(data_buf+17)<<8)|*(data_buf+18);
		DIS_IN[7]=(int16_t)(*(data_buf+19)<<8)|*(data_buf+20);
		DIS_IN[8]=(int16_t)(*(data_buf+21)<<8)|*(data_buf+22);
		DIS_IN[9]=(int16_t)(*(data_buf+23)<<8)|*(data_buf+24);
		DIS_IN[10]=(int16_t)(*(data_buf+25)<<8)|*(data_buf+26);	
		DIS_IN[11]=(int16_t)(*(data_buf+27)<<8)|*(data_buf+28);
		DIS_IN[12]=(int16_t)(*(data_buf+29)<<8)|*(data_buf+30);
		DIS_IN[13]=(int16_t)(*(data_buf+31)<<8)|*(data_buf+32);
		DIS_IN[14]=(int16_t)(*(data_buf+33)<<8)|*(data_buf+34);
		DIS_IN[15]=(int16_t)(*(data_buf+35)<<8)|*(data_buf+36);
		DIS_IN[16]=(int16_t)(*(data_buf+37)<<8)|*(data_buf+38);
		DIS_IN[17]=(int16_t)(*(data_buf+39)<<8)|*(data_buf+40);
		DIS_IN[18]=(int16_t)(*(data_buf+41)<<8)|*(data_buf+42);
		DIS_IN[19]=(int16_t)(*(data_buf+43)<<8)|*(data_buf+44);
    
	}			
}
 


u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//串口接收缓存
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
static u8 _data_len = 0,_data_cnt = 0;
void USART2_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
				if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)//MAX_send num==50
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_Receive_Anl(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;

	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}

   OSIntExit(); 

}

void UsartSend_GOL_LINK(uint8_t ch)
{


	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); 
}

static void Send_Data_GOL_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK(dataToSend[i]);
}


//------------------------------------------------------GOL_LINK----------------------------------------------------

void Send_IMU_TO_FLOW(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp =(vs16)( mpu6050.Acc.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(mpu6050.Acc.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(mpu6050.Acc.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = (vs16)(mpu6050.Gyro_deg.x*100);//q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(mpu6050.Gyro_deg.y*100);//q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(mpu6050.Gyro_deg.z*100);// q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(ak8975.Mag_Val.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(ak8975.Mag_Val.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(ak8975.Mag_Val.z);//mode.save_video;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  Rc_Get_PWM.THROTTLE>1250||(mode.use_dji&&(Rc_Get.THROTTLE>1050));//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
//	_temp =  mode.save_video;
//	data_to_send[_cnt++]=BYTE0(_temp);	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}

void Send_IMU_TO_FLOW2(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x14;//功能字
	data_to_send[_cnt++]=0;//数据量
	

	_temp =(vs16)( mpu6050.Acc_I16.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(mpu6050.Acc_I16.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(mpu6050.Acc_I16.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = (vs16)(mpu6050.Gyro_I16.x);//q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(mpu6050.Gyro_I16.y);//q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(mpu6050.Gyro_I16.z);// q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(ak8975.Mag_Adc.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(ak8975.Mag_Adc.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(ak8975.Mag_Adc.z);//mode.save_video;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  mpu6050.Acc_CALIBRATE;
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp =  mpu6050.Gyro_CALIBRATE;
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp =  ak8975.Mag_CALIBRATED;
	data_to_send[_cnt++]=BYTE0(_temp);	
//	_temp =  mode.save_video;
//	data_to_send[_cnt++]=BYTE0(_temp);	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
		Send_Data_GOL_LINK(data_to_send, _cnt);
}

void Send_IMU(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x80;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(mpu6050.Gyro_deg.x*100);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(mpu6050.Gyro_deg.y*100);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(mpu6050.Gyro_deg.z*100);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = (vs16)( mpu6050.Acc.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( mpu6050.Acc.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( mpu6050.Acc.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
		
	_temp = (vs16)LIMIT(ALT_POS_SONAR*1000,0,2500);//ultra_distance;(baro_to_ground);//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		
	_temp = (vs16)(-ALT_VEL_SONAR*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(-ALT_VEL_BMP*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void Send_FLOW_MODE(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x81;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = mode.en_flow_gro_fix;
	data_to_send[_cnt++]=BYTE0(_temp);
	
		_temp = mode.flow_size;
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}


void GOL_LINK_TASK(void)
{
static u8 cnt[4];
Send_IMU_TO_FLOW();
if(cnt[1]++>1)
{cnt[1]=0;
	
	Send_IMU_TO_FLOW2();
}
}


void Uart5_Init(u32 br_num)//-----video sonar F
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	


	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}

}
u16 PWM_DJI[4]={0};
void Data_Receive_Anl5(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==0x01)//FLOW_MINE_frame
  {
	PWM_DJI[0]= ((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
	PWM_DJI[1]= ((int16_t)(*(data_buf+6)<<8)|*(data_buf+7)); 
	PWM_DJI[2]= ((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
	PWM_DJI[3]= ((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
	circle.x=((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
	circle.y=((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
	circle.control[0]=((int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
	circle.control[1]=((int16_t)(*(data_buf+18)<<8)|*(data_buf+19));
	circle.r=((int16_t)(*(data_buf+20)<<8)|*(data_buf+21));
	circle.check=(*(data_buf+22))&0x01;
	circle.connect=(*(data_buf+22)>>1)&0x01;
  track.check=(*(data_buf+22)>>2)&0x01;
	}
	else if(*(data_buf+2)==0x03)//Num
  {
	circle.connect=1;
	circle.lose_cnt=0;
	track.check=circle.check=(*(data_buf+4));///10.;
	rc_value_temp=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	if(rc_value_temp<320)
	circle.x=rc_value_temp;//Moving_Median(16,3,rc_value_temp);
	circle.y=(int16_t)(*(data_buf+7));//Moving_Median(17,3,((int16_t)(*(data_buf+7))));
	circle.control[0]=(int8_t)(*(data_buf+8));
	circle.control[1]=(int8_t)(*(data_buf+9));
	circle.r=(int8_t)(*(data_buf+10));
	//track.check=(int8_t)(*(data_buf+11));
	}	
	else if(*(data_buf+2)==0x21)//Num
  {
	circle.connect=1;
	circle.lose_cnt=0;
	track.check=circle.check=(*(data_buf+4));///10.;
	circle.x=(int16_t)((*(data_buf+5)<<8)|*(data_buf+6));
	circle.y=(int16_t)((*(data_buf+7)<<8)|*(data_buf+8));
	circle.z=(int16_t)((*(data_buf+9)<<8)|*(data_buf+10));
	circle.pit=(int16_t)((*(data_buf+11)<<8)|*(data_buf+12));
	circle.rol=(int16_t)((*(data_buf+13)<<8)|*(data_buf+14));
	circle.yaw=(int16_t)((*(data_buf+15)<<8)|*(data_buf+16));	
		
	}	
}
u8 TxBuffer5[256];
u8 TxCounter5=0;
u8 count5=0; 

u8 Rx_Buf5[256];	//串口接收缓存
u8 RxBuffer5[50];
u8 RxState5 = 0;
u8 RxBufferNum5 = 0;
u8 RxBufferCnt5 = 0;
u8 RxLen5 = 0;
static u8 _data_len5 = 0,_data_cnt5 = 0;
void UART5_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(UART5->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART5->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志

		com_data = UART5->DR;
				if(RxState5==0&&com_data==0xAA)
		{
			RxState5=1;
			RxBuffer5[0]=com_data;
		}
		else if(RxState5==1&&com_data==0xAF)
		{
			RxState5=2;
			RxBuffer5[1]=com_data;
		}
		else if(RxState5==2&&com_data>0&&com_data<0XF1)
		{
			RxState5=3;
			RxBuffer5[2]=com_data;
		}
		else if(RxState5==3&&com_data<50)
		{
			RxState5 = 4;
			RxBuffer5[3]=com_data;
			_data_len5 = com_data;
			_data_cnt5 = 0;
		}
		else if(RxState5==4&&_data_len5>0)
		{
			_data_len5--;
			RxBuffer5[4+_data_cnt5++]=com_data;
			if(_data_len5==0)
				RxState5 = 5;
		}
		else if(RxState5==5)
		{
			RxState5 = 0;
			RxBuffer5[4+_data_cnt5]=com_data;
			Data_Receive_Anl5(RxBuffer5,_data_cnt5+5);
		}
		else
			RxState5 = 0;
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{
				
		UART5->DR = TxBuffer5[TxCounter++]; //写DR清除中断标志          
		if(TxCounter5 == count5)
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
		USART_ClearITPendingBit(UART5,USART_IT_TXE);
	}

   OSIntExit(); 

}
void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{

while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
USART_SendData(UART5, data_num); ;//USART1, ch); 

}
void UsartSend_DJI_LINK(uint8_t ch)
{


	while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
USART_SendData(UART5, ch); 
}

static void Send_Data_DJI_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_DJI_LINK(dataToSend[i]);
}


u16 DJI_RC[4]={1500,1500,1000,1500};
u16 DJI_RC_TEMP[4]={1500,1500,1000,1500};
void Send_DJI(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = DJI_RC[0];//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = DJI_RC[1];//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = DJI_RC[2];//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = DJI_RC[3];//(CH_filter[YAWr]+1500);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mode.use_dji;//(CH_filter[YAWr]+1500);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mode.dji_mode;//1 z 0 GPS
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = KEY_SEL[3]<<3|KEY_SEL[2]<<2|KEY_SEL[1]<<1|KEY_SEL[0];//1 z 0 GPS
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = KEY[7]<<7|KEY[6]<<6|KEY[5]<<5|KEY[4]<<4|KEY[3]<<3|KEY[2]<<2|KEY[1]<<1|KEY[0];//1 z 0 GPS
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = motor_out[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  _temp = motor_out[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = motor_out[2];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = motor_out[3];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_DJI_LINK(data_to_send, _cnt);
}

void Send_DJI2(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = SPID.OP;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  _temp = SPID.OI;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.OD;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = SPID.IP;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.II;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.ID;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = SPID.YP;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.YI;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.YD;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp = motor_out[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  _temp = motor_out[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = motor_out[2];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = motor_out[3];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_DJI_LINK(data_to_send, _cnt);
}
u16 OFF_YAW_DJ=50;
u8 state_v_test=13;
void Send_DJI3(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x03;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = Yaw_DJ+OFF_YAW_DJ;//PWM_dj[1];//Pitch*10;//(CH_filter[YAWr]+1500);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pitch_DJ;//PWM_dj[0];//Roll *10;//(CH_filter[YAWr]+1500);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Roll  *10;//(CH_filter[YAWr]+1500);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = ALT_POS_SONAR2*1000;//(CH_filter[YAWr]+1500);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_BMP*1000;//(CH_filter[YAWr]+1500);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = ALT_VEL_BMP*1000;//(CH_filter[YAWr]+1500);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp = motor_out[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  _temp = motor_out[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = motor_out[2];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = motor_out[3];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Rc_Get.AUX1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Rc_Get.AUX2;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp = state_v;
	//	_temp = state_v_test;
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_DJI_LINK(data_to_send, _cnt);
}

void UsartSend_UP_LINK(uint8_t ch)
{

	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch); 
}

static void Send_Data_UP_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_UP_LINK(dataToSend[i]);
}
u8 BLE_UP1[20]={"A1231123"};
u8 BLE_TEST[20]={"AT+"};
u8 BLE_BAUD[20]={"AT+BAUD[G]"};//C 9600 F 57600 G 115200
u8 BLE_RENEW[20]={"AT+RENEW"};
u8 BLE_RATE[20] ={"AT+RATE[100]"};
u8 SET_PIN[20] ={"AT+PIN6666"};
void Usart1_Init(u32 br_num)//-------UPload_board1
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
//	//串口中断优先级
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

	//配置PA3  WK  BLE控制端
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART1, ENABLE); 
	//Send_Data_UP_LINK(BLE_UP1,20);
 #define EN_BLE_SET 0
 #if EN_BLE_SET
	 GPIO_ResetBits(GPIOA,GPIO_Pin_3);//透传模式
	 delay_ms(500);
   Send_Data_UP_LINK(BLE_RATE,20);
	 delay_ms(100);
	 Send_Data_UP_LINK(BLE_RENEW,20);
	 while(1);
#endif
 GPIO_SetBits(GPIOA,GPIO_Pin_3);//透传模式
 Send_Data_UP_LINK(SET_PIN,20);
  GPIO_ResetBits(GPIOA,GPIO_Pin_3);//透传模式
//	   //USART1 初始化设置
//	USART_InitStructure.USART_BaudRate =  9600;//波特率设置
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	
	
}
u8 BLE_RX_BUF[50];
void USART1_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	static u16 _cnt;
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART1->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志

		com_data = USART1->DR;
	  BLE_RX_BUF[_cnt++]=com_data;
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART_ClearITPendingBit(USART1,USART_IT_TXE);
	}

   OSIntExit(); 

}

void Data_Receive_Anl4(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	if(Rc_Get_PWM.THROTTLE!=1139)
	Feed_Rc_Dog(2);//通信看门狗喂狗
  if(*(data_buf+2)==0x66)//RC_GET1
  {
		for(i=0;i<32;i++)
		NRF24L01_RXDATA[i]=*(data_buf+i+4);
		
	  NRF_DataAnl();
  
	}
	else if(*(data_buf+2)==0x88)//RC_PWM
  {
		Rc_Get_PWM.PITCH=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
		Rc_Get_PWM.ROLL=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
		Rc_Get_PWM.THROTTLE=((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
		Rc_Get_PWM.YAW=((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
		
		Rc_Get_PWM.AUX1=((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
		Rc_Get_PWM.RST=((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
		Rc_Get_PWM.HEIGHT_MODE=((int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
		Rc_Get_PWM.POS_MODE=((int16_t)(*(data_buf+18)<<8)|*(data_buf+19));
		
	  RX_CH_PWM[THRr]=	LIMIT(Rc_Get_PWM.THROTTLE-RX_CH_FIX_PWM[THRr]-20,1000,2000)	;
	  RX_CH_PWM[PITr]=  my_deathzoom_rc(Rc_Get_PWM.ROLL-RX_CH_FIX_PWM[ROLr]-20,5)	;
	  RX_CH_PWM[ROLr]=  my_deathzoom_rc(Rc_Get_PWM.PITCH-RX_CH_FIX_PWM[PITr]-20,5)	;
				
		if(!mode.yaw_imu_control)	
		RX_CH_PWM[YAWr]=  my_deathzoom_rc(Rc_Get_PWM.YAW-RX_CH_FIX_PWM[YAWr]-20,5)	;
		else{	
		if(fabs( ypr_sb[2])>32&&fabs(ypr_sb[1])<15)	
		RX_CH_PWM[YAWr]=  limit_mine(ypr_sb[2]*10,500)	+1500;
		else
		RX_CH_PWM[YAWr]=1500;	}
		RX_CH_PWM[AUX3r]=Rc_Get_PWM.POS_MODE;
		RX_CH_PWM[AUX4r]=Rc_Get_PWM.HEIGHT_MODE;
	}
}


u8 TxBuffer4[256];
u8 TxCounter4=0;
u8 count4=0; 

u8 Rx_Buf4[256];	//串口接收缓存
u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4 = 0;
static u8 _data_len4 = 0,_data_cnt4 = 0;
void UART4_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(UART4->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART4->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//清除中断标志

		com_data = UART4->DR;
				if(RxState4==0&&com_data==0xAA)
		{
			RxState4=1;
			RxBuffer4[0]=com_data;
		}
		else if(RxState4==1&&com_data==0xAF)
		{
			RxState4=2;
			RxBuffer4[1]=com_data;
		}
		else if(RxState4==2&&com_data>0&&com_data<0XF1)
		{
			RxState4=3;
			RxBuffer4[2]=com_data;
		}
		else if(RxState4==3&&com_data<50)
		{
			RxState4 = 4;
			RxBuffer4[3]=com_data;
			_data_len4 = com_data;
			_data_cnt4 = 0;
		}
		else if(RxState4==4&&_data_len4>0)
		{
			_data_len4--;
			RxBuffer4[4+_data_cnt4++]=com_data;
			if(_data_len4==0)
				RxState4 = 5;
		}
		else if(RxState4==5)
		{
			RxState4 = 0;
			RxBuffer4[4+_data_cnt4]=com_data;
			Data_Receive_Anl4(RxBuffer4,_data_cnt4+5);
		}
		else
			RxState4 = 0;
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(UART4,USART_IT_TXE ) )
	{
				
		UART4->DR = TxBuffer4[TxCounter++]; //写DR清除中断标志          
		if(TxCounter5 == count5)
		{
			UART4->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
		USART_ClearITPendingBit(UART4,USART_IT_TXE);
	}

   OSIntExit(); 

}

void Usart4_Init(u32 br_num)//-------SD_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(UART4, ENABLE);  //使能串口1 
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	

	//使能USART2接收中断
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(UART4, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}

void UsartSend_SD(uint8_t ch)
{

while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch); ;//USART1, ch); 
}

static void Send_Data_SD(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_SD(dataToSend[i]);
}

void UART2_Put_Char(unsigned char DataToSend)
{

while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, DataToSend); 

}

void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+8);
	UART2_Put_Char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=az;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}


void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2+2;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+4);
	UART2_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;							
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
								 
	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;				
	ctemp=alt;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART2_Put_Char(ctemp);	   
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}



//------------------------------------------------------GOL_LINK_SD----------------------------------------------------
void Send_IMU_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x05;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = Roll*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pitch*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Yaw*10	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[1]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[2]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[3]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[3]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp=ak8975_fc.Mag_Adc.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ak8975_fc.Mag_Adc.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ak8975_fc.Mag_Adc.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ak8975_fc.Mag_Val.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp=ak8975_fc.Mag_Val.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp=ak8975_fc.Mag_Val.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

data_to_send[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_ATT_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = except_A.y*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = except_A.x*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = except_A.z*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	
	
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_FLOW_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = target_position[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = target_position[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = now_position[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = now_position[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = tar_speed[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = tar_speed[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = actual_speed[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = actual_speed[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = nav[ROLr]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = nav[PITr]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_ALT_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x03;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = exp_height;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = ultra_dis_lpf;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = ultra_ctrl_out;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = wz_speed;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = thr_test	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;//baroAlt	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_MODE_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x04;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = mode.en_sd_save;
	data_to_send[_cnt++]=BYTE0(_temp);

	
	
  data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_FLOW_USE_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x06;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = imu_nav.flow.speed.east ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.speed.west;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  imu_nav.flow.speed.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.speed.y ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = imu_nav.flow.position.east	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.position.west	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_SLAM_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x07;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = slam.dis[2];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[3];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[4];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	

	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_GPS_SD(void)
{u8 i;	u8 sum = 0;u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x08;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp32 =  imu_nav.gps.J;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.W;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp =imu_nav.gps.gps_mode;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =imu_nav.gps.star_num;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 =  imu_nav.gps.X_O;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_O;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.X_UKF;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_UKF;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	
	

	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

//void SD_LINK_TASK(void)
//{
//static u8 cnt[4];
//static u8 flag;
//if(cnt[0]++>1)
//{cnt[0]=0;
//Send_ATT_PID_SD();
//}
//if(cnt[1]++>0)
//{cnt[1]=0;
////	if(flag)
////	{flag=0;
//	Send_IMU_SD();
////}
////	else{flag=1;
////	Send_MODE_SD();
////	}
//}
//if(cnt[2]++>2)
//{cnt[2]=0;
//	//Send_FLOW_PID_SD();
//	Send_FLOW_USE_SD();
//}
//if(cnt[3]++>2)
//{cnt[3]=0;
//	//Send_ALT_PID_SD();
//	Send_GPS_SD();//Send_SLAM_SD();
//}

//}

/*
#define SEND_IMU 0
#define SEND_FLOW 1
#define SEND_GPS 2
#define SEND_ALT 3
*/
void SD_LINK_TASK2(u8 sel)
{
static u8 cnt[4];
static u8 flag;
	
	switch(sel)
	{
		case SEND_IMU:
				Send_IMU_SD();
		break;
		case SEND_FLOW:
				Send_FLOW_USE_SD();
		break;
		case SEND_GPS:
				Send_GPS_SD();
		break;
		case SEND_ALT:
				Send_ALT_PID_SD();
		break;
	}

 Send_MODE_SD();
}

void Usart3_Init(u32 br_num)//-------PXY
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

	
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART3, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}

float rate_gps_board;
 void Data_Receive_Anl3(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	if(*(data_buf+2)==0x01)//SONAR
  {
	//rate_gps_board=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));///10.;
	sonar_avoid[0]=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
	sonar_avoid[1]=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
	sonar_avoid[2]=((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
	sonar_avoid[3]=((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
	sonar_avoid[4]=((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
	sonar_avoid[5]=((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
	sonar_avoid[6]=((int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
	sonar_avoid[7]=((int16_t)(*(data_buf+18)<<8)|*(data_buf+19));
	sonar_avoid_c[0]=((int16_t)(*(data_buf+20)<<8)|*(data_buf+21));
	sonar_avoid_c[1]=((int16_t)(*(data_buf+22)<<8)|*(data_buf+23));	
	sys.avoid=1;
	}	
	else if(*(data_buf+2)==0x02)//SLAM_frame
  {

	circle.connect=1;
	circle.lose_cnt=0;
	circle.check=(*(data_buf+4));///10.;
  rc_value_temp=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	if(rc_value_temp<320)
	circle.x=rc_value_temp;//Moving_Median(16,3,rc_value_temp);
	circle.y=(int16_t)(*(data_buf+7));//Moving_Median(17,3,((int16_t)(*(data_buf+7))));
	circle.control[0]=(int8_t)(*(data_buf+8));
	circle.control[1]=(int8_t)(*(data_buf+9));
	circle.r=(int8_t)(*(data_buf+10));
	track.check=(int8_t)(*(data_buf+11));
	}			
	else if(*(data_buf+2)==0x03)//SLAM_frame
  {

	marker.connect=1;
	marker.lose_cnt=0;
	marker.check=(*(data_buf+4));///10.;
	marker.pos_now[0]=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	marker.pos_now[1]=((int16_t)(*(data_buf+7)<<8)|*(data_buf+8));
	marker.pos_set[0]=((int16_t)(*(data_buf+9)<<8)|*(data_buf+10));
	marker.pos_set[1]=((int16_t)(*(data_buf+11)<<8)|*(data_buf+12));
	marker.angle[0]=((int16_t)(*(data_buf+13)<<8)|*(data_buf+14));		
	marker.angle[1]=((int16_t)(*(data_buf+15)<<8)|*(data_buf+16));
	marker.angle[2]=((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
	}			
}

//----------------------pxy
Pixy_Color Pixy;//结构体定义
unsigned char Raw_Data[20];//原始数据大包，为解串准备
u16 counter;//计数用

char USART1_Flag=0 ;
int USART1_FAIL = 0;	//通信失败标志、帧头错误置高
int USART1_Half_OK=0;
unsigned char Pixy_Data[20];
u32 MianJi=0;
double Zero_Width=0;//零度时的宽
double Zero_Height=0;//零度时的高
double Zero_MianJi=0;//零度时的面积=零度时的高*零度时的宽
double Distance=0;
double angle=0;
int Ten_to_Octal(u32 ten_data,u32 octal_data)
{
    int i,j; 
    int tmp;	
	  octal_data=0;
    for(i=1;;i++) 
    {
//        tmp=ten_data%8;           /*依次获得转换为八进制时的各位数（个、十、百位。。。。）*/
			  tmp=ten_data&7;  //相当于tmp=ten_data%8;    位操作速度大于求余                
        for(j=1;j<=i;j++) tmp*=10;
        tmp/=10;
        octal_data+=tmp;               /*存储转换好的十进制数*/
        ten_data/=8;
        if(ten_data==0)   break;           /*若最后一位不满8，则结束循环*/
    }
		return octal_data;
}
void Rand_to_Zero(s16 data_width,s16 data_height,s16 data_angle)
{ 
	 if(data_angle)
	 {
		 if(data_angle<0)  data_angle=-data_angle;
     if(data_angle==45 || data_angle==46) data_angle=44;//45°角度无法计算出来（分母为0），则用44°代替。    		 
//		 if(data_angle-90>0) data_angle=data_angle-90;//90-180°之间均按减去90°来计算。 
		 if(data_angle>90) 
		 {
			  data_angle=180-data_angle;//90-180°之间均按180°减去来计算
			  if(data_angle==45 || data_angle==46) data_angle=44;
		 }
		 //45°±5°范围内的摄像头返回的宽和高度稍微变一点数据对公式算出来的值能产生较大影响，所以考虑更改摄像头返回的值，让长和宽值接近
		 if(ABS(data_angle-45)<=8)
		 {
        if(data_width > data_height) 
				{				
					 if((data_width-data_height)%2==0)  data_width-=(data_width-data_height)/2;//差值为偶数  a%(2^n)=a&(2^n-1)
					 else  data_width-=(data_width-data_height+1)>>1;//奇数
        }
        else 
        {				
					 if((data_height - data_width)%2==0)  data_height=data_height-(data_height - data_width)/2;//差值为偶数 
					 else  data_height=data_height-(data_height - data_width+1)/2;//奇数
        }					
     }	 
		 angle = (((double)data_angle)/180) *3.141592654;		 
     //零度时的数据一定不能用整形，要不然算的数据有大偏差		 
		 Zero_Height = (data_height * cos(angle) -  data_width *sin(angle))/(cos(angle*2)) ;
		 Zero_Width  = (data_height - Zero_Height * cos(angle))/sin(angle) ; 
		 Zero_MianJi = Zero_Height * Zero_Width;	 
	 }
	 else  
	 {
		 Zero_Height=data_height;
		 Zero_Width=data_width;
		 Zero_MianJi=MianJi;
	 }
}
void Blocks_Distance(double Data_MianJi)
{
//	  Distance=1.2*100000/Data_MianJi - 2.6*10000000/Data_MianJi/Data_MianJi + 30.0 ;
  	Distance=2.87*10000000000/(Data_MianJi*Data_MianJi*Data_MianJi) - 1.08*100000000/(Data_MianJi*Data_MianJi) + 1.78*100000/Data_MianJi + 22.8 ;
Distance=LIMIT(Distance,0,500);
}

void Blocks_Receive_Data(void)
{
//    int i; 	
		
//		++j;
	   /*  单一色 1个：帧头为55 aa 55 aa
				 单一色 2个：面积大的先发，帧头为55 aa 55 aa  ，下一个帧头为55 aa
				 CC模式 1个：帧头为55 aa 56 aa
				 CC模式+单一模式 各1个：先发单一模式，帧头为55 aa 55 aa，后发CC模式，帧头为56 aa
				 CC模式 2个：帧头为55 aa 56 aa (此情况不准，两个标记会产生结合，角度几乎不对)			  
		 */
	  /*******************若为CC模式，1个，包头为55 aa 56 aa ,18个数据**************************/
//	  switch(USART1_Half_OK)
//	  {
//			case 0x03:
//			  {
	if(USART1_Half_OK==0x03){
				  Pixy.Pixy_Sig    = Ten_to_Octal(Pixy_Data[6] + Pixy_Data[7]*256,Pixy.Pixy_Sig);
					Pixy.Pixy_PosX   = Pixy_Data[8]  + Pixy_Data[9]*256;
					Pixy.Pixy_PosY   = Pixy_Data[10] + Pixy_Data[11]*256;
					Pixy.Pixy_Width  = Pixy_Data[12] + Pixy_Data[13]*256;
					Pixy.Pixy_Height = Pixy_Data[14] + Pixy_Data[15]*256;
          //求角度时，若角度为正，返回数据为0x？？ 0x00
				  if(Pixy_Data[17]==0x00)      Pixy.Pixy_Angle = Pixy_Data[16] + Pixy_Data[17]*256;
				  //求角度时，若角度为负，返回数据为0x？？ 0xFF
				  //即求出0x？？对应的十进制 - 0xFF对应的十进制数(255) = 负的角度
				  else if(Pixy_Data[17]==0xFF) Pixy.Pixy_Angle = (int)Pixy_Data[16] - 255;}
	else if(USART1_Half_OK==0x01||USART1_Half_OK==0x02)
	{
	  Pixy.Pixy_Sig    = Pixy_Data[6]  + Pixy_Data[7]*256;
					Pixy.Pixy_PosX   = Pixy_Data[8]  + Pixy_Data[9]*256;
					Pixy.Pixy_PosY   = Pixy_Data[10] + Pixy_Data[11]*256;
					Pixy.Pixy_Width  = Pixy_Data[12] + Pixy_Data[13]*256;
					Pixy.Pixy_Height = Pixy_Data[14] + Pixy_Data[15]*256;	
	
	}
//			  }break;
//			
//			case 0x01:
//				{
//				  Pixy.Pixy_Sig    = Pixy_Data[6]  + Pixy_Data[7]*256;
//					Pixy.Pixy_PosX   = Pixy_Data[8]  + Pixy_Data[9]*256;
//					Pixy.Pixy_PosY   = Pixy_Data[10] + Pixy_Data[11]*256;
//					Pixy.Pixy_Width  = Pixy_Data[12] + Pixy_Data[13]*256;
//					Pixy.Pixy_Height = Pixy_Data[14] + Pixy_Data[15]*256;	
//		    }break;
//				
//			case 0x02:
//			  {
//				  Pixy.Pixy_Sig    = Pixy_Data[4]  + Pixy_Data[5]*256;
//					Pixy.Pixy_PosX   = Pixy_Data[6]  + Pixy_Data[7]*256;
//					Pixy.Pixy_PosY   = Pixy_Data[8]  + Pixy_Data[9]*256;
//					Pixy.Pixy_Width  = Pixy_Data[10] + Pixy_Data[11]*256;
//					Pixy.Pixy_Height = Pixy_Data[12] + Pixy_Data[13]*256;	
//			  }break;
//			
//			case 0x04:
//			  {
//				  Pixy.Pixy_Sig    = Ten_to_Octal(Pixy_Data[4] + Pixy_Data[5]*256,Pixy.Pixy_Sig);
//					Pixy.Pixy_PosX   = Pixy_Data[6]  + Pixy_Data[7]*256;
//					Pixy.Pixy_PosY   = Pixy_Data[8]  + Pixy_Data[9]*256;
//					Pixy.Pixy_Width  = Pixy_Data[10] + Pixy_Data[11]*256;
//					Pixy.Pixy_Height = Pixy_Data[12] + Pixy_Data[13]*256;
//          //求角度时，若角度为正，返回数据为0x？？ 0x00
//				  if(Pixy_Data[15]==0x00)      Pixy.Pixy_Angle = Pixy_Data[14] + Pixy_Data[15]*256;
//				  //求角度时，若角度为负，返回数据为0x？？ 0xFF
//				  //即求出0x？？对应的十进制 - 0xFF对应的十进制数(255) = 负的角度
//				  else if(Pixy_Data[15]==0xFF) Pixy.Pixy_Angle = (int)Pixy_Data[14] - 255;		
//			   }break;
//			default:break;
//		}	
		MianJi=Pixy.Pixy_Width * Pixy.Pixy_Height;
   // if(USART1_Half_OK==3 || USART1_Half_OK==4) 
  //  {			
			Rand_to_Zero(Pixy.Pixy_Width,Pixy.Pixy_Height,Pixy.Pixy_Angle);
			Blocks_Distance(Zero_MianJi);
			//Control_Car();
			//LCD_Pixy();
		//}
		//USART1_Half_OK=0; 
}

void data_get_pxy(char data)
{
u8 i;
Raw_Data[counter] = data;
			if(counter==0 && Raw_Data[0]!=0x55 && Raw_Data[0]!=0x56)	 USART1_FAIL = 1;//通信失败
			if(USART1_FAIL==0)//通信成功
			{
					counter++;
				  if(counter>=14)
					{
						if(Raw_Data[0]==0x55 && Raw_Data[1]==0xAA && Raw_Data[2]==0x55 && Raw_Data[3]==0xAA)       USART1_Half_OK=0x01;//正常码模式，帧头为55 aa 55 aa
						else if(Raw_Data[0]==0x55 && Raw_Data[1]==0xAA && Raw_Data[2]!=0x55 && Raw_Data[2]!=0x56)  USART1_Half_OK=0x02;//正常码模式，帧头为55 aa
						else if(Raw_Data[0]==0x55 && Raw_Data[1]==0xAA && Raw_Data[2]==0x56 && Raw_Data[3]==0xAA)  USART1_Half_OK=0x03;//cc模式，帧头为55 aa 56 aa
						else if(Raw_Data[0]==0x56 && Raw_Data[1]==0xAA)  USART1_Half_OK=0x04;//正常码模式+CC模式，帧头56 aa
						if(USART1_Half_OK!=0)
						{
							 if((USART1_Half_OK==0x01 && counter==16)||(USART1_Half_OK==0x02 && counter==14)||(USART1_Half_OK==0x03 && counter==18)||(USART1_Half_OK==0x04 && counter==16))  
							 {
									for(i=0;i<counter;i++)  Pixy_Data[i]=Raw_Data[i];
									USART1_Flag=1;
////							USART1_Half_OK=0;
									counter=0;
							 }						 
						}
				 }
			}
			else
			{
				USART1_FAIL=0;
				counter=0;
			}
			
//					   if(USART1_Flag)
//			 {	 
////				  pixy_per_time=Time3_counter;
////				  LCD_Write_string("Time:",0,7);
////				  LCD_Display_int(pixy_per_time,7,7);
////				  Time3_counter=0;
//  				Blocks_Receive_Data();
//			    USART1_Flag=0;
//				 // Camera_counter++;	
//          				 
//			 }
}


u8 Rx_Buf3[256];	//串口接收缓存
u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)
{  OSIntEnter();  
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志

		com_data = USART3->DR;
		data_get_pxy(com_data);
		
				if(RxState3==0&&com_data==0xAA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xAF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<50)
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
			Data_Receive_Anl3(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
	
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}

		USART_ClearITPendingBit(USART3,USART_IT_TXE);
	}
 OSIntExit();        
}

void UsartSend_GPS(uint8_t ch)
{

while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); ;//USART1, ch); 
}

static void Send_Data_GPS(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GPS(dataToSend[i]);
}

#define MAX_Yun_Angle 60
float Angle_Yun[2]={0,0};

void Send_IMU_TO_GPS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp =  mode.en_sonar_avoid;//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	Angle_Yun[0]= -(float)(RX_CH[6]-1000)/1000*MAX_Yun_Angle;//AUX3
	
	_temp = (vs16)(Angle_Yun[0]*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Angle_Yun[1]*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GPS(data_to_send, _cnt);
}

void CPU_LINK_TASK(void)
{
static u8 cnt[4];
static u8 flag;
if(cnt[0]++>0)
{cnt[0]=0;
 Send_IMU_TO_GPS();
}
}



//-------------------------NAV_BOARD_LINK

//------------------------------------------------------GOL_LINK----------------------------------------------------

void UsartSend_GOL_LINK_NAV(uint8_t ch)
{


	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}

static void Send_Data_GOL_LINK_NAV(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK_NAV(dataToSend[i]);
}

void Send_IMU_NAV(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK_NAV(data_to_send, _cnt);
}

int16_t BLE_DEBUG[16];
u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区
void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;

char ctemp;	
BLE_DEBUG[0]=ax;
BLE_DEBUG[1]=ay;
BLE_DEBUG[2]=az;	
BLE_DEBUG[3]=gx;
BLE_DEBUG[4]=gy;
BLE_DEBUG[5]=gz;
BLE_DEBUG[6]=hx;
BLE_DEBUG[7]=hy;
BLE_DEBUG[8]=hz;
BLE_DEBUG[9]=yaw;
BLE_DEBUG[10]=pitch;
BLE_DEBUG[11]=roll;
BLE_DEBUG[12]=alt;
BLE_DEBUG[13]=tempr;
BLE_DEBUG[14]=press;
BLE_DEBUG[15]=IMUpersec;


	
SendBuff1[i++]=0xa5;
SendBuff1[i++]=0x5a;
SendBuff1[i++]=14+8;
SendBuff1[i++]=0xA2;

if(ax<0)ax=32768-ax;
ctemp=ax>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=ax;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(ay<0)ay=32768-ay;
ctemp=ay>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=ay;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(az<0)az=32768-az;
ctemp=az>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=az;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(gx<0)gx=32768-gx;
ctemp=gx>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gx;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(gy<0)gy=32768-gy;
ctemp=gy>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gy;
SendBuff1[i++]=ctemp;
temp+=ctemp;
//-------------------------
if(gz<0)gz=32768-gz;
ctemp=gz>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gz;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hx<0)hx=32768-hx;
ctemp=hx>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hx;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hy<0)hy=32768-hy;
ctemp=hy>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hy;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hz<0)hz=32768-hz;
ctemp=hz>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hz;
SendBuff1[i++]=ctemp;
temp+=ctemp;

SendBuff1[i++]=temp%256;
SendBuff1[i++]=(0xaa);
//
 temp=0xaF+2+2;
SendBuff1[i++]=(0xa5);
SendBuff1[i++]=(0x5a);
SendBuff1[i++]=(14+4);
SendBuff1[i++]=(0xA1);


if(yaw<0)yaw=32768-yaw;
ctemp=yaw>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=yaw;							
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(pitch<0)pitch=32768-pitch;
ctemp=pitch>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=pitch;
SendBuff1[i++]=ctemp;
temp+=ctemp;
							 
if(roll<0)roll=32768-roll;
ctemp=roll>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=roll;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(alt<0)alt=32768-alt;
ctemp=alt>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;				
ctemp=alt;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(tempr<0)tempr=32768-tempr;
ctemp=tempr>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=tempr;
SendBuff1[i++]=ctemp;	   
temp+=ctemp;

if(press<0)press=32768-press;
ctemp=press>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=press;
SendBuff1[i++]=ctemp;
temp+=ctemp;

ctemp=IMUpersec>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=IMUpersec;
SendBuff1[i++]=ctemp;
temp+=ctemp;

SendBuff1[i++]=(temp%256);
SendBuff1[i++]=(0xaa);
}


u8 SendBuff2[SEND_BUF_SIZE2];	//发送数据缓冲区
void data_per_uart2(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff2[_cnt++]=0xAA;
	SendBuff2[_cnt++]=0xAF;
	SendBuff2[_cnt++]=0x01;//功能字
	SendBuff2[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);

	_temp = q_nav[0]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	
	_temp =  0;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  0;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  ALT_POS_SONAR2*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	
	SendBuff2[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[_cnt++] = sum;
}


u8 SendBuff3[SEND_BUF_SIZE3];	//发送数据缓冲区
void data_per_uart3(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff3[_cnt++]=0xAA;
	SendBuff3[_cnt++]=0xAF;
	SendBuff3[_cnt++]=0x01;//功能字
	SendBuff3[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);

	_temp = q_nav[0]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	SendBuff3[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff3[i];
	SendBuff3[_cnt++] = sum;
}


u8 SendBuff4[SEND_BUF_SIZE4];	//发送数据缓冲区
void data_per_uart4(u8 sel)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;


switch(sel){
	case SEND_ALT:
	SendBuff4[_cnt++]=0xAA;
	SendBuff4[_cnt++]=0xAF;
	SendBuff4[_cnt++]=0x03;//功能字
	SendBuff4[_cnt++]=0;//数据量
	_temp = exp_height;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ultra_dis_lpf;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ultra_ctrl_out;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = wz_speed;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
 	_temp = thr_test	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_BMP*1000	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_SONAR2*1000	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
  _temp = mode.en_sd_save;
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_BMP_EKF*1000	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ALT_VEL_BMP_EKF*1000	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ultra_distance	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	
	SendBuff4[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff4[i];
	SendBuff4[_cnt++] = sum;
	break;
	case SEND_IMU:
	SendBuff4[_cnt++]=0xAA;
	SendBuff4[_cnt++]=0xAF;
	SendBuff4[_cnt++]=0x05;//功能字
	SendBuff4[_cnt++]=0;//数据量
	_temp = Rol_fc*10;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = Pit_fc*10;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = Yaw_fc*10	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=(mpu6050.Gyro_I16.x-mpu6050.Gyro_Offset.x);
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=(mpu6050.Gyro_I16.y-mpu6050.Gyro_Offset.y);
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=(mpu6050.Gyro_I16.z-mpu6050.Gyro_Offset.z);
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050_fc.Acc_I16.x;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050_fc.Acc_I16.y;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050_fc.Acc_I16.z;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[0]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[1]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[2]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[3]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);

	_temp=ak8975_fc.Mag_Adc.x;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ak8975_fc.Mag_Adc.y;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ak8975_fc.Mag_Adc.z;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ak8975_fc.Mag_Val.x;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
		_temp=ak8975_fc.Mag_Val.y;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
		_temp=ak8975_fc.Mag_Val.z;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
		_temp=mode.en_sd_save;
	SendBuff4[_cnt++]=BYTE0(_temp);
	SendBuff4[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[_cnt++] = sum;
	break;
	case SEND_FLOW:
	SendBuff4[_cnt++]=0xAA;
	SendBuff4[_cnt++]=0xAF;
	SendBuff4[_cnt++]=0x06;//功能字
	SendBuff4[_cnt++]=0;//数据量
	_temp=imu_nav.flow.speed.x_f*1000;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=imu_nav.flow.speed.y_f*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=imu_nav.flow.speed.x*1000;//origin
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=imu_nav.flow.speed.y*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	_temp=target_position[LAT]*100;//
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=target_position[LON]*100;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=now_position[LAT]*100;//
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=now_position[LON]*100;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	_temp=flow_matlab_data[0]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=flow_matlab_data[1]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=flow_matlab_data[2]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=flow_matlab_data[3]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);

	_temp=baroAlt_fc;//baro_matlab_data[0];
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=acc_bmp*1000;//baro_matlab_data[1];
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=circle.check&&circle.connect;///10.;
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=circle.x;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=circle.y;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=circle.z;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=circle.pit;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=circle.rol;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=circle.yaw;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=k_scale_pix;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	SendBuff4[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[_cnt++] = sum;
	break;
  case SEND_PID:
	SendBuff4[_cnt++]=0xAA;
	SendBuff4[_cnt++]=0xAF;
	SendBuff4[_cnt++]=0x07;//功能字
	SendBuff4[_cnt++]=0;//数据量
	_temp=SPID.OP;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=SPID.OI;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=SPID.OD;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	_temp=SPID.IP;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=SPID.II;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=SPID.ID;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	_temp=SPID.YP;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=SPID.YI;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=SPID.YD;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	_temp=HPID.OP;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=HPID.OI;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=HPID.OD;//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	SendBuff4[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[_cnt++] = sum;
	break;
	case SEND_DEBUG:
	SendBuff4[_cnt++]=0xAA;
	SendBuff4[_cnt++]=0xAF;
	SendBuff4[_cnt++]=0x08;//功能字
	SendBuff4[_cnt++]=0;//数据量
	for(i=0;i<16;i++){
	_temp=BLE_DEBUG[i];//filter
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	}
	
	SendBuff4[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[_cnt++] = sum;
	break;
	
	
	
	default:break;
}
}


void data_per_uart4_ble(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;

char ctemp;	
	
SendBuff4[i++]=0xa5;
SendBuff4[i++]=0x5a;
SendBuff4[i++]=14+8;
SendBuff4[i++]=0xA2;

if(ax<0)ax=32768-ax;
ctemp=ax>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=ax;
SendBuff4[i++]=ctemp;
temp+=ctemp;

if(ay<0)ay=32768-ay;
ctemp=ay>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=ay;
SendBuff4[i++]=ctemp;
temp+=ctemp;

if(az<0)az=32768-az;
ctemp=az>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=az;
SendBuff4[i++]=ctemp;
temp+=ctemp;

if(gx<0)gx=32768-gx;
ctemp=gx>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=gx;
SendBuff4[i++]=ctemp;
temp+=ctemp;

if(gy<0)gy=32768-gy;
ctemp=gy>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=gy;
SendBuff4[i++]=ctemp;
temp+=ctemp;
//-------------------------
if(gz<0)gz=32768-gz;
ctemp=gz>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=gz;
SendBuff4[i++]=ctemp;
temp+=ctemp;

if(hx<0)hx=32768-hx;
ctemp=hx>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=hx;
SendBuff4[i++]=ctemp;
temp+=ctemp;

if(hy<0)hy=32768-hy;
ctemp=hy>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=hy;
SendBuff4[i++]=ctemp;
temp+=ctemp;

if(hz<0)hz=32768-hz;
ctemp=hz>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=hz;
SendBuff4[i++]=ctemp;
temp+=ctemp;

SendBuff4[i++]=temp%256;
SendBuff4[i++]=(0xaa);
//
 temp=0xaF+2+2;
SendBuff4[i++]=(0xa5);
SendBuff4[i++]=(0x5a);
SendBuff4[i++]=(14+4);
SendBuff4[i++]=(0xA1);


if(yaw<0)yaw=32768-yaw;
ctemp=yaw>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=yaw;							
SendBuff4[i++]=ctemp;
temp+=ctemp;

if(pitch<0)pitch=32768-pitch;
ctemp=pitch>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=pitch;
SendBuff4[i++]=ctemp;
temp+=ctemp;
							 
if(roll<0)roll=32768-roll;
ctemp=roll>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=roll;
SendBuff4[i++]=ctemp;
temp+=ctemp;

if(alt<0)alt=32768-alt;
ctemp=alt>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;				
ctemp=alt;
SendBuff4[i++]=ctemp;
temp+=ctemp;

if(tempr<0)tempr=32768-tempr;
ctemp=tempr>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=tempr;
SendBuff4[i++]=ctemp;	   
temp+=ctemp;

if(press<0)press=32768-press;
ctemp=press>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=press;
SendBuff4[i++]=ctemp;
temp+=ctemp;

ctemp=IMUpersec>>8;
SendBuff4[i++]=ctemp;
temp+=ctemp;
ctemp=IMUpersec;
SendBuff4[i++]=ctemp;
temp+=ctemp;

SendBuff4[i++]=(temp%256);
SendBuff4[i++]=(0xaa);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

