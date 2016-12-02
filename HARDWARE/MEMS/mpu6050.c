#include "../HARDWARE/MEMS/mpu6050.h"
#include "../HARDWARE/MEMS/hml_sample.h"
#include "../HARDWARE/MATH/my_math.h"
#include "../HARDWARE/DRIVER/iic.h"
#include "../HARDWARE/include.h"

MPU6050_STRUCT mpu6050_fc,mpu6050;



void MEMS_CAL(void)
{
static u16 cnt,cnt1,cnt2;
//Gro
if(mpu6050.Gyro_CALIBRATE==1)
	cnt++;

if(cnt>2/0.5)
{
cnt=0;
mpu6050.Gyro_CALIBRATE=0;
}

//Acc
if(mpu6050.Acc_CALIBRATE==1)
	cnt1++;

if(cnt1>2/0.5)
{
cnt1=0;
mpu6050.Acc_CALIBRATE=0;
}

//HML
if(ak8975.Mag_CALIBRATED==1)
	cnt2++;

if(cnt2>10/0.5)
{
cnt2=0;
ak8975.Mag_CALIBRATED=0;
}

}

MPU6050_STRUCT mpu6050_r,mpu6050_fc_r;



u8 mpu6050_buffer[14];
u8 mpu6050_ok;
void MPU6050_Read(void)
{
	I2C_FastMode = 1;
	IIC_Read_nByte(MPU6050_ADDR,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
}


/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
失败为0
*******************************************************************************/ 
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
	u8 b;
	IIC_Read_nByte(dev, reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	mpu6050_ok = !( IIC_Write_1Byte(dev, reg, b) );
}
/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
reg	   寄存器地址
bitStart  目标字节的起始位
length   位长度
data    存放改变目标字节位的值
返回   成功 为1 
失败为0
*******************************************************************************/ 
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{
	
	u8 b,mask;
	IIC_Read_nByte(dev, reg, 1, &b);
	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	b &= mask;
	b |= data;
	IIC_Write_1Byte(dev, reg, b);
}

/**************************实现函数********************************************
*函数原型:		
*功　　能:	    设置 采样率
*******************************************************************************/
void MPU6050_set_SMPLRT_DIV(uint16_t hz)
{
	IIC_Write_1Byte(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV,1000/hz - 1);
		//I2C_Single_Write(MPU6050_ADDRESS,MPU_RA_SMPLRT_DIV, (1000/sample_rate - 1));
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
	
}
/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG,7, 3, 0x00);   //不自检
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG,7, 3, 0x00);   //不自检
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
enabled =1   睡觉
enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_ADDR, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/**************************实现函数********************************************
*函数原型:		
*功　　能:	    设置低通滤波截止频率
*******************************************************************************/
void MPU6050_setDLPF(uint8_t mode)
{
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void MPU6050_INT_Config()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOD, GPIO_Pin_7);	

}
/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_Init(u16 lpf)
{ 
	u16 default_filter = 1;
	
	MPU6050_INT_Config();
	
  switch(lpf)
	{
	case 5:
			default_filter = MPU6050_DLPF_BW_5;
			break;
	case 10:
			default_filter = MPU6050_DLPF_BW_10;
			break;
	case 20:
			default_filter = MPU6050_DLPF_BW_20;
			break;
	case 42:
			default_filter = MPU6050_DLPF_BW_42;
			break;
	case 98:
			default_filter = MPU6050_DLPF_BW_98;
			break;
	case 188:
			default_filter = MPU6050_DLPF_BW_188;
			break;
	case 256:
			default_filter = MPU6050_DLPF_BW_256;
			break;
	default:
			default_filter = MPU6050_DLPF_BW_42;
			break;
	}
	//I2c_Soft_Init();

	//设备复位
//	IIC_Write_1Byte(MPU6050_ADDR,MPU6050_RA_PWR_MGMT_1, 0x80);
	
	MPU6050_setSleepEnabled(0); //进入工作状态
	Delay_ms(10);
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO); //设置时钟  0x6b   0x03
	Delay_ms(10);
	MPU6050_set_SMPLRT_DIV(1000);  //1000hz
	Delay_ms(10);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
	Delay_ms(10);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_8);	//加速度度最大量程 +-8G
	Delay_ms(10);
	MPU6050_setDLPF(default_filter);  //42hz
	Delay_ms(10);
	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
	Delay_ms(10);
	MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
	Delay_ms(10);
}

s32 sum_temp[7]={0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,gyro_sum_cnt = 0;

void MPU6050_Data_Offset()
{static u8 acc_cal_temp=0,gro_cal_temp=0;
static u8 state_cal_acc,state_cal_gro;
static u8 init;
if(time_1s>5){
	switch(state_cal_acc)
	{
		case 0:if(mpu6050_fc_r.Acc_CALIBRATE!=acc_cal_temp)
		{				mpu6050_fc.Acc_CALIBRATE=1;state_cal_acc=1;}break;
		case 1:if(mpu6050_fc.Acc_CALIBRATE==0)
		{			acc_cal_temp=mpu6050_fc_r.Acc_CALIBRATE;state_cal_acc=0;}break;
	}
	
	switch(state_cal_gro)
	{
		case 0:if(mpu6050_fc_r.Gyro_CALIBRATE!=gro_cal_temp)
		{				mpu6050_fc.Gyro_CALIBRATE=1;state_cal_gro=1;}break;
		case 1:if(mpu6050_fc.Gyro_CALIBRATE==0)
		{			gro_cal_temp=mpu6050_fc_r.Gyro_CALIBRATE;state_cal_gro=0;}break;
	}
}
else
{
acc_cal_temp=mpu6050_fc_r.Acc_CALIBRATE;
gro_cal_temp=mpu6050_fc_r.Gyro_CALIBRATE;
}	


	if(mpu6050_fc.Acc_CALIBRATE == 1)
	{
    acc_sum_cnt++;
		sum_temp[A_X] += mpu6050_fc.Acc_I16.x;
		sum_temp[A_Y] += mpu6050_fc.Acc_I16.y;
		sum_temp[A_Z] += mpu6050_fc.Acc_I16.z - 65536/16;   // +-8G
		sum_temp[TEM] += mpu6050_fc.Tempreature;

    if( acc_sum_cnt >= OFFSET_AV_NUM )
		{
			mpu6050_fc.Acc_Offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
			mpu6050_fc.Acc_Offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
			mpu6050_fc.Acc_Offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
			mpu6050_fc.Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
			acc_sum_cnt =0;
			mpu6050_fc.Acc_CALIBRATE = 0;
		//	Param_SaveAccelOffset(&mpu6050.Acc_Offset);
			sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
		}	
	}



	if(mpu6050_fc.Gyro_CALIBRATE)
	{
		gyro_sum_cnt++;
		sum_temp[G_X] += mpu6050_fc.Gyro_I16.x;
		sum_temp[G_Y] += mpu6050_fc.Gyro_I16.y;
		sum_temp[G_Z] += mpu6050_fc.Gyro_I16.z;
		sum_temp[TEM] += mpu6050_fc.Tempreature;

    if( gyro_sum_cnt >= OFFSET_AV_NUM )
		{
			mpu6050_fc.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
			mpu6050_fc.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
			mpu6050_fc.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
			mpu6050_fc.Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
			gyro_sum_cnt =0;
			if(mpu6050_fc.Gyro_CALIBRATE == 1)
					WRITE_PARM();//Param_SaveGyroOffset(&mpu6050.Gyro_Offset);
			mpu6050_fc.Gyro_CALIBRATE = 0;
			sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
		}
	}
}


void Transform(float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
	*it_x = itx;
	*it_y = ity;
	*it_z = itz;

}
#define  IIR_ORDER     4      //使用IIR滤波器的阶数
static double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
static double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
static double InPut_IIR[3][IIR_ORDER+1] = {0};
static double OutPut_IIR[3][IIR_ORDER+1] = {0};
static double b_IIR_gro[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
static double a_IIR_gro[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
static double InPut_IIR_gro[3][IIR_ORDER+1] = {0};
static double OutPut_IIR_gro[3][IIR_ORDER+1] = {0};

s16 FILT_BUF[7][(FILTER_NUM + 1)];
uint8_t filter_cnt = 0,filter_cnt_old = 0;

float mpu6050_tmp[7],mpu6050_tmp_acc_circle[7];
float mpu_fil_tmp[7],mpu_fil_tmp_acc_circle[7];
float test_ang =0,test_ang_old=0,test_ang_d,test_fli_a,test_i;
int acc_off[3]={-7,11,40};
float acc_scale[3]={1.0007,1.0018,1.0156};//x y z
int acc_max[3]={4100  ,4123 ,3950},acc_min[3]={ -4035 ,-4100 ,-4360};
//int acc_max[3]={4182 ,4120 ,4444},acc_min[3]={ -4053 ,-4139 ,-3900};
u8 EN_TUO_ACC_FIX=0;
void MPU6050_Data_Prepare(float T)
{	
	u8 i;
	s32 FILT_TMP[7] = {0,0,0,0,0,0,0};
  float Gyro_tmp[3];
	

	MPU6050_Data_Offset(); //校准函数

	/*读取buffer原始数据*/
	mpu6050_fc.Acc_I16.x = -((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;
	mpu6050_fc.Acc_I16.y = -((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
	mpu6050_fc.Acc_I16.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;
 
	mpu6050_fc.Gyro_I16.x = -((((int16_t)mpu6050_buffer[ 8]) << 8) | mpu6050_buffer[ 9]) ;
	mpu6050_fc.Gyro_I16.y = -((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) ;
	mpu6050_fc.Gyro_I16.z = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) ;

	Gyro_tmp[0] = mpu6050_fc.Gyro_I16.x ;//
  Gyro_tmp[1] = mpu6050_fc.Gyro_I16.y ;//
	Gyro_tmp[2] = mpu6050_fc.Gyro_I16.z ;//

	mpu6050_fc.Tempreature = ((((int16_t)mpu6050_buffer[6]) << 8) | mpu6050_buffer[7]); //tempreature
	mpu6050_fc.TEM_LPF += 2 *3.14f *T *(mpu6050_fc.Tempreature - mpu6050_fc.TEM_LPF);
	mpu6050_fc.Ftempreature = mpu6050_fc.TEM_LPF/340.0f + 36.5f;

//======================================================================
	if( ++filter_cnt > FILTER_NUM )	
	{
		filter_cnt = 0;
		filter_cnt_old = 1;
	}
	else
	{
		filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);
	}
//10 170 4056
	/* 得出校准后的数据 */

	switch(mcuID[0]){
		case DRONE_350_ID://300
						acc_max[0]=4100;acc_max[1]=4123;acc_max[2]=3950;  acc_min[0]= -4035;acc_min[1]=-4100 ;acc_min[2]=-4360;	
		break;
		case DRONE_330_ID://450
						acc_max[0]=4180;acc_max[1]=4140;acc_max[2]=4470;  acc_min[0]= -4050;acc_min[1]=-4120 ;acc_min[2]=-3890;
		break;
		default:acc_max[0]=4100;acc_max[1]=4123;acc_max[2]=3950;  acc_min[0]= -4035;acc_min[1]=-4100 ;acc_min[2]=-4360;
		break;
		}
	
	if (EN_TUO_ACC_FIX){
	
//	for(i=0;i<3;i++){
//	acc_off[i]=acc_max[i]+acc_min[i];
//	acc_scale[i]=(float)(4096*2)/(acc_max[i]-acc_min[i]);
//	}
  mpu6050_tmp[A_X]=((float)(mpu6050_fc.Acc_I16.x - acc_off[0])) / acc_scale[0];
  mpu6050_tmp[A_Y]=((float)(mpu6050_fc.Acc_I16.y - acc_off[1])) / acc_scale[1];
  mpu6050_tmp[A_Z]=((float)(mpu6050_fc.Acc_I16.z - acc_off[2])) / acc_scale[2];
}
	else{
	mpu6050_tmp[A_X] = (mpu6050_fc.Acc_I16.x - mpu6050_fc.Acc_Offset.x) ;
	mpu6050_tmp[A_Y] = (mpu6050_fc.Acc_I16.y - mpu6050_fc.Acc_Offset.y) ;
	mpu6050_tmp[A_Z] = (mpu6050_fc.Acc_I16.z - mpu6050_fc.Acc_Offset.z) ;
	}
	mpu6050_tmp[G_X] = Gyro_tmp[0] - mpu6050_fc.Gyro_Offset.x ;//
	mpu6050_tmp[G_Y] = Gyro_tmp[1] - mpu6050_fc.Gyro_Offset.y ;//
	mpu6050_tmp[G_Z] = Gyro_tmp[2] - mpu6050_fc.Gyro_Offset.z ;//
	

	/* 更新滤波滑动窗口数组 */
	FILT_BUF[A_X][filter_cnt] = mpu6050_tmp[A_X];
	FILT_BUF[A_Y][filter_cnt] = mpu6050_tmp[A_Y];
	FILT_BUF[A_Z][filter_cnt] = mpu6050_tmp[A_Z];
	FILT_BUF[G_X][filter_cnt] = mpu6050_tmp[G_X]; 
	FILT_BUF[G_Y][filter_cnt] = mpu6050_tmp[G_Y];
	FILT_BUF[G_Z][filter_cnt] = mpu6050_tmp[G_Z];

	for(i=0;i<FILTER_NUM;i++)
	{
		FILT_TMP[A_X] += FILT_BUF[A_X][i];
		FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
		FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
		FILT_TMP[G_X] += FILT_BUF[G_X][i];
		FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
		FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
	}
#define EN_MPU_FILT 1
#if EN_MPU_FILT	
mpu_fil_tmp[A_X]= IIR_I_Filter( mpu6050_tmp[A_X], InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
mpu_fil_tmp[A_Y]= IIR_I_Filter( mpu6050_tmp[A_Y], InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
mpu_fil_tmp[A_Z]= IIR_I_Filter( mpu6050_tmp[A_Z], InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
mpu_fil_tmp[G_X] = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
mpu_fil_tmp[G_Y] = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
mpu_fil_tmp[G_Z] = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;
#else
mpu_fil_tmp[A_X]= mpu6050_tmp[A_X];
mpu_fil_tmp[A_Y]= mpu6050_tmp[A_Y];
mpu_fil_tmp[A_Z]= mpu6050_tmp[A_Z];
mpu_fil_tmp[G_X]= mpu6050_tmp[G_X];
mpu_fil_tmp[G_Y]= mpu6050_tmp[G_Y];
mpu_fil_tmp[G_Z]= mpu6050_tmp[G_Z];
#endif
//	mpu_fil_tmp_acc_circle[A_X]= IIR_I_Filter( mpu6050_tmp_acc_circle[A_X], InPut_IIR_gro[0], OutPut_IIR_gro[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	mpu_fil_tmp_acc_circle[A_Y]= IIR_I_Filter( mpu6050_tmp_acc_circle[A_Y], InPut_IIR_gro[1], OutPut_IIR_gro[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	mpu_fil_tmp_acc_circle[A_Z]= IIR_I_Filter( mpu6050_tmp_acc_circle[A_Z], InPut_IIR_gro[2], OutPut_IIR_gro[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	Transform(mpu_fil_tmp_acc_circle[A_X],mpu_fil_tmp_acc_circle[A_Y],mpu_fil_tmp_acc_circle[A_Z],&mpu6050.Acc_c.x,&mpu6050.Acc_c.y,&mpu6050.Acc_c.z);
	/*坐标转换*/
	Transform(mpu_fil_tmp[A_X],mpu_fil_tmp[A_Y],mpu_fil_tmp[A_Z],&mpu6050_fc.Acc.x,&mpu6050_fc.Acc.y,&mpu6050_fc.Acc.z);
	Transform(mpu_fil_tmp[G_X],mpu_fil_tmp[G_Y],mpu_fil_tmp[G_Z],&mpu6050_fc.Gyro.x,&mpu6050_fc.Gyro.y,&mpu6050_fc.Gyro.z);

	mpu6050_fc.Gyro_deg.x = mpu6050_fc.Gyro.x *TO_ANGLE;
	mpu6050_fc.Gyro_deg.y = mpu6050_fc.Gyro.y *TO_ANGLE;
	mpu6050_fc.Gyro_deg.z = mpu6050_fc.Gyro.z *TO_ANGLE;
	

	
//======================================================================
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
