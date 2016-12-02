#include "ident.h"
#include "../HARDWARE/define.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/MEMS/mpu6050.h"
//定义多组输入输出数据的存储
float CDataYrPitch[I_CONTROL_DATA_LENGTH];
float CDataYrRoll[I_CONTROL_DATA_LENGTH];
float CDataYrYaw[I_CONTROL_DATA_LENGTH];

float CDataYrGPitch[I_CONTROL_DATA_LENGTH];
float CDataYrGRoll[I_CONTROL_DATA_LENGTH];
float CDataYrGYaw[I_CONTROL_DATA_LENGTH];

float CDataYEPitch[I_CONTROL_DATA_LENGTH];
float CDataYERoll[I_CONTROL_DATA_LENGTH];
float CDataYEYaw[I_CONTROL_DATA_LENGTH];

float CDataYGPitch[I_CONTROL_DATA_LENGTH];
float CDataYGRoll[I_CONTROL_DATA_LENGTH];
float CDataYGYaw[I_CONTROL_DATA_LENGTH];

float CDataUPitch[I_CONTROL_DATA_LENGTH];
float CDataURoll[I_CONTROL_DATA_LENGTH];
float CDataUYaw[I_CONTROL_DATA_LENGTH];

//-----------------------------------------------------------------
float RELS_Pitch_xie = 0;   //噪声的估计值
float RELS_Pitch_xiek[I_CONTROL_DATA_LENGTH];   //噪声的估计值的存储
RELS_T rels_pitch;
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//定义滤波存储序列
float CDataUfkPitch[I_CONTROL_DATA_LENGTH];
float CDataYfkPitch[I_CONTROL_DATA_LENGTH];
PPCFFRELS_T cffrels_pitch;
float CDataUfkRoll[I_CONTROL_DATA_LENGTH];
float CDataYfkRoll[I_CONTROL_DATA_LENGTH];
PPCFFRELS_T cffrels_roll;

//定义实际作用在每个通道的控制量∑
float CEPower = 0;
float CERoll = 0;
float CEPitch = 0;
float CEYaw = 0;

//PID用的弧度角度转换
#define F_C2D (57.29578f)  //弧度转角度的乘数

void ident(void)
{
//用于循环更新状态值的循环变量
int StateRenewI = 0;  
static u8 init;
	if(!init){
	init=1;
			//给存储的数据赋初值
	memset(CDataYrPitch, 0.0, sizeof(CDataYrPitch));
	memset(CDataYrRoll, 0.0, sizeof(CDataYrRoll));
	memset(CDataYrYaw, 0.0, sizeof(CDataYrYaw));
	
	memset(CDataYrGPitch, 0.0, sizeof(CDataYrGPitch));
	memset(CDataYrGRoll, 0.0, sizeof(CDataYrGRoll));
	memset(CDataYrGYaw, 0.0, sizeof(CDataYrGYaw));
	
	memset(CDataYEPitch, 0.0, sizeof(CDataYEPitch));
	memset(CDataYERoll, 0.0, sizeof(CDataYERoll));
	memset(CDataYEYaw, 0.0, sizeof(CDataYEYaw));
	
	memset(CDataYGPitch, 0.0, sizeof(CDataYGPitch));
	memset(CDataYGRoll, 0.0, sizeof(CDataYGRoll));
	memset(CDataYGYaw, 0.0, sizeof(CDataYGYaw));
	
	memset(CDataUPitch, 0.0, sizeof(CDataUPitch));
	memset(CDataURoll, 0.0, sizeof(CDataURoll));
	memset(CDataUYaw, 0.0, sizeof(CDataUYaw));
		//RELS初始化
	RELS_init(&rels_pitch, RELS_NA, RELS_NB, RELS_NC, RELS_D);
	//自校正PID型控制器初始化，这里使用稳态参数初始化
	//这里默认Pitch和Roll的模型是一样的，反正到时候有自适应控制
	PPCFFRELS_initAsSteady(&cffrels_pitch, PPCFFRELS_NA, PPCFFRELS_NB, PPCFFRELS_D, PPCFFRELS_AM1, PPCFFRELS_AM2, PPCFFRELS_AM3);   //这句以后要包含在控制器函数初始化里面
	PPCFFRELS_initAsSteady(&cffrels_roll, PPCFFRELS_NA, PPCFFRELS_NB, PPCFFRELS_D, PPCFFRELS_AM1, PPCFFRELS_AM2, PPCFFRELS_AM3);   //这句以后要包含在控制器函数初始化里面
	}
	//定义实际得到的控制数据临时变量
	


			//更新状态，注意这里是逆着来的
		for(StateRenewI=I_CONTROL_DATA_LENGTH-1;StateRenewI>0;StateRenewI--) {  //循环到第二个，第一个为最新的变量
			CDataYrPitch[StateRenewI] = CDataYrPitch[StateRenewI-1];
			CDataYrRoll[StateRenewI] = CDataYrRoll[StateRenewI-1];
			CDataYrYaw[StateRenewI] = CDataYrYaw[StateRenewI-1];
			
			CDataYrGPitch[StateRenewI] = CDataYrGPitch[StateRenewI-1];
			CDataYrGRoll[StateRenewI] = CDataYrGRoll[StateRenewI-1];
			CDataYrGYaw[StateRenewI] = CDataYrGYaw[StateRenewI-1];
			
			CDataYEPitch[StateRenewI] = CDataYEPitch[StateRenewI-1];
			CDataYERoll[StateRenewI] = CDataYERoll[StateRenewI-1];
			CDataYEYaw[StateRenewI] = CDataYEYaw[StateRenewI-1];
			
			CDataYGPitch[StateRenewI] = CDataYGPitch[StateRenewI-1];
			CDataYGRoll[StateRenewI] = CDataYGRoll[StateRenewI-1];
			CDataYGYaw[StateRenewI] = CDataYGYaw[StateRenewI-1];
			
			CDataUPitch[StateRenewI] = CDataUPitch[StateRenewI-1];
			CDataURoll[StateRenewI] = CDataURoll[StateRenewI-1];
			CDataUYaw[StateRenewI] = CDataUYaw[StateRenewI-1];
			// 更新xiek
			RELS_Pitch_xiek[StateRenewI] = RELS_Pitch_xiek[StateRenewI-1];
			
			//更新滤波器的值，这里仅仅负责滚动，更新在单独的函数中
			CDataUfkPitch[StateRenewI] = CDataUfkPitch[StateRenewI-1];
			CDataYfkPitch[StateRenewI] = CDataYfkPitch[StateRenewI-1];
			CDataUfkRoll[StateRenewI] = CDataUfkRoll[StateRenewI-1];
			CDataYfkRoll[StateRenewI] = CDataYfkRoll[StateRenewI-1];
		}
	
		float ControlPitch=except_A.y   ;
		float ControlRoll= except_A.x   ;
    float ControlYaw=	 except_A.z	  ;	
		
		//获取这次的数据
		CDataYrPitch[0] = ControlPitch/F_C2D;//弧度制
		CDataYrRoll[0] = ControlRoll/F_C2D;
		CDataYrYaw[0] = ControlYaw/F_C2D;
		//角速度控制
		CDataYrGPitch[0] = ControlPitch/F_C2D;
		CDataYrGRoll[0] = ControlRoll/F_C2D;
		CDataYrGYaw[0] = ControlYaw/F_C2D;
		
		float numberVX = 0;  //记录陀螺仪的真实值
		float numberVY = 0;
		float numberVZ = 0;
		float OmegaBX = 0;
		float OmegaBY = 0;
		float OmegaBZ = 0;

		//定义角度控制用的欧拉角，这里心中还是有些疑惑，就设置成static了，理论上是不用的
	  float Phi = Roll;//x
		float Theta = Pitch;//y
		float Psi = Yaw;//z

		CDataYEPitch[0] = Theta;
		CDataYERoll[0] = Phi;
		CDataYEYaw[0] = Psi;
		
		OmegaBX=-mpu6050.Gyro_deg.x;
		OmegaBY=mpu6050.Gyro_deg.y;
		OmegaBZ=mpu6050.Gyro_deg.z;
		CDataYGPitch[0] = OmegaBY;
		CDataYGRoll[0] = OmegaBX;
		CDataYGYaw[0] = OmegaBZ;
	
		CDataUPitch[1] = CEPitch;  //注意这里记录的其实是上一次的值，因为这里默认系统始终存在一阶延迟
		CDataURoll[1] = CERoll;
		CDataUYaw[1] = CEYaw;
		
		//更新xiek，这句也被放到这里来了,也要注意得到的其实是上一次的xie
		RELS_Pitch_xiek[1] = RELS_Pitch_xie;
		
		//这里默认积分使能则表示灰机起飞了，则开始递推最小二乘辨识------------------------------------------------
		RELS_Pitch_xie = RELS_Update(&rels_pitch, CDataYEPitch, CDataUPitch, RELS_Pitch_xiek);
		
	  //             结构体           buf素质       buf——reg       姿态角         控制量
		updateSTCPP(&cffrels_pitch, CDataYfkPitch, CDataUfkPitch, CDataYEPitch, CDataUPitch);
		updateSTCPP(&cffrels_roll, CDataYfkRoll, CDataUfkRoll, CDataYERoll, CDataURoll);


		//updateSTCPPFilterOnly(&cffrels_pitch, CDataYfkPitch, CDataUfkPitch, CDataYEPitch, CDataUPitch);
		//updateSTCPPFilterOnly(&cffrels_roll, CDataYfkRoll, CDataUfkRoll, CDataYERoll, CDataURoll);
	//out-put
		CEPitch=ctrl_1.out.y;
		CERoll= ctrl_1.out.x;
		CEYaw=  ctrl_1.out.z;
		
#ifdef _IDENTIFICATION_PITCH_
				CERoll = 0;
				CEPitch = ((float)(rand()%200-100))*0.2f;  //噪声范围为-20~20
				CEYaw = 0; 
	
#endif 
#ifdef _IDENTIFICATION_ROLL_
				CERoll = ((float)(rand()%200-100))*0.4f;  //噪声范围为-40~40
				CEPitch = 0;
				CEYaw = 0; 

#endif 
#ifdef _IDENTIFICATION_YAW_
				//CERoll = 0;
				//CEPitch = 0;  这里使用有PID控制的方式来辅助找平
				CEYaw = ((float)(rand()%200-100))*0.3f;  //噪声范围为-30~30

#endif 
#ifdef _IDENTIFICATION_ALL_WITH_CONTROL_
				//CERoll += ((float)(rand()%200-100))*0.01f;  //直接给电机的输入加上±1的白噪声
				CEPitch += ((float)(rand()%200-100))*0.01f;  //直接给电机的输入加上±1的白噪声
				//CEYaw += ((float)(rand()%200-100))*0.01f;  //直接给电机的输入加上±1的白噪声
#endif 				

}