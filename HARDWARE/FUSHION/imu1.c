//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files
#include "../HARDWARE/MATH/my_math.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include <math.h>
#include "../HARDWARE/include.h"
//#include "../HARDWARE/FUSHION/kf_oldx_yaw.h"


float K_yaw=1;
//匿名飞控姿态解算
float q_nav[4];
xyz_f_t reference_v_fc,reference_v;
ref_t 	ref;
float reference_vr[3],reference_vr_fc[3];
float Roll,Pitch,Yaw,yaw_mag_view[5];    				//姿态角
float Roll_mid_down,Pitch_mid_down,Yaw_mid_down;    				//姿态角
float ref_q[4] = {1,0,0,0};
volatile float q0_m = 1.0f, q1_m = 0.0f, q2_m = 0.0f, q3_m = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
float ref_q_imd_down[4] = {1,0,0,0};
float reference_vr_imd_down[3];

float Kp =0.6f;//2.25f;//0.6f   ;             	// proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki =0.1f    ;            	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)

float q_nav[4];
xyz_f_t reference_v;
ref_t 	ref;
float reference_vr[3];
//xyz_f_t Gravity_Vec;  				//解算的重力向量
	
float Roll,Pitch,Yaw;    				//姿态角
float Roll_mid_down,Pitch_mid_down,Yaw_mid_down;    				//姿态角
float norm_acc,norm_q;
float norm_acc_lpf;
xyz_f_t mag_sim_3d;
extern u8 fly_ready;

int test_flag[3]={-1,-1,1};
int test_flag1[3]={-1,-1,1};
int test_flag2[3]={1,1,1};
//use
float mag_norm ,mag_norm_xyz, yaw_mag_view[5];

//------------------KF  parameter------------------
float gh_yaw=0.1;
float ga_yaw=0.1;//<---use
float gw_yaw=0.1;
float yaw_kf;
double P_kf_yaw[4]={1,0,0,1}; 
double X_kf_yaw[2]={0,0};
#if USE_MINI_BOARD
float k_kf_z=2;
#else
float k_kf_z=1.428;
#endif
u8 yaw_cross;
u8 dis_angle_lock=0;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float hx,float hy,float hz,float *rol,float *pit,float *yaw) 
{		
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static xyz_f_t mag_tmp;
	static float yaw_mag;
	static u16 cnt;
	static u8 init;
	if(cnt++>256||init)
	{
	init=1;
	
	}else{
	X_kf_yaw[0]=yaw_mag_view[3];
	X_kf_yaw[1]=0;
	}
	
	
	#if !IMU_HML_ADD_500
	mag_norm_tmp = 20 *(6.28f *half_T);	
	
	mag_norm_xyz = my_sqrt(hx * hx + hy * hy + hz * hz);
	
		if( mag_norm_xyz != 0)
	{
		mag_tmp.x += mag_norm_tmp *( test_flag[0]*(float)hx /( mag_norm_xyz ) - mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *( test_flag[1]*(float)hy /( mag_norm_xyz ) - mag_tmp.y);	
		mag_tmp.z += mag_norm_tmp *( test_flag[2]*(float)hz /( mag_norm_xyz ) - mag_tmp.z);	
//		mag_tmp.x += mag_norm_tmp *( (float)ak8975.Mag_Val.x /( mag_norm_xyz ) - mag_tmp.x);
//		mag_tmp.y += mag_norm_tmp *( (float)ak8975.Mag_Val.y /( mag_norm_xyz ) - mag_tmp.y);	
//		mag_tmp.z += mag_norm_tmp *( (float)-ak8975.Mag_Val.z /( mag_norm_xyz ) - mag_tmp.z);	
	}
  xyz_f_t reference_v_fc_temp;
	reference_v_fc_temp.x=reference_vr_imd_down_fc[0];
	reference_v_fc_temp.y=reference_vr_imd_down_fc[1];
	reference_v_fc_temp.z=reference_vr_imd_down_fc[2];
	
	simple_3d_trans(&reference_v_fc_temp,&mag_tmp,&mag_sim_3d);
	
	mag_norm = my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);
	
	if(dis_angle_lock||(mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0 && fabs(Pit_fc)<12 && fabs(Rol_fc)<12))
	{
		
		yaw_mag_view[1] = fast_atan2( ( mag_sim_3d.y/mag_norm ) , ( mag_sim_3d.x/mag_norm) ) *57.3f;	
	}
	else
		yaw_mag_view[1]=X_kf_yaw[0];
	
	#endif
	  float calMagY,calMagX,magTmp2[3],euler[3];
	#if !IMU_HML_ADD_500
	magTmp2[0]=test_flag1[0]*hx;
	magTmp2[1]=test_flag1[1]*hy;
	magTmp2[2]=test_flag1[2]*hz;
	euler[1]=Pit_fc/RAD_DEG *test_flag2[0] ;
	euler[0]=Rol_fc/RAD_DEG *test_flag2[1] ;
	  calMagY = magTmp2[0] * sin(euler[1])* sin(euler[0]) 
							+ magTmp2[1] * cos(euler[0])
							-magTmp2[2] * cos(euler[1]) * sin(euler[0]); //倾斜补偿磁力计的X轴分量
		calMagX = magTmp2[0] * cos(euler[1]) + magTmp2[2] * cos(euler[1]); //倾斜补偿磁力计的Y轴分量
			//calMagY = mz * sin(sita) + my * cos(sita); //倾斜补偿磁力计的Y轴分量
 // calMagX = mx * cos(fi) + my * sin(fi) * sin(sita) - mz * sin(fi) * cos(sita); //倾斜补偿磁力计的X轴分量
	if(dis_angle_lock||( fabs(Pit_fc)<12 && fabs(Rol_fc)<12))
	//yaw_mag_view[0] =atan(calMagY/(calMagX+0.00001))* RAD_DEG;// fast_atan2(calMagY, calMagX) * RAD_DEG; //计算Yaw (-PI < Roll < PI) 并将弧度转化成角度
	yaw_mag_view[0] =fast_atan2(calMagY, calMagX) * RAD_DEG; 
	else
	yaw_mag_view[0]=X_kf_yaw[0];
	#endif
	
	#if IMU_HML_ADD_500
		magTmp2[0]=hx;
		magTmp2[1]=hy;
		magTmp2[2]=hz;
		euler[1]=Pit_fc/RAD_DEG  ;
		euler[0]=Rol_fc/RAD_DEG  ;
    calMagY = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1])* sin(euler[0])+magTmp2[2] * sin(euler[1]) * cos(euler[0]); 
    calMagX = magTmp2[1] * cos(euler[0]) + magTmp2[2] * sin(euler[0]);
	if( dis_angle_lock||(fabs(Pit_fc)<12 && fabs(Rol_fc)<12))
  yaw_mag_view[4]=To_180_degrees(fast_atan2(calMagX,calMagY)* RAD_DEG +180);
	else
	yaw_mag_view[4]=X_kf_yaw[0];
  #endif
	
	
	float tempy;
	
	#if !IMU_HML_ADD_500
	if(yaw_mag_view[0]*yaw_mag_view[1]>0)
	tempy=yaw_mag_view[0]/2+yaw_mag_view[1]/2;
	else
	#endif	
	#if IMU_HML_ADD_500
	tempy=yaw_mag_view[4];///2+yaw_mag_view[0]/2;
	#endif	
	yaw_mag_view[3]=Moving_Median( 18,15,tempy);	
	
	
	double Z_yaw[2]={ yaw_mag_view[3] , 0 };
	if(yaw_mag_view[3]*X_kf_yaw[0]<0&&!(fabs(yaw_mag_view[3])<90))
	{Z_yaw[0]=X_kf_yaw[0];yaw_cross=1;}
	else
	yaw_cross=0;
	kf_oldx_yaw(X_kf_yaw,P_kf_yaw,Z_yaw ,-gz*k_kf_z,gh_yaw,ga_yaw,gw_yaw,half_T*2);

	yaw_kf=Moving_Median( 19,5,X_kf_yaw[0]);
	//=============================================================================
	#if !IMU_HML_ADD_500
	yaw_mag=yaw_mag_view[3] ;

	// 计算等效重力向量//十分重要
	if(mode.en_imu_ekf==0){
	reference_vr_fc[0]=reference_v_fc.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_vr_fc[1]=reference_v_fc.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_vr_fc[2]=reference_v_fc.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);
	}

	// 计算加速度向量的模
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   
	norm_acc_lpf +=  NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001


	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//把加计的三维向量转成单位向量。
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			/* 叉乘得到误差 */
			ref.err_tmp.x = ay*reference_v_fc.z - az*reference_v_fc.y;
			ref.err_tmp.y = az*reference_v_fc.x - ax*reference_v_fc.z;
	    //ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;
			
			/* 误差低通 */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
	//			 ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );
			
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
//				ref.err.z = ref.err_lpf.z ;
		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;
//		ref.err.z = 0 ;
	}
	/* 误差积分 */
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	
	/* 积分限幅 */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	
	if( reference_v.z > 0.0f )
	{
		if( fly_ready||(fabs(Pit_fc)>10)||(fabs(Rol_fc)>10)  )
		{
	//	yaw_correct = Kp *0.2f *To_180_degrees(yaw_mag - YAW_R);
			yaw_correct = Kp *0.1f *LIMIT( my_deathzoom( To_180_degrees(yaw_mag - Yaw_fc), 10),-20,20 )*K_yaw;
			//已经解锁，只需要低速纠正。
		}
		else
		{
			yaw_correct = Kp *1.5f *To_180_degrees(yaw_mag - Yaw_fc)*K_yaw;
			//没有解锁，视作开机时刻，快速纠正
		}
// 		if( yaw_correct>360 || yaw_correct < -360  )
// 		{
// 			yaw_correct = 0;
// 			//限制纠正范围+-360，配合+-180度取值函数
// 		}
	}

	
	ref.g.x = (gx - reference_v_fc.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = (gy - reference_v_fc.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	ref.g.z = (gz - reference_v_fc.z *yaw_correct) *ANGLE_TO_RADIAN;
	
	/* 用叉积误差来做PI修正陀螺零偏 */

	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  

	/* 四元数规一化 normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;
	
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
	if(!mode.yaw_sel)
	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;//
	else
	#endif
	*yaw =yaw_kf;	
}


//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	400.0f		// sample frequency in Hz


//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = 0.01f;								// 2 * proportional gain (Kp)
volatile float q0_m_fc = 1.0f, q1_m_fc = 0.0f, q2_m_fc = 0.0f, q3_m_fc = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
volatile float q0_fc = 1.0f, q1_fc = 0.0f, q2_fc = 0.0f, q3_fc = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
	float ref_q_imd_down_fc[4] = {1,0,0,0};
	float reference_vr_imd_down_fc[3];
	u8 init_q=1;
void MadgwickAHRSupdate(float dt,float gx, float gy, float gz, float ax, float ay, float az, 
	float mx, float my, float mz,float *rol,float *pit,float *yaw){
  float T;
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  static u16 init_cnt;
		if(init_cnt++>100){init_cnt=100+1;T=dt;
		}
		else 
		{		
	  if(init_q){
    float Pit,Rol;
    Pit=-atan(mpu6050_fc.Acc.x/mpu6050_fc.Acc.z)*57.3;
		Rol=atan(mpu6050_fc.Acc.y/mpu6050_fc.Acc.z)*57.3;
			
		#if IMU_HML_ADD_500
		float magTmp2 [3];	
		magTmp2[0]=ak8975_fc.Mag_Val.x;
		magTmp2[1]=ak8975_fc.Mag_Val.y;
		magTmp2[2]=ak8975_fc.Mag_Val.z;
		float euler[2]; 	
		euler[1]=Pit*0.0173  ;
		euler[0]=Rol*0.0173  ;
		float calMagY = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1])* sin(euler[0])+magTmp2[2] * sin(euler[1]) * cos(euler[0]); 
		float calMagX = magTmp2[1] * cos(euler[0]) + magTmp2[2] * sin(euler[0]);
		float yaw_mag=To_180_degrees(fast_atan2(calMagX,calMagY)* RAD_DEG +180);
		#endif	
		float angle_cal[3];
		angle_cal[0]=euler[0];
		angle_cal[1]=euler[1];
		angle_cal[2]=yaw_mag;
		euler_to_q(angle_cal,ref_q_imd_down_fc);
	  q0_fc=ref_q_imd_down_fc[0];
	  q1_fc=ref_q_imd_down_fc[1];
	  q2_fc=ref_q_imd_down_fc[2];
	  q3_fc=ref_q_imd_down_fc[3];	
	  }T=2*dt;
		
	}
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {

	MadgwickAHRSupdateIMU(T,gx, gy, gz, ax, ay, az);
	ref_q_imd_down_fc[0]=q0_fc;
	ref_q_imd_down_fc[1]=q1_fc;
	ref_q_imd_down_fc[2]=q2_fc;
	ref_q_imd_down_fc[3]=q3_fc;
	reference_vr_imd_down_fc[0] = 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2]);
	reference_vr_imd_down_fc[1] = 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]);
	reference_vr_imd_down_fc[2] = 1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2]);
	*rol = atan2(2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]),1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2])) *57.3f;
	*pit = asin(2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q_imd_down_fc[1]*ref_q_imd_down_fc[2] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[3]), 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[0] + ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1]) - 1) *57.3f  ;// 

		return;
	}
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float dt,float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1_fc * gx - q2_fc * gy - q3_fc * gz);
	qDot2 = 0.5f * (q0_fc * gx + q2_fc * gz - q3_fc * gy);
	qDot3 = 0.5f * (q0_fc * gy - q1_fc * gz + q3_fc * gx);
	qDot4 = 0.5f * (q0_fc * gz + q1_fc * gy - q2_fc * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0_fc;
		_2q1 = 2.0f * q1_fc;
		_2q2 = 2.0f * q2_fc;
		_2q3 = 2.0f * q3_fc;
		_4q0 = 4.0f * q0_fc;
		_4q1 = 4.0f * q1_fc;
		_4q2 = 4.0f * q2_fc;
		_8q1 = 8.0f * q1_fc;
		_8q2 = 8.0f * q2_fc;
		q0q0 = q0_fc * q0_fc;
		q1q1 = q1_fc * q1_fc;
		q2q2 = q2_fc * q2_fc;
		q3q3 = q3_fc * q3_fc;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_fc - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2_fc + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3_fc - _2q1 * ax + 4.0f * q2q2 * q3_fc - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0_fc += qDot1 * dt;
	q1_fc += qDot2 * dt;
	q2_fc += qDot3 * dt;
	q3_fc += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0_fc * q0_fc + q1_fc * q1_fc + q2_fc * q2_fc + q3_fc * q3_fc);
	q0_fc *= recipNorm;
	q1_fc *= recipNorm;
	q2_fc *= recipNorm;
	q3_fc *= recipNorm;
	
	

}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


//---------------------------------------------------------------------------------------------------
// Definitions



//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = 0.8;//(2.0f * 0.5f);											// 2 * proportional gain (Kp)
volatile float twoKi = 0.2;//(2.0f * 0.0f);											// 2 * integral gain (Ki)
float twoKp_s=1.6;
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki


//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(float dt,float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz
	,float *rol,float *pit,float *yaw) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
 static u16 init_cnt;
		if(init_cnt++>500){init_cnt=500+1;twoKp=twoKp_s;
		}
			else twoKp=2;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(dt,gx, gy, gz, ax, ay, az);
	ref_q_imd_down_fc[0]=q0_fc;
	ref_q_imd_down_fc[1]=q1_fc;
	ref_q_imd_down_fc[2]=q2_fc;
	ref_q_imd_down_fc[3]=q3_fc;
	reference_vr_imd_down_fc[0] = 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2]);
	reference_vr_imd_down_fc[1] = 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]);
	reference_vr_imd_down_fc[2] = 1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2]);
	*rol = fast_atan2(2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]),1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2])) *57.3f;
	*pit = asin(2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q_imd_down_fc[1]*ref_q_imd_down_fc[2] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[3]), 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[0] + ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1]) - 1) *57.3f  ;// 

		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0_fc * q0_fc;
        q0q1 = q0_fc * q1_fc;
        q0q2 = q0_fc * q2_fc;
        q0q3 = q0_fc * q3_fc;
        q1q1 = q1_fc * q1_fc;
        q1q2 = q1_fc * q2_fc;
        q1q3 = q1_fc * q3_fc;
        q2q2 = q2_fc * q2_fc;
        q2q3 = q2_fc * q3_fc;
        q3q3 = q3_fc * q3_fc;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0_fc;
	qb = q1_fc;
	qc = q2_fc;
	q0_fc += (-qb * gx - qc * gy - q3_fc * gz);
	q1_fc += (qa * gx + qc * gz - q3_fc * gy);
	q2_fc += (qa * gy - qb * gz + q3_fc * gx);
	q3_fc += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0_fc * q0_fc + q1_fc * q1_fc + q2_fc * q2_fc + q3_fc * q3_fc);
	q0_fc *= recipNorm;
	q1_fc *= recipNorm;
	q2_fc *= recipNorm;
	q3_fc *= recipNorm;
	
	
	ref_q_imd_down_fc[0]=q0_fc;
	ref_q_imd_down_fc[1]=q1_fc;
	ref_q_imd_down_fc[2]=q2_fc;
	ref_q_imd_down_fc[3]=q3_fc;
	reference_vr_imd_down_fc[0] = 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2]);
	reference_vr_imd_down_fc[1] = 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]);
	reference_vr_imd_down_fc[2] = 1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2]);
	*rol = fast_atan2(2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]),1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2])) *57.3f;
	*pit = asin(2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q_imd_down_fc[1]*ref_q_imd_down_fc[2] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[3]), 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[0] + ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1]) - 1) *57.3f  ;// 

}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float dt,float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1_fc * q3_fc - q0_fc * q2_fc;
		halfvy = q0_fc * q1_fc + q2_fc * q3_fc;
		halfvz = q0_fc * q0_fc - 0.5f + q3_fc * q3_fc;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dt;//(1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt;//(1.0f / sampleFreq);
			integralFBz += twoKi * halfez * dt;//(1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);//(1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * dt);////(1.0f / sampleFreq));
	gz *= (0.5f * dt);//(1.0f / sampleFreq));
	qa = q0_fc;
	qb = q1_fc;
	qc = q2_fc;
	q0_fc += (-qb * gx - qc * gy - q3_fc * gz);
	q1_fc += (qa * gx + qc * gz - q3_fc * gy);
	q2_fc+= (qa * gy - qb * gz + q3_fc * gx);
	q3_fc += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0_fc * q0_fc + q1_fc * q1_fc + q2_fc * q2_fc + q3_fc * q3_fc);
	q0_fc *= recipNorm;
	q1_fc *= recipNorm;
	q2_fc *= recipNorm;
	q3_fc *= recipNorm;
}


//====================================================================================================
// END OF CODE
//====================================================================================================
