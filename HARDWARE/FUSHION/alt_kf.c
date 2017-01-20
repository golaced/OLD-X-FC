#include "../HARDWARE/FUSHION/alt_kf.h"
#include "../HARDWARE/define.h"
#include "../HARDWARE/MATH/my_math.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include "../HARDWARE/MATH/Quaternion.h"
#include "../HARDWARE/MEMS/ms5611_2.h"
#include "../HARDWARE/MEMS/mpu6050.h"
#include "../HARDWARE/DRIVER/usart_fc.h"
#include "../HARDWARE/CONTROL/height_ctrl.h"
#include "../HARDWARE/include.h"
//#include "../HARDWARE/UKF_BARO/kf_oldx.h"
#include "../HARDWARE/EKF_BARO/baro_ekf_oldx.h"
//sonar
float ALT_POS_BMP,ALT_VEL_BMP;
float ALT_POS_SONAR,ALT_VEL_SONAR,ALT_POS_SONAR2,ALT_POS_SONAR3;
float ALT_POS_SONAR2,ALT_POS_SONAR3;
float ALT_POS_BMP_EKF,ALT_VEL_BMP_EKF;
float ALT_POS_BMP_UKF_OLDX,ALT_VEL_BMP_UKF_OLDX;


double P_baro[9]={1,0,0,0,1,0,0,0,1}; 
double X_ukf_baro[3];
double P_barob[16]={1,0,0,0,1,0,0,0,1}; 
double X_ukf_barob[4];
//------------------KF  parameter------------------
float gh=0.15;
float ga=0.1;
float gwa=0.1;
double P_kf_baro[9]={1,0,0,1,0,0,1,0,0}; 
double X_kf_baro[3];

//float r_baro_ukf[3]={1,1,1};float q_baro_ukf[3]={0.01,0.01,0.01};
float r_baro_ukf[4]={10,1,0.1,0.1};float q_baro_ukf[4]={0.001,0.001,0.001,0.001};

float dead_accz=0.06;
float acc_off_baro=0;
float acc_scale_bmp=1;
float k_flt_accz=0.75;
float acc_bmp;

float X_apo_height[2] = {0.0f, 0.0f};
float P_apo_k_height[4] = {100.0f,0.0f,0.0f,100.0f};
float k_bais=  0.0;
float k_bais2= 0;
float r_baro = 10;//10; // 10.0f;			
float r_acc =  0.1; // 0.5f;
float x_tst[2];
float p_tst[2]={1,1};
float kf_tst[2]={1,1};

void body_to_NEZ(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
}

void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q) {
    float qc[4];

    qc[0] = q[0];
    qc[1] = -q[1];
    qc[2] = -q[2];
    qc[3] = -q[3];

    body_to_NEZ(vr, v, qc);
}

static void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[1*3 + 0]*v[1] + m[2*3 + 0]*v[2];
    vr[1] = m[0*3 + 1]*v[0] + m[1*3 + 1]*v[1] + m[2*3 + 1]*v[2];
    vr[2] = m[0*3 + 2]*v[0] + m[1*3 + 2]*v[1] + m[2*3 + 2]*v[2];
}

float k_acc_bais=0;
float acc_body[3];
float acc_bias[3];
float w_z_baro=0.5;
float w_acc_bias=0.05;
float accel_bias_corr[3];


int flag_ero=1;
float Alt_Offset_m1;
int en_bias_fix=0;
float flt_body_acc=0.5;
float k_body_acc=0.3;
float acc_est,acc_est_imu;
void ukf_baro_task1(float T)// 气压计加速度计融合
{
static u8 init;
if(!init)
{

}	
float posz;
if(NAV_BOARD_CONNECT)
posz=(float)(baroAlt_fc)/1000.;
else
posz=(float)baroAlt_fc/1000.;
float posz_flt=(float)baro_only_move_flt/1000.;
static float temp_r;
u8 i,j;
float acc_temp1,temp;  
float accIn[3];
float acc_body_temp[3];
 		accIn[0] =(float) mpu6050_fc.Acc.x/4096.*9.8-acc_bias[0]*en_bias_fix;//16438.;
		accIn[1] =(float) mpu6050_fc.Acc.y/4096.*9.8-acc_bias[1]*en_bias_fix;//16438.;
		accIn[2] =(float) mpu6050_fc.Acc.z/4096.*9.8-acc_bias[2]*en_bias_fix;//16438.;
    body_to_NEZ(acc_body_temp, accIn, ref_q_imd_down_fc);

    acc_body[0]=flt_body_acc*acc_body_temp[1]*k_body_acc+(1-flt_body_acc)*acc_body[0];
    acc_body[1]=flt_body_acc*acc_body_temp[0]*k_body_acc+(1-flt_body_acc)*acc_body[1];
    //acc_temp1=my_deathzoom(((float)(reference_vr_fc[2] *mpu6050_fc.Acc.z + reference_vr_fc[0] *mpu6050_fc.Acc.x + reference_vr_fc[1 ] *mpu6050_fc.Acc.y - 4096  )/4096.0f) *9.8,dead_accz);
    acc_body[2]=(float)(reference_vr_imd_down_fc[2] *mpu6050_fc.Acc.z + reference_vr_imd_down_fc[0] *mpu6050_fc.Acc.x + reference_vr_imd_down_fc[1] *mpu6050_fc.Acc.y - 4096  )*k_flt_accz/4096.0f*9.8+(1-k_flt_accz)*temp_r;
		
		//acc_body[2] =flt_body_acc*acc_body_temp[2]*1+(1-flt_body_acc)*acc_body[2];
		acc_temp1=acc_body[2];
    float corr_baro = flag_ero*( posz- ALT_POS_BMP_UKF_OLDX);
		accel_bias_corr[2] -= corr_baro * w_z_baro * w_z_baro;
    float R_control_now1[9];
		R_control_now1[0]=R_control_now[0][0];R_control_now1[3]=R_control_now[0][1];R_control_now1[6]=R_control_now[0][2];
		R_control_now1[1]=R_control_now[1][0];R_control_now1[4]=R_control_now[1][1];R_control_now1[7]=R_control_now[1][2];
		R_control_now1[2]=R_control_now[2][0];R_control_now1[5]=R_control_now[2][1];R_control_now1[8]=R_control_now[2][2];
		/* transform error vector from NED frame to body frame */
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += PX4_R(R_control_now1, j, i)*accel_bias_corr[j];
			}

			if (isfinite(c)) {
				acc_bias[i] += c * w_acc_bias * T;
			}
		}
		
		//if(fabs(Pit_fc)<5&&fabs(Rol_fc)<5)
			acc_bias[2]=0;
		
		//temp_r += ( 1 / ( 1 + 1 / ( 10 *3.14f *T ) ) ) *my_deathzoom( (temp_r - acc_temp1),0 );
    static float acc_bais;
		//acc_bais-=X_ukf_barob[3];//(ALT_POS_BMP_UKF_OLDX-posz)*k_acc_bais;
	  acc_bmp=LIMIT(my_deathzoom(acc_temp1-acc_off_baro,dead_accz)*acc_scale_bmp,-3.6,3.6);//+LIMIT(acc_bais,-1.5,1.5);
    // rotate acc to world frame
   	#if !DEBUG_WITHOUT_SB
		#if USE_RECIVER_MINE==1
		if(Rc_Get.THROTTLE<1200&&NS==2||ero.baro_ekf)		
		#else
		if(Rc_Get_PWM.THROTTLE<1111&&NS==2||ero.baro_ekf)		
		#endif
		{
		if(!ero.baro_ekf)	
		acc_off_baro=LIMIT(acc_temp1,-3,3);
		X_ukf_baro[0] =posz;X_ukf_baro[1]=X_ukf_baro[2]=0;acc_bais=0;
		X_kf_baro[0] =posz;X_kf_baro[1]=X_kf_baro[2]=0;
		X_apo_height[0] =posz;X_apo_height[1]=0;
		ero.baro_ekf=0;
		}
   	#endif 
	//#define BARO_UKF 
	#define BARO_KF	
		
	#if defined(BARO_UKF) //UKF

	#elif  defined(BARO_KF) //KF
	double Z_kf[3]={posz,0,0};
	kf_oldx( X_kf_baro,  P_kf_baro,  Z_kf,  acc_bmp, gh,  ga,  gwa,T);
	ALT_POS_BMP_UKF_OLDX=X_kf_baro[0];//Moving_Median(21,5,X_ukf_baro[0]);
	ALT_VEL_BMP_UKF_OLDX=X_kf_baro[1];//Moving_Median(22,5,X_ukf_baro[1]);
	#else  //EKF  
	float Z_baro_ekf[2]={posz,acc_bmp};		
	BARO_EKF_OLDX(X_apo_height,P_apo_k_height, X_apo_height, P_apo_k_height ,Z_baro_ekf,  r_baro,  r_acc, T);
	ALT_POS_BMP_UKF_OLDX=X_apo_height[0];//Moving_Median(21,5,X_ukf_baro[0]);
	ALT_VEL_BMP_UKF_OLDX=X_apo_height[1];//Moving_Median(22,5,X_ukf_baro[1]);
	#endif
	
	#if USE_RECIVER_MINE
	if(Rc_Get.THROTTLE<1200&&NS==2)	
	#else
	if(Rc_Get_PWM.THROTTLE<1200&&NS==2)	
	#endif
	#if !DEBUG_WITHOUT_SB
	Alt_Offset_m1=ALT_POS_BMP_UKF_OLDX;
	#endif
	x_tst[0]=kf_tst[0]*x_tst[0]+(1-kf_tst[0])*ALT_VEL_BMP_UKF_OLDX;
	
	static float spd_r,spd_r_imu;
	
	acc_est=(ALT_VEL_BMP_UKF_OLDX-spd_r)/T;
	spd_r=ALT_VEL_BMP_UKF_OLDX;
	if(fabs(acc_est>5)||fabs(ALT_VEL_BMP_UKF_OLDX)>8)	
  {ero.baro_ekf=1;ero.baro_ekf_cnt++;}
	

}