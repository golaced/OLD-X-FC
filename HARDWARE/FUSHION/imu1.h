//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include "../HARDWARE/parameter.h"
#include "../HARDWARE/MATH/my_math.h"
#include "math.h"

typedef struct 
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;
	
}ref_t;

extern xyz_f_t reference_v;
extern float reference_vr[3],reference_vr_fc[3];
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float *rol,float *pit,float *yaw);


extern float Roll,Pitch,Yaw,yaw_mag_view[5];
extern float Roll_mid_down,Pitch_mid_down,Yaw_mid_down;    				//вкл╛╫г
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) ;
extern float ref_q[4] , q_nav[4];
extern float ref_q_imd_down[4];
extern float reference_vr_imd_down[3],yaw_kf;
extern double X_kf_yaw[2];
//----------------------------------------------------------------------------------------------------
// Variable declaration
extern float Pit_fc,Rol_fc,Yaw_fc,Yaw_fc_q;
extern volatile float beta;				// algorithm gain
extern volatile float q0_m, q1_m, q2_m, q3_m;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float dt,float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float *rol,float *pit,float *yaw);
void MadgwickAHRSupdateIMU(float dt,float gx, float gy, float gz, float ax, float ay, float az);
extern float ref_q_imd_down_fc[4] ;
extern float reference_vr_imd_down_fc[3];

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)


//---------------------------------------------------------------------------------------------------
// Function declarations

extern void MahonyAHRSupdate(float dt ,float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz
	,float *rol,float *pit,float *yaw);
extern void MahonyAHRSupdateIMU(float dt,float gx, float gy, float gz, float ax, float ay, float az);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
