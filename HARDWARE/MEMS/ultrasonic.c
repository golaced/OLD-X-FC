#include "../HARDWARE/include.h"
#include "../HARDWARE/MEMS/ultrasonic.h"
#include "../HARDWARE/DRIVER/usart_fc.h"
#include "../HARDWARE/error.h"
#include "../HARDWARE/MATH/my_math.h"
s8 ultra_start_f;
u8 ultra_ok = 1;
int ultra_distance,ultra_distance_r;
float ultra_delta;
double x_pred = 0.0f; //
double v_pred = 0.0f; //       
double x_post = 0.0f; //
double v_post = 0.0f; //
float sonar_raw = 0.0f;  // 
float scale_kal_sonar_v=0.2;
float sonar_filter(float hight,float dt_sonar)//³¬Éù²¨kalmanÂË²¨  Î´Ê¹ÓÃ
{float x_new;
 static float reg;
	float LPF_1=1;//0.75; 
	float MAX_SPEED=0.5;
	/* no data for long time */
	if (dt_sonar > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt_sonar * v_pred;
	v_pred = v_post;
   v_pred=limit_mine	(v_pred,MAX_SPEED);
	 if(fabs(v_pred) < 0.01)           \
    v_pred = 0;   
	 
	 v_pred=reg*(1-LPF_1)+v_pred*(LPF_1);
	 reg=v_pred;
	 x_new = hight;
	sonar_raw = x_new;
	x_post = x_pred +  0.8461f* (x_new - x_pred);//0.8461f
	v_post = v_pred +  6.2034f* (x_new - x_pred)*scale_kal_sonar_v;
  v_post=limit_mine(v_post,MAX_SPEED);
	  if(fabs(v_post) < 0.01)           \
    v_post = 0;   
	return x_pred;//m/s
}


double x_pred_bmp = 0.0f; // m   0
double v_pred_bmp = 0.0f; //       1
double x_post_bmp = 0.0f; // m    2
double v_post_bmp = 0.0f; // m/s  3
float sonar_filter_bmp(float hight,float dt_sonar)//ÆøÑ¹¼ÆkalmanÂË²¨  Î´Ê¹ÓÃ
{float x_new;
 static float reg;
	float LPF_1=0.75; 
	float MAX_SPEED=0.5;
	/* no data for long time */
	if (dt_sonar > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred_bmp = x_post_bmp + dt_sonar * v_pred_bmp;
	v_pred_bmp = v_post_bmp;
   v_pred_bmp=limit_mine	(v_pred_bmp,MAX_SPEED);
	 if(fabs(v_pred_bmp) < 0.01)           \
    v_pred_bmp = 0;   
	 
	 v_pred_bmp=reg*(1-LPF_1)+v_pred_bmp*(LPF_1);
	 reg=v_pred_bmp;
	 x_new = hight;
	x_post_bmp = x_pred_bmp +  0.91* (x_new - x_pred_bmp);//0.8461f
	v_post_bmp = v_pred_bmp +  6.2034f* (x_new - x_pred_bmp)*scale_kal_sonar_v;
  v_post_bmp=limit_mine(v_post_bmp,MAX_SPEED);
	  if(fabs(v_post_bmp) < 0.01)           \
    v_post_bmp = 0;   
	return x_pred_bmp;//m/s
}

