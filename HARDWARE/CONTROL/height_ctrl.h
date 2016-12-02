#ifndef __HEIGHT_CTRL_H
#define __HEIGHT_CTRL_H

#include "stm32f4xx.h"
#define MAX_VERTICAL_SPEED_UP  2000										//最大上升速度mm/s
#define MAX_VERTICAL_SPEED_DW  1500										//最大下降速度mm/s
typedef struct
{
	float err;
	float err_old;
	float err_d;
	float err_i;
	float pid_out;

}_st_height_pid_v;

typedef struct
{
	float kp;
	float kd;
	float ki;

}_st_height_pid;

void Height_Ctrl(float T,float thr);
void Height_Ctrl_safe(float T,float thr);
void BMP_Ctrl(float T,float thr);
void Ultra_PID_Init(void);

void WZ_Speed_PID_Init(void);

void height_speed_ctrl(float T,float thr,float exp_z_speed,float );
void height_speed_ctrl_Safe(float T,float thr,float exp_z_speed,float );
void Baro_Ctrl(float T,float thr);

void Ultra_Ctrl(float T,float thr);
void Ultra_Ctrl_Safe(float T,float thr);
extern float ultra_ctrl_out;

extern float height_ctrl_out,ultra_speed_bmp,baro_only_move_flt;

extern u8 baro_ctrl_start;

extern float ultra_speed;
extern float exp_height_speed,exp_height;
extern float ultra_speed,bmp_speed;
extern float ultra_dis_lpf,bmp_dis_lpf;
extern float ultra_ctrl_out,ultra_ctrl_out_use,bmp_ctrl_out;
extern float wz_speed;
extern float wz_acc_mms2;
extern _st_height_pid wz_speed_pid;
extern _st_height_pid ultra_pid,ultra_pid_safe;
extern _st_height_pid_v ultra_ctrl,ultra_ctrl_safe;
extern  _st_height_pid ultra_pid_use;
extern void CtrlAlti(float dt,float alt_now,float v_now,float thr);//m  //50hz  =20ms
extern int Thr_mine;
extern int wz_acc_ukf;
extern _st_height_pid_v wz_speed_pid_v,wz_speed_pid_v_eso;
extern _st_height_pid wz_speed_pid,wz_speed_pid_safe;
extern float adrc_out,baro_only_move;
extern u8 hold_alt_flag;
#define ULTRA_SPEED 		 300    // mm/s
#define ULTRA_MAX_HEIGHT 2000   // mm
#define BMP_MAX_HEIGHT   4000   // mm
#define ULTRA_INT        300    // 积分幅度

extern float exp_spd_zv,thr_use,thr_in_view,wz_speed_pid_v_view;
void applyMultirotorAltHold(u16 thr,float T);
extern float baro_speed_ano;
#endif

