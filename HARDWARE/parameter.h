#ifndef _PARAMETER_H
#define	_PARAMETER_H

#include "../HARDWARE/define.h"

typedef struct
{
	float kp;
	float kd;
	float ki;
	float kdamp;

}pid_t;

typedef struct 
{
  float x;
	float y;
	float z;
}xyz_f_t;


typedef struct 
{
  s16 x;
	s16 y;
	s16 z;

}xyz_s16_t;


typedef union
{
	uint8_t raw_data[64];
	struct
	{
		xyz_f_t Accel;
		xyz_f_t Gyro;
		xyz_f_t Mag;
		uint32_t Baro;
		float Temperature;
	}Offset;
}sensor_setup_t;

typedef  struct{
pid_t roll;
pid_t pitch;	
pid_t yaw;	
 }pid_group_t;

typedef union
{
 uint8_t raw_data[128];
 struct{
 pid_group_t ctrl1;
 pid_group_t ctrl2;
 }groups;
}pid_setup_t;

struct _BODY
{
float l;//m
float mess_l;//kg
float mess_all;//kg
};

struct _BLDC
{
u8 num;
float gain;
float off;
float time;//s
float mess;//kg
u16 kv;
u8 power_level;	//x S
u8 paralle_1;	
u8 paralle_2;	
};
#define DRONE_N 0
#define DRONE_X 1
struct _DRONE
{u8 type;
 u8 use_drone_pid;
 struct _BODY body; 
 struct _BLDC bldc;
 pid_group_t ctrl1;
 pid_group_t ctrl2;	
};
extern struct _DRONE drone;


void Senser_Calibrat_Read(void);
void PID_Para_Read(void);
void Para_Init(void);

void Para_ResetToFactorySetup(void);
void Param_SavePID(void);
void Param_SaveAccelOffset(xyz_f_t *offset);
void Param_SaveGyroOffset(xyz_f_t *offset);
void Param_SaveMagOffset(xyz_f_t *offset);
void Parameter_Save(void);

#endif

