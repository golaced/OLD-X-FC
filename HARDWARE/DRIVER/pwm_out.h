#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_
#include "../HARDWARE/define.h"
u8 PWM_Out_Init(uint16_t hz);
u8 PWM_DJ_Init(uint16_t hz);
void SetPwm(int16_t pwm[],s16 min,s16 max);
void  Set_DJ(float ang1,float ang2,float ang3,float ang4);
extern u16 PWM_dj[4];

void LED_Init();
void LEDRGB(void);
#endif

