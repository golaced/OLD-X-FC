#ifndef _MultiRotor_rc_H_
#define _MultiRotor_rc_H_
#include "../HARDWARE/define.h"
void NRF_DataAnl(void);
void Nrf_Check_Event(void);
void NRF_Send_AF(void);
void NRF_Send_AE(void);
void NRF_Send_OFFSET(void);
void NRF_Send_PID(void);
void NRF_Send_ARMED(void);
void NRF_SEND_test(void);
extern unsigned int cnt_timer2;
extern void RC_Send_Task(void);
extern u8 key_rc[6];
extern u16 data_rate;
extern void CAL_CHECK(void);
extern u8 loss_nrf;
extern u32 cnt_loss_nrf;
extern float ypr_sb[3];
#define RX_PLOAD_WIDTH 32
extern u8 	NRF24L01_RXDATA[RX_PLOAD_WIDTH];		//nrf24l01??????
extern u8 	NRF24L01_TXDATA[RX_PLOAD_WIDTH];		//nrf24l01???????



#define CH_NUM 				(8) 	//接收机通道数量
void Fly_Ready(float T);
void RC_Duty(float , u16 * );
void Feed_Rc_Dog(u8 ch_mode);
void Mode_FC(void);

extern float CH_filter[];
extern s16 CH[];
extern u8 fly_ready,NS ;
extern u8 height_ctrl_mode,height_ctrl_mode_use ;
\
extern u16 RX_CH[CH_NUM],RX_CH_PWM[CH_NUM];
extern int RX_CH_FIX[4],RX_CH_FIX_PWM[4];


extern u8 is_lock;
extern u8 tx_lock;
extern u8 EN_FIX_GPSF;
extern u8 EN_FIX_LOCKWF;
extern u8 EN_CONTROL_IMUF;
extern u8 EN_FIX_INSF;
extern u8 EN_FIX_HIGHF;

extern u8 EN_FIX_GPS;
extern u8 EN_FIX_LOCKW;
extern u8 EN_CONTROL_IMU;
extern u8 EN_FIX_INS;
extern u8 EN_FIX_HIGH;
extern u8 EN_TX_GX;
extern u8 EN_TX_AX;
extern u8 EN_TX_HM;
extern u8 EN_TX_YRP;
extern u8 EN_TX_GPS;
extern u8 EN_TX_HIGH;
extern u8	(up_load_set);
extern u8	(up_load_pid);
extern u16 yaw_sb;
#endif
