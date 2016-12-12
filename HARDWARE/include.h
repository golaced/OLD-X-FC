#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>
 
#include "delay.h" 
#include "malloc.h"   
#include "../HARDWARE/DRIVER/time.h"
#include "../HARDWARE/MEMS/mpu6050.h"
#include "../HARDWARE/parameter.h"
#include "../HARDWARE/DRIVER/usart_fc.h"   					
#include "../HARDWARE/DRIVER/rc_mine.h"												
#include "../HARDWARE/MEMS/ultrasonic.h"
#include "../HARDWARE/DRIVER/flash.h"
#include "../HARDWARE/DRIVER/dma.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/CONTROL/height_ctrl.h"
#include "../HARDWARE/CONTROL/visual_contol.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include "../HARDWARE/FUSHION/alt_kf.h"
#include "../HARDWARE/error.h"
#include "../HARDWARE/MEMS/hml_sample.h"
#include "../HARDWARE/MEMS/ms5611_2.h"
#include "../HARDWARE/FUSHION/alt_kf.h"
#include "../HARDWARE/DRIVER/flash.h"
#include "../HARDWARE/ucos_task.h"
#include "../HARDWARE/CONTROL/eso.h"
#include "../HARDWARE/CONTROL/neuron_pid.h"
#include "../HARDWARE/CONTROL/sonar_avoid.h"
#include "../HARDWARE/CONTROL/visual_contol.h"
#include "../HARDWARE/CONTROL/h_inf.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/CONTROL/height_ctrl.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include "../HARDWARE/MATH/Quaternion.h"
#include "../HARDWARE/MATH/miniMatrix.h"
#include "../HARDWARE/DRIVER/pwm_out.h"
/***************中断优先级******************/
#define NVIC_GROUP NVIC_PriorityGroup_2		//中断分组选择
/***********************************************/
//================传感器  模块选择===================
//#define USE_US100           //使用us100型号超声波 
#define USE_KS103					//使用ks103型号超声波
//#define SONAR_SAMPLE1					//0-5m 32ms  no fix
#define SONAR_SAMPLE2					//0-5m 100ms T fix
//#define SONAR_SAMPLE3					//0-11m 68ms no fix

//#define SONAR_USE_SCL  
//#define SONAR_USE_TIG
#define SONAR_USE_UART    

#define SONAR_HEIGHT 80   //超声波安装高度

#define USE_PXY 0    //未使用勿修改

#define DRONE_330_ID 3145777//未使用勿修改
#define DRONE_350_ID 4915281//未使用勿修改

#define USE_RC_GROUND 1  //使用NRF-G模块

#define IMU_HML_ADD_500 1//新的磁力融合
#define USE_BLE_FOR_APP 1//直接使用FC蓝牙与APP通信
//===============================系统时钟宏定义==========
#define A_X 0
#define A_Y 1
#define A_Z 2
#define G_Y 3
#define G_X 4
#define G_Z 5
#define TEM 6
#define ITEMS 7

#define ROLr 0
#define PITr 1
#define THRr 2
#define YAWr 3
#define AUX1r 4 
#define AUX2r 5
#define AUX3r 6
#define AUX4r 7

#define X 0
#define Y 1

#define East 0
#define North 1
//=======================参数宏定义==========
#define RtA 		57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.03051756				
#define Gyro_Gr	0.0005426

#define PI                     3.14159265359f //圆周率
#define RAD_DEG            		 57.2957795056f //弧度转化成角度的比例因子
#define DEG_RAD                0.01745329252f //角度转化成弧度的比例因子
#define GRAVITY_MSS                  9.80665f //地球重力加速度
#define earthRate                0.000072921f //地球自转角速度
#define earthRadius                6378145.0f //地球半径
#define earthRadiusInv          1.5678540e-7f //地球半径的倒数

#define TO_ANGLE 				0.06103f 		//0.061036 //   4000/65536  +-2000   ???

#define FIX_GYRO_Y 			1.02f				//陀螺仪Y轴固有补偿
#define FIX_GYRO_X 			1.02f				//陀螺仪X轴固有补偿

#define TO_M_S2 				0.23926f   	//   980cm/s2    +-8g   980/4096
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度

#define MAX_ACC  4096.0f						//+-8G		加速度计量程
#define TO_DEG_S 500.0f      				//T = 2ms  默认为2ms ，数值等于1/T
//================控制=====================
#define MAX_CTRL_ANGLE			25.0f										//遥控能达到的最大角度
#define ANGLE_TO_MAX_AS 		30.0f										//角度误差N时，期望角速度达到最大（可以通过调整CTRL_2的P值调整）
#define CTRL_2_INT_LIMIT 		0.5f *MAX_CTRL_ANGLE		//外环积分幅度
#define MAX_FIX_ANGLE 6

#define MAX_CTRL_ASPEED 	 	300.0f									//ROL,PIT允许的最大控制角速度
#define MAX_CTRL_YAW_SPEED 	60.0f									//YAW允许的最大控制角速度
#define CTRL_1_INT_LIMIT 		0.8f *MAX_CTRL_ASPEED		//内环积分幅度//0.5
//================矢量舵机控制=======================未使用现在
#define MAX_DJ_ANGLE 30  
#define SCALE_DJ 0.5
extern float dj_angle_set,dj_angle_offset[3],Angle_Yun[2];
extern float off_yaw;//遥控方向
//=================PWM========================
#define MAX_PWM				100			///%	最大PWM输出为100%油门
#define MAX_THR       80 			///%	油门通道最大占比80%，留20%给控制量
#define READY_SPEED   20      ///%	解锁后电机转速20%油门
//================系统设置===================
#define EN_TIM_INNER  0 //400Hz控制  有BUG请勿使用
#define EN_ATT_CAL_FC 1 //由飞控计算 IMU数据
#define EN_TIM_IMU  0     //未使用现在
#define USE_RECIVER_MINE 0//使用自己的手柄 未使用现在
#define BLE_BAD 0        //未使用现在
#define NEW_FLY_BOARD 0  //0—>PWM使用5678   未使用现在
#define PLANE_IS_BIG  0  //0->穿越机  				未使用现在 勿修改
#define USE_CYCLE_HML_CAL  0//0->使用椭球拟合
#define DEBUG_WITHOUT_SB 0	//调试时不需要手柄 上电2s后自动解锁
#define USE_TOE_IN_UNLOCK 0 //   未使用现在 勿修改
#define DRONE_X6 0  //PWM输出模式  未使用现在 勿修改
#define WIN8 0      //使用8寸桨  未使用现在 勿修改
//============== DMA使能=========================
#define EN_DMA_UART1 1  //UPLOAD
#define EN_DMA_UART2 0  //FLOW
#define EN_DMA_UART3 0  //GPS
#define EN_DMA_UART4 1 //SD
//===============机型选择====================
//#define   ZHOU_550          
//#define   ZHOU_350
#define   ZHOU_300

extern u32 mcuID[3];
extern u8 fly_ready,force_Thr_low;



enum
{
	SETBIT0 = 0x0001,  SETBIT1 = 0x0002,	SETBIT2 = 0x0004,	 SETBIT3 = 0x0008,
	SETBIT4 = 0x0010,	 SETBIT5 = 0x0020,	SETBIT6 = 0x0040,	 SETBIT7 = 0x0080,
	SETBIT8 = 0x0100,	 SETBIT9 = 0x0200,	SETBIT10 = 0x0400, SETBIT11 = 0x0800,
	SETBIT12 = 0x1000, SETBIT13 = 0x2000,	SETBIT14 = 0x4000, SETBIT15 = 0x8000		
};
//CLR BIT.    Example: a &= CLRBIT0
enum
{
	CLRBIT0 = 0xFFFE,  CLRBIT1 = 0xFFFD,	CLRBIT2 = 0xFFFB,	 CLRBIT3 = 0xFFF7,	
	CLRBIT4 = 0xFFEF,	 CLRBIT5 = 0xFFDF,	CLRBIT6 = 0xFFBF,	 CLRBIT7 = 0xFF7F,
	CLRBIT8 = 0xFEFF,	 CLRBIT9 = 0xFDFF,	CLRBIT10 = 0xFBFF, CLRBIT11 = 0xF7FF,
	CLRBIT12 = 0xEFFF, CLRBIT13 = 0xDFFF,	CLRBIT14 = 0xBFFF, CLRBIT15 = 0x7FFF
};
//CHOSE BIT.  Example: a = b&CHSBIT0
enum
{
	CHSBIT0 = 0x0001,  CHSBIT1 = 0x0002,	CHSBIT2 = 0x0004,	 CHSBIT3 = 0x0008,
	CHSBIT4 = 0x0010,	 CHSBIT5 = 0x0020,	CHSBIT6 = 0x0040,	 CHSBIT7 = 0x0080,
	CHSBIT8 = 0x0100,	 CHSBIT9 = 0x0200,	CHSBIT10 = 0x0400, CHSBIT11 = 0x0800,
	CHSBIT12 = 0x1000, CHSBIT13 = 0x2000,	CHSBIT14 = 0x4000, CHSBIT15 = 0x8000		
};

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	无符号8位整型变量  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		有符号8位整型变量  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable 	无符号32位整型变量 */
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 		有符号32位整型变量 */
typedef float          fp32;                    /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double         fp64;                    /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */

extern u8 rc_thr_mid;
extern int16_t BLE_DEBUG[16];
#endif

