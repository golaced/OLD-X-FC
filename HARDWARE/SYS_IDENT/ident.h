#include "RELS.h"
#include "CFFRELS.h"
#include "PPCFFRELS.h"
#include "STC_PP.h"
#include "STC_PP_PID.h"

extern float CEPower ;
extern float CERoll ;
extern float CEPitch ;
extern float CEYaw  ;


//#define _IDENTIFICATION_PITCH_  //取消这句的注释来输出噪声数据用于俯仰角控制的系统辨识，电机解锁时才输出
//#define _IDENTIFICATION_ROLL_  //取消这句的注释来输出噪声数据用于滚转角控制的系统辨识，电机解锁时才输出
//#define _IDENTIFICATION_YAW_  //取消这句的注释来输出噪声数据用于偏航角控制的系统辨识，电机解锁时才输出
#define _IDENTIFICATION_ALL_WITH_CONTROL_  //取消这句的注释来输出噪声数据用于3个欧拉角控制的系统辨识，其间有飞控的反馈控制和手动控制的影响，电机解锁时才输出


//定义存储的控制用数据的长度（历史和当前，当前的索引为0），这个值一定得大于2，因为默认U有一阶延迟
#define I_CONTROL_DATA_LENGTH 15  

//递推增广最小二乘估计的相关定义
#define  RELS_NA 1       //a的阶数减一
#define  RELS_NB 1       //b的阶数减一
#define  RELS_NC 0       //c的阶数减一
#define  RELS_D 1       //d延迟

//递推增广最小二乘估计的相关定义
#define  PPCFFRELS_NA 1            //a的阶数减一
#define  PPCFFRELS_NB 1            //b的阶数减一
#define  PPCFFRELS_D 1             //d延迟
#define  PPCFFRELS_AM1 1           //Am的第1个参数
#define  PPCFFRELS_AM2 -1.3        //Am的第2个参数
#define  PPCFFRELS_AM3 0.48        //Am的第3个参数
#define  PPCFFRELS_LAMBDA 0.995    //遗忘因子



void ident(void);