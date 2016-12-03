#include "../HARDWARE/include.h" 
#define NUM1 8

typedef struct 
{
  float ox[NUM1];
	float oy[NUM1];
	float oz[NUM1];
	float kx[NUM1];
	float ky[NUM1];
	float kz[NUM1];
	
	float Ox;
	float Oy;
	float Oz;
	float Kx;
	float Ky;
	float Kz;
	
	u8 state;
	u8 mode;
}HML_CAL;

extern HML_CAL cycle_hml_cal;
extern u8 HML_SAMPLE(u8 en,int hx,int hy,int hz,float pit ,float rol,float gx,float gy,float gz,float T);