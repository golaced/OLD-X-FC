/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "FastMath.h"

void Quaternion_Normalize(float *q);
void Quaternion_FromEuler(float *q, float *rpy);
void Quaternion_ToEuler(float *q, float* rpy);
void Quaternion_FromRotationMatrix(float *R, float *Q);
void Quaternion_RungeKutta4(float *q, float *w, float dt, int normalize);
void Quaternion_From6AxisData(float* q, float *accel, float *mag);
void Quaternion_To_R(float *q, float R[3][3]);
void Eular_FromRotationMatrix(float R[3][3], float *rpy);
void Quaternion_ToNumQ( float *q, float *rpy);
void Quaternion_To_R2(float *q, float R[3][3]);

void q_to_dcm(float data[4],float dcm[3][3]);
void dcm_to_q(float q[4] ,float dcm[3][3]);
void euler_to_q(float angle[3],float q[4]) ;
void  q_to_euler(float data[4],float angle[3]);
void dcm_to_euler(float euler[3],float data[3][3] );
void cal_ero_outter_px4(void);
extern float ero_angle_px4[4];

extern float R_control_now[3][3];
#define PX4_ARRAY2D(_array, _ncols, _x, _y) (_array[_x * _ncols + _y])
#define PX4_R(_array, _x, _y) PX4_ARRAY2D(_array, 3, _x, _y)
#endif
