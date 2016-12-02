/*
 * File: baro_ukf_oldx.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 02-Nov-2016 13:48:07
 */

#ifndef __BARO_UKF_OLDX_H__
#define __BARO_UKF_OLDX_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "baro_ukf_oldx_types.h"

/* Function Declarations */
extern void BARO_UKF_OLDX(double Xr[3], double Pr[9], const double Zr[3], const
  double Qr[9], const double Rr[9], double T);
extern void baro_ukf_oldx_initialize(void);
extern void baro_ukf_oldx_terminate(void);

#endif

/*
 * File trailer for baro_ukf_oldx.h
 *
 * [EOF]
 */
