/*
 * File: baro_ukf_oldx_bias.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 03-Nov-2016 17:13:22
 */

#ifndef __BARO_UKF_OLDX_BIAS_H__
#define __BARO_UKF_OLDX_BIAS_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "baro_ukf_oldx_bias_types.h"

/* Function Declarations */
extern void baro_ukf_oldx_bias(const double ffunr[16], double Xr[4], double Pr
  [16], const double Zr[4], const double Qr[16], const double Rr[16], double T);
extern void baro_ukf_oldx_bias_initialize(void);
extern void baro_ukf_oldx_bias_terminate(void);

#endif

/*
 * File trailer for baro_ukf_oldx_bias.h
 *
 * [EOF]
 */
