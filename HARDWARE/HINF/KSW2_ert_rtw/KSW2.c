/*
 * File: KSW2.c
 *
 * Code generated for Simulink model 'KSW2'.
 *
 * Model version                  : 1.8
 * Simulink Coder version         : 8.7 (R2014b) 08-Sep-2014
 * C/C++ source code generated on : Fri Dec 16 10:39:35 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objective: Execution efficiency
 * Validation result: Passed (8), Warnings (4), Error (0)
 */

#include "KSW2.h"
#include "KSW2_private.h"

/* Block states (auto storage) */
DW_KSW2_T KSW2_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_KSW2_T KSW2_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_KSW2_T KSW2_Y;

/* Real-time model */
RT_MODEL_KSW2_T KSW2_M_;
RT_MODEL_KSW2_T *const KSW2_M = &KSW2_M_;

/* Model step function */
void KSW2_step(void)
{
  real_T DiscreteTransferFcn_tmp;

  /* DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' incorporates:
   *  Inport: '<Root>/In1'
   */
  DiscreteTransferFcn_tmp = ((((KSW2_U.In1 - KSW2_P.DiscreteTransferFcn_DenCoef
    [1] * KSW2_DW.DiscreteTransferFcn_states[0]) -
    KSW2_P.DiscreteTransferFcn_DenCoef[2] * KSW2_DW.DiscreteTransferFcn_states[1])
    - KSW2_P.DiscreteTransferFcn_DenCoef[3] *
    KSW2_DW.DiscreteTransferFcn_states[2]) - KSW2_P.DiscreteTransferFcn_DenCoef
    [4] * KSW2_DW.DiscreteTransferFcn_states[3]) /
    KSW2_P.DiscreteTransferFcn_DenCoef[0];

  /* Outport: '<Root>/Out1' incorporates:
   *  DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn'
   */
  KSW2_Y.Out1 = (((KSW2_P.DiscreteTransferFcn_NumCoef[0] *
                   DiscreteTransferFcn_tmp + KSW2_P.DiscreteTransferFcn_NumCoef
                   [1] * KSW2_DW.DiscreteTransferFcn_states[0]) +
                  KSW2_P.DiscreteTransferFcn_NumCoef[2] *
                  KSW2_DW.DiscreteTransferFcn_states[1]) +
                 KSW2_P.DiscreteTransferFcn_NumCoef[3] *
                 KSW2_DW.DiscreteTransferFcn_states[2]) +
    KSW2_P.DiscreteTransferFcn_NumCoef[4] * KSW2_DW.DiscreteTransferFcn_states[3];

  /* Update for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' */
  KSW2_DW.DiscreteTransferFcn_states[3] = KSW2_DW.DiscreteTransferFcn_states[2];
  KSW2_DW.DiscreteTransferFcn_states[2] = KSW2_DW.DiscreteTransferFcn_states[1];
  KSW2_DW.DiscreteTransferFcn_states[1] = KSW2_DW.DiscreteTransferFcn_states[0];
  KSW2_DW.DiscreteTransferFcn_states[0] = DiscreteTransferFcn_tmp;
}

/* Model initialize function */
void KSW2_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(KSW2_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&KSW2_DW, 0,
                sizeof(DW_KSW2_T));

  /* external inputs */
  KSW2_U.In1 = 0.0;

  /* external outputs */
  KSW2_Y.Out1 = 0.0;

  /* InitializeConditions for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' */
  KSW2_DW.DiscreteTransferFcn_states[0] = KSW2_P.DiscreteTransferFcn_InitialStat;
  KSW2_DW.DiscreteTransferFcn_states[1] = KSW2_P.DiscreteTransferFcn_InitialStat;
  KSW2_DW.DiscreteTransferFcn_states[2] = KSW2_P.DiscreteTransferFcn_InitialStat;
  KSW2_DW.DiscreteTransferFcn_states[3] = KSW2_P.DiscreteTransferFcn_InitialStat;
}

/* Model terminate function */
void KSW2_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
