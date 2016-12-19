/*
 * File: W1.c
 *
 * Code generated for Simulink model 'W1'.
 *
 * Model version                  : 1.8
 * Simulink Coder version         : 8.7 (R2014b) 08-Sep-2014
 * C/C++ source code generated on : Fri Dec 16 10:35:18 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objective: Execution efficiency
 * Validation result: Passed (8), Warnings (4), Error (0)
 */

#include "W1.h"
#include "W1_private.h"

/* Block states (auto storage) */
DW_W1_T W1_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_W1_T W1_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_W1_T W1_Y;

/* Real-time model */
RT_MODEL_W1_T W1_M_;
RT_MODEL_W1_T *const W1_M = &W1_M_;

/* Model step function */
void W1_step(void)
{
  real_T DiscreteTransferFcn_tmp;

  /* DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' incorporates:
   *  Inport: '<Root>/In1'
   */
  DiscreteTransferFcn_tmp = (W1_U.In1 - W1_P.DiscreteTransferFcn_DenCoef[1] *
    W1_DW.DiscreteTransferFcn_states) / W1_P.DiscreteTransferFcn_DenCoef[0];

  /* Outport: '<Root>/Out1' incorporates:
   *  DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn'
   */
  W1_Y.Out1 = W1_P.DiscreteTransferFcn_NumCoef[0] * DiscreteTransferFcn_tmp +
    W1_P.DiscreteTransferFcn_NumCoef[1] * W1_DW.DiscreteTransferFcn_states;

  /* Update for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' */
  W1_DW.DiscreteTransferFcn_states = DiscreteTransferFcn_tmp;
}

/* Model initialize function */
void W1_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(W1_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&W1_DW, 0,
                sizeof(DW_W1_T));

  /* external inputs */
  W1_U.In1 = 0.0;

  /* external outputs */
  W1_Y.Out1 = 0.0;

  /* InitializeConditions for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' */
  W1_DW.DiscreteTransferFcn_states = W1_P.DiscreteTransferFcn_InitialStat;
}

/* Model terminate function */
void W1_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
