/*
 * File: W1.h
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

#ifndef RTW_HEADER_W1_h_
#define RTW_HEADER_W1_h_
#include <stddef.h>
#include <string.h>
#ifndef W1_COMMON_INCLUDES_
# define W1_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* W1_COMMON_INCLUDES_ */

#include "W1_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTransferFcn_states;   /* '<S1>/Discrete Transfer Fcn' */
} DW_W1_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T In1;                          /* '<Root>/In1' */
} ExtU_W1_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T Out1;                         /* '<Root>/Out1' */
} ExtY_W1_T;

/* Parameters (auto storage) */
struct P_W1_T_ {
  real_T DiscreteTransferFcn_NumCoef[2];/* Expression: [num_w1]
                                         * Referenced by: '<S1>/Discrete Transfer Fcn'
                                         */
  real_T DiscreteTransferFcn_DenCoef[2];/* Expression: [den_w1]
                                         * Referenced by: '<S1>/Discrete Transfer Fcn'
                                         */
  real_T DiscreteTransferFcn_InitialStat;/* Expression: 0
                                          * Referenced by: '<S1>/Discrete Transfer Fcn'
                                          */
};

/* Real-time Model Data Structure */
struct tag_RTM_W1_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern P_W1_T W1_P;

/* Block states (auto storage) */
extern DW_W1_T W1_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_W1_T W1_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_W1_T W1_Y;

/* Model entry point functions */
extern void W1_initialize(void);
extern void W1_step(void);
extern void W1_terminate(void);

/* Real-time Model object */
extern RT_MODEL_W1_T *const W1_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('c_generate/W1')    - opens subsystem c_generate/W1
 * hilite_system('c_generate/W1/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'c_generate'
 * '<S1>'   : 'c_generate/W1'
 */
#endif                                 /* RTW_HEADER_W1_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
